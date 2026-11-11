//! Survey add-on: volumes between two terrain models (GEOROZDIL).
//!
//! The existing terrain is the TIN on `GEO_DMT` (GEODMT), the design the TIN on `GEO_DMT_navrh`
//! (GEONAVRH), both 3DFACEs. Every pair of overlapping triangles is intersected exactly (both are
//! convex), and, when a boundary is given, clipped to its convex pieces too. Within such a piece
//! both surfaces are planes, so their difference is linear: it is split where it changes sign,
//! and each part's volume is its area times the difference at its centroid. Cut is where the
//! existing terrain lies above the design (material to dig out), fill where it lies below.
//!
//! The zero line, where the surfaces meet and cut turns into fill, is drawn on
//! `GEO_nulova_cara`.

use cadcraft_doc::{EntityKind, HAlign, Handle, LwPolyline, VAlign};
use cadcraft_geom::{PolyVertex, Vec2, Vec3};
use serde_json::{Value, json};

use super::geo_calc::{
    add, area_centroid, bbox, clip_half, clip_to_tri, ear_clip, ensure_layer, overlaps, path_of, plane, say, signed_area, text, tin_on,
};
use super::geo_dmt::{LAYER_DESIGN, LAYER_TIN, chain};
use super::{CommandSpec, bad};
use crate::{Accept, Input, Interactive, Prompt, Result, Session, Step};

pub const LAYER_VOLUMES: &str = "GEO_kubatury";
pub const LAYER_ZERO: &str = "GEO_nulova_cara";

/// Vertices of a boundary.
const MAX_BOUNDARY: usize = 20_000;
/// Triangle pairs (and boundary pieces) clipped at once.
const MAX_WORK: usize = 200_000_000;
/// Grid cells of the design index along one axis.
const MAX_CELLS: usize = 2_000;

pub(super) fn specs() -> Vec<CommandSpec> {
    vec![
        CommandSpec::new("georozdil", "Volumes Between Two Terrains...", run_georozdil)
            .menu(&["Geo", "Volumes Between Two Terrains..."])
            .alias(&["geodiff", "geokub2"])
            .params("{existing?: layer of the existing terrain's 3DFACEs (default GEO_DMT), design?: layer of the design's (default GEO_DMT_navrh), boundary?: handle of a closed polyline (default: wherever both models are), label?: bool (default true), zeroLine?: bool, draw where cut turns into fill (default true)} → {cut, fill, net, area, zeroLines, message}")
            .interactive(|_| Ok(Box::new(RozdilM))),
    ]
}

/// Cut, fill and the area both models cover, and the zero-line segments.
#[derive(Debug, Default)]
pub struct Diff {
    pub cut: f64,
    pub fill: f64,
    pub area: f64,
    /// Area-weighted centre of the covered area.
    pub centre: Vec2,
    pub zero: Vec<(Vec2, Vec2)>,
}

/// A triangle in plan, counter-clockwise, and the plane z = a·x + b·y + c through it.
struct Flat {
    t: [Vec2; 3],
    plane: (f64, f64, f64),
    bb: (Vec2, Vec2),
}

fn flats(tris: &[[Vec3; 3]], o: Vec2) -> Vec<Flat> {
    tris.iter()
        .filter_map(|t| {
            let t = t.map(|v| Vec3::new(v.x - o.x, v.y - o.y, v.z));
            let plane = plane(&t)?;
            let mut f = [t[0].xy(), t[1].xy(), t[2].xy()];
            if signed_area(&f) < 0.0 {
                f.swap(1, 2);
            }
            Some(Flat { bb: bbox(&f), t: f, plane })
        })
        .collect()
}

/// A uniform grid over the design's triangles, to find the ones under an existing one quickly.
struct Grid {
    lo: Vec2,
    cell: f64,
    nx: usize,
    ny: usize,
    cells: Vec<Vec<usize>>,
}

impl Grid {
    fn new(items: &[Flat]) -> std::result::Result<Grid, String> {
        let (mut lo, mut hi) = (Vec2::new(f64::INFINITY, f64::INFINITY), Vec2::new(f64::NEG_INFINITY, f64::NEG_INFINITY));
        for f in items {
            lo = Vec2::new(lo.x.min(f.bb.0.x), lo.y.min(f.bb.0.y));
            hi = Vec2::new(hi.x.max(f.bb.1.x), hi.y.max(f.bb.1.y));
        }
        let (w, h) = ((hi.x - lo.x).max(1e-6), (hi.y - lo.y).max(1e-6));
        let n = items.len().max(1) as f64;
        let cell = ((w * h / n).sqrt() * 2.0).max(w / MAX_CELLS as f64).max(h / MAX_CELLS as f64).max(1e-6);
        if !cell.is_finite() {
            return Err("the terrain model has no usable extent".into());
        }
        let nx = ((w / cell).ceil() as usize).clamp(1, MAX_CELLS);
        let ny = ((h / cell).ceil() as usize).clamp(1, MAX_CELLS);
        let mut g = Grid { lo, cell, nx, ny, cells: vec![Vec::new(); nx * ny] };
        let mut entries = 0usize;
        for (i, f) in items.iter().enumerate() {
            let (x0, y0, x1, y1) = g.range(f.bb);
            for y in y0..=y1 {
                for x in x0..=x1 {
                    entries += 1;
                    if entries > MAX_WORK {
                        return Err("the terrain models are too large to compare at once".into());
                    }
                    if let Some(c) = g.cells.get_mut(y * nx + x) {
                        c.push(i);
                    }
                }
            }
        }
        Ok(g)
    }

    fn range(&self, bb: (Vec2, Vec2)) -> (usize, usize, usize, usize) {
        let ix = |v: f64| (((v - self.lo.x) / self.cell).floor().max(0.0) as usize).min(self.nx - 1);
        let iy = |v: f64| (((v - self.lo.y) / self.cell).floor().max(0.0) as usize).min(self.ny - 1);
        (ix(bb.0.x), iy(bb.0.y), ix(bb.1.x), iy(bb.1.y))
    }
}

/// Volumes between the `existing` and `design` triangles, inside `boundary` when given.
pub fn between(existing: &[[Vec3; 3]], design: &[[Vec3; 3]], boundary: Option<&[Vec2]>) -> std::result::Result<Diff, String> {
    // Work near the origin: S-JTSK coordinates are ~10^6 m.
    let o = boundary.and_then(|b| b.first().copied()).or_else(|| existing.first().map(|t| t[0].xy())).unwrap_or(Vec2::ZERO);
    let ex = flats(existing, o);
    let de = flats(design, o);
    if ex.is_empty() || de.is_empty() {
        return Err("both terrain models need triangles".into());
    }
    let pieces: Option<Vec<([Vec2; 3], (Vec2, Vec2))>> = match boundary {
        None => None,
        Some(b) => {
            let shifted: Vec<Vec2> = b.iter().map(|v| Vec2::new(v.x - o.x, v.y - o.y)).collect();
            let p: Vec<_> = ear_clip(&shifted).into_iter().map(|t| (t, bbox(&t))).collect();
            if p.is_empty() {
                return Err("the boundary encloses no area".into());
            }
            Some(p)
        }
    };
    let grid = Grid::new(&de)?;
    let mut seen = vec![usize::MAX; de.len()];
    let mut d = Diff::default();
    let (mut cx, mut cy) = (0.0, 0.0);
    let mut work = 0usize;
    let mut add_piece = |poly: &[Vec2], e: &Flat, f: &Flat, d: &mut Diff| {
        let (ea, eb, ec) = e.plane;
        let (fa, fb, fc) = f.plane;
        // design − existing: positive where fill is needed.
        let h = |q: Vec2| (fa - ea) * q.x + (fb - eb) * q.y + (fc - ec);
        let (area, cen) = area_centroid(poly);
        if !(area > 0.0 && area.is_finite()) {
            return;
        }
        d.area += area;
        cx += cen.x * area;
        cy += cen.y * area;
        for (part, sign) in [(clip_half(poly, h), 1.0), (clip_half(poly, |q| -h(q)), -1.0)] {
            if part.len() < 3 {
                continue;
            }
            let (a, c) = area_centroid(&part);
            let v = a * h(c) * sign;
            if v > 0.0 {
                if sign > 0.0 {
                    d.fill += v;
                } else {
                    d.cut += v;
                }
            }
        }
        // Where the difference is zero inside this piece.
        let n = poly.len();
        let mut hits: Vec<Vec2> = Vec::new();
        for i in 0..n {
            let (Some(&a), Some(&b)) = (poly.get(i), poly.get((i + 1) % n)) else { continue };
            let (ha, hb) = (h(a), h(b));
            if (ha > 0.0) != (hb > 0.0) && (ha - hb).abs() > 1e-15 {
                let t = ha / (ha - hb);
                let p = Vec2::new(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y));
                if !hits.iter().any(|q| (q.x - p.x).hypot(q.y - p.y) < 1e-9) {
                    hits.push(p);
                }
            }
        }
        if let [p, q] = hits.as_slice() {
            d.zero.push((Vec2::new(p.x + o.x, p.y + o.y), Vec2::new(q.x + o.x, q.y + o.y)));
        }
    };
    for (ei, e) in ex.iter().enumerate() {
        let (x0, y0, x1, y1) = grid.range(e.bb);
        for y in y0..=y1 {
            for x in x0..=x1 {
                let Some(cell) = grid.cells.get(y * grid.nx + x) else { continue };
                for &fi in cell {
                    let Some(s) = seen.get_mut(fi) else { continue };
                    if *s == ei {
                        continue;
                    }
                    *s = ei;
                    let Some(f) = de.get(fi) else { continue };
                    if !overlaps(e.bb, f.bb) {
                        continue;
                    }
                    work += 1;
                    if work > MAX_WORK {
                        return Err("the terrain models are too large to compare at once".into());
                    }
                    let common = clip_to_tri(&e.t, &f.t);
                    if common.len() < 3 {
                        continue;
                    }
                    match &pieces {
                        None => add_piece(&common, e, f, &mut d),
                        Some(ps) => {
                            let cb = bbox(&common);
                            for (t, pb) in ps {
                                if !overlaps(cb, *pb) {
                                    continue;
                                }
                                work += 1;
                                if work > MAX_WORK {
                                    return Err("the terrain models are too large to compare at once".into());
                                }
                                let p = clip_to_tri(&common, t);
                                if p.len() >= 3 {
                                    add_piece(&p, e, f, &mut d);
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    if d.area > 0.0 {
        d.centre = Vec2::new(cx / d.area + o.x, cy / d.area + o.y);
    }
    Ok(d)
}

fn layer_param(p: &Value, key: &str, default: &str) -> Result<String> {
    match p.get(key) {
        None | Some(Value::Null) => Ok(default.to_string()),
        Some(Value::String(l)) if !l.is_empty() && l.chars().count() <= 64 => Ok(l.clone()),
        Some(_) => Err(bad("georozdil", format!("`{key}` must be a layer name"))),
    }
}

fn run_georozdil(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "georozdil";
    let space = s.space();
    let existing_layer = layer_param(p, "existing", LAYER_TIN)?;
    let design_layer = layer_param(p, "design", LAYER_DESIGN)?;
    let boundary = match p.get("boundary") {
        None | Some(Value::Null) => None,
        Some(v) => {
            let h = v
                .as_str()
                .and_then(Handle::parse_hex)
                .or_else(|| v.as_u64().map(Handle))
                .ok_or_else(|| bad(CMD, "`boundary` must be an object handle"))?;
            let (poly, _) = path_of(s, h, &space).ok_or_else(|| bad(CMD, "the boundary must be a closed polyline"))?;
            if poly.len() < 3 || poly.len() > MAX_BOUNDARY {
                return Err(bad(CMD, format!("the boundary needs 3 to {MAX_BOUNDARY} vertices")));
            }
            Some(poly)
        }
    };
    let existing = tin_on(s, &space, &existing_layer)?;
    if existing.is_empty() {
        return Err(bad(CMD, format!("no existing terrain on {existing_layer}: run GEODMT first")));
    }
    let design = tin_on(s, &space, &design_layer)?;
    if design.is_empty() {
        return Err(bad(CMD, format!("no design surface on {design_layer}: select the design's points and lines and run GEONAVRH")));
    }
    let d = between(&existing, &design, boundary.as_deref()).map_err(|e| bad(CMD, e))?;
    if d.area <= 0.0 {
        return Err(bad(CMD, "the two terrain models do not overlap (inside the boundary)"));
    }
    let mut zero_lines = 0;
    if p.get("zeroLine").and_then(Value::as_bool).unwrap_or(true) && !d.zero.is_empty() {
        ensure_layer(s, LAYER_ZERO, 1)?;
        for (v, closed) in chain(d.zero.clone()) {
            if v.len() < 2 {
                continue;
            }
            let pl = LwPolyline { vertices: v.into_iter().map(PolyVertex::new).collect(), closed, const_width: 0.0, elevation: 0.0, plinegen: false };
            add(s, &space, LAYER_ZERO, EntityKind::LwPolyline(pl))?;
            zero_lines += 1;
        }
    }
    let area = d.area;
    if p.get("label").and_then(Value::as_bool).unwrap_or(true) {
        ensure_layer(s, LAYER_VOLUMES, 6)?;
        let at = match &boundary {
            Some(b) => area_centroid(b).1,
            None => d.centre,
        };
        let size = (area.sqrt() / 25.0).clamp(0.2, 5.0);
        let lines = [
            "Kubatury mezi terény".to_string(),
            format!("Výkop: {:.1} m³", d.cut),
            format!("Násyp: {:.1} m³", d.fill),
            format!("Plocha: {area:.1} m²"),
        ];
        for (i, l) in lines.into_iter().enumerate() {
            let pos = Vec2::new(at.x, at.y + (1.5 - i as f64) * 1.6 * size);
            text(s, &space, LAYER_VOLUMES, pos, size, l, 0.0, (HAlign::Center, VAlign::Middle))?;
        }
    }
    Ok(json!({
        "cut": d.cut, "fill": d.fill, "net": d.cut - d.fill, "area": area, "zeroLines": zero_lines,
        "message": format!("Existing to design: cut {:.1} m³, fill {:.1} m³ (net {:+.1} m³) over {area:.1} m²", d.cut, d.fill, d.cut - d.fill),
    }))
}

struct RozdilM;

impl Interactive for RozdilM {
    fn name(&self) -> &'static str {
        "GEOROZDIL"
    }
    fn prompt(&self, _s: &Session) -> Prompt {
        Prompt::new("Select the boundary polyline, or Enter for wherever both models are", Accept::SELECT)
    }
    fn input(&mut self, s: &mut Session, i: Input) -> Result<Step> {
        let params = match i {
            Input::Cancel => return Ok(Step::Cancel),
            Input::Enter => json!({}),
            Input::Pick(hs) => match hs.first() {
                Some(h) => json!({ "boundary": h.hex() }),
                None => return Ok(Step::Continue),
            },
            _ => return Ok(Step::Continue),
        };
        let r = run_georozdil(s, &params);
        say(s, r);
        Ok(Step::Done)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cadcraft_doc::Face3d;

    /// A grid of `n` × `n` cells over 0..20, as triangles of z = f(x, y), split along one or the
    /// other diagonal.
    fn surface(n: usize, flip: bool, f: impl Fn(f64, f64) -> f64, shift: Vec2) -> Vec<[Vec3; 3]> {
        let step = 20.0 / n as f64;
        let v = |i: usize, j: usize| {
            let (x, y) = (i as f64 * step, j as f64 * step);
            Vec3::new(x + shift.x, y + shift.y, f(x, y))
        };
        let mut out = Vec::new();
        for i in 0..n {
            for j in 0..n {
                let (a, b, c, d) = (v(i, j), v(i + 1, j), v(i + 1, j + 1), v(i, j + 1));
                if flip {
                    out.push([a, b, d]);
                    out.push([b, c, d]);
                } else {
                    out.push([a, b, c]);
                    out.push([a, c, d]);
                }
            }
        }
        out
    }

    fn sq(x0: f64, y0: f64, x1: f64, y1: f64) -> Vec<Vec2> {
        vec![Vec2::new(x0, y0), Vec2::new(x1, y0), Vec2::new(x1, y1), Vec2::new(x0, y1)]
    }

    #[test]
    fn a_tilted_design_over_flat_ground() {
        // Flat ground at 100; the design rises 0.1 m per metre eastwards through 100 at x = 10.
        // West of x = 10 the design is lower (cut), east higher (fill): 20 × ∫0..10 0.1·t dt = 100.
        for shift in [Vec2::ZERO, Vec2::new(-742_000.0, -1_043_000.0)] {
            let ex = surface(1, false, |_, _| 100.0, shift);
            let de = surface(7, true, |x, _| 100.0 + 0.1 * (x - 10.0), shift);
            let d = between(&ex, &de, None).unwrap();
            assert!((d.cut - 100.0).abs() < 1e-6 && (d.fill - 100.0).abs() < 1e-6 && (d.area - 400.0).abs() < 1e-6, "{d:?}");
            // The zero line runs along x = 10.
            assert!(!d.zero.is_empty());
            for (p, q) in &d.zero {
                assert!((p.x - shift.x - 10.0).abs() < 1e-6 && (q.x - shift.x - 10.0).abs() < 1e-6, "{p:?} {q:?}");
            }
            let total: f64 = d.zero.iter().map(|(p, q)| (q.x - p.x).hypot(q.y - p.y)).sum();
            assert!((total - 20.0).abs() < 1e-6, "{total}");
            // The western half only: all cut.
            let b: Vec<Vec2> = sq(0.0, 0.0, 10.0, 20.0).iter().map(|v| Vec2::new(v.x + shift.x, v.y + shift.y)).collect();
            let d = between(&ex, &de, Some(&b)).unwrap();
            assert!((d.cut - 100.0).abs() < 1e-6 && d.fill.abs() < 1e-9 && (d.area - 200.0).abs() < 1e-6, "{d:?}");
        }
    }

    #[test]
    fn only_the_overlap_counts() {
        // The design covers x 10..30 only (shifted by 10): overlap 10 × 20, 1 m of fill.
        let ex = surface(4, false, |_, _| 100.0, Vec2::ZERO);
        let de = surface(3, false, |_, _| 101.0, Vec2::new(10.0, 0.0));
        let d = between(&ex, &de, None).unwrap();
        assert!((d.fill - 200.0).abs() < 1e-6 && d.cut.abs() < 1e-9 && (d.area - 200.0).abs() < 1e-6, "{d:?}");
        assert!(d.zero.is_empty());
    }

    fn faces(s: &mut Session, layer: &str, tris: &[[Vec3; 3]]) {
        let space = s.space();
        for t in tris {
            add(s, &space, layer, EntityKind::Face3d(Face3d { corners: [t[0], t[1], t[2], t[2]], hidden_edges: 0 })).unwrap();
        }
    }

    #[test]
    fn the_command_end_to_end() {
        let mut s = Session::new();
        assert!(s.execute("georozdil", &json!({})).is_err());
        faces(&mut s, LAYER_TIN, &surface(2, false, |_, _| 100.0, Vec2::ZERO));
        assert!(s.execute("georozdil", &json!({})).is_err());
        faces(&mut s, LAYER_DESIGN, &surface(3, true, |x, _| 100.0 + 0.1 * (x - 10.0), Vec2::ZERO));
        let r = s.execute("georozdil", &json!({})).unwrap();
        assert!((r["cut"].as_f64().unwrap() - 100.0).abs() < 1e-6, "{r}");
        assert!((r["fill"].as_f64().unwrap() - 100.0).abs() < 1e-6, "{r}");
        assert_eq!(r["zeroLines"], 1, "{r}");
        let d = s.doc().unwrap();
        assert_eq!(d.model.iter().filter(|e| e.common.layer == LAYER_ZERO).count(), 1);
        assert_eq!(d.model.iter().filter(|e| e.common.layer == LAYER_VOLUMES).count(), 4);
    }

    #[test]
    fn geonavrh_builds_the_design_layer() {
        let mut s = Session::new();
        let mut text = String::new();
        for i in 0..3 {
            for j in 0..3 {
                text.push_str(&format!("{} {} {} {}\n", i * 3 + j + 1, i * 10, j * 10, 100.0));
            }
        }
        s.execute("geobody", &json!({ "text": text, "sjtsk": false })).unwrap();
        s.execute("geodmt", &json!({})).unwrap();
        // Without a selection there is nothing to build the design from.
        s.set_selection(Vec::new());
        assert!(s.execute("geonavrh", &json!({})).is_err());
        let space = s.space();
        let mut hs = Vec::new();
        for (x, y) in [(0.0, 0.0), (20.0, 0.0), (20.0, 20.0), (0.0, 20.0)] {
            let pt = cadcraft_doc::Point { p: Vec3::new(x, y, 99.0), angle: 0.0 };
            hs.push(add(&mut s, &space, "navrh", EntityKind::Point(pt)).unwrap().hex());
        }
        let r = s.execute("geonavrh", &json!({ "handles": hs })).unwrap();
        assert_eq!(r["triangles"], 2, "{r}");
        // GEODMT's model is untouched.
        assert_eq!(s.doc().unwrap().model.iter().filter(|e| e.common.layer == LAYER_TIN).count(), 8);
        let r = s.execute("georozdil", &json!({ "label": false })).unwrap();
        assert!((r["cut"].as_f64().unwrap() - 400.0).abs() < 1e-6, "{r}");
        assert!(s.execute("geodmt", &json!({ "layer": "a/b" })).is_err());
        assert!(s.execute("geodmt", &json!({ "layer": 5 })).is_err());
    }

    #[test]
    fn hostile_input_is_refused_not_panicking() {
        let mut s = Session::new();
        for p in [json!({ "existing": 3 }), json!({ "design": "" }), json!({ "boundary": "zz" }), json!({ "boundary": [1] })] {
            assert!(s.execute("georozdil", &p).is_err(), "{p}");
        }
        let nan = [[Vec3::new(f64::NAN, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0)]];
        let ok = surface(1, false, |_, _| 0.0, Vec2::ZERO);
        let _ = between(&nan, &ok, None);
        let _ = between(&ok, &nan, None);
        let huge = [[Vec3::new(-1e300, -1e300, 0.0), Vec3::new(1e300, -1e300, 0.0), Vec3::new(0.0, 1e300, 0.0)]];
        let _ = between(&ok, &huge, None);
        let _ = between(&huge, &huge, Some(&sq(0.0, 0.0, 1.0, 1.0)));
        assert!(between(&ok, &ok, Some(&[Vec2::ZERO; 3])).is_err());
    }
}
