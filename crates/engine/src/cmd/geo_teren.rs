//! Survey add-on: more work on the terrain model.
//!
//! - GEOSPADNICE: a fall line (the way water runs) from a picked point down the TIN, following
//!   the steepest slope of each triangle until it reaches a hollow or the model's edge.
//! - GEOHRANA: flip the edge shared by two triangles (the diagonal of their quadrilateral), the
//!   usual hand correction where the automatic triangulation crossed a ridge or a valley.
//! - GEOMRIZKA: a grid of points at a spacing, with their heights from the model.
//! - GEOSTANICENI: station marks and labels along a line or polyline.
//! - GEOSTL: the terrain model as an STL file (for 3D printing or other programs).

use cadcraft_doc::{EntityKind, Face3d, HAlign, Handle, Line, Point, Polyline3d, VAlign};
use cadcraft_geom::{Vec2, Vec3};
use serde_json::{Value, json};

use super::geo_calc::{add, ensure_layer, height_at, one_target, path_of, plane, say, signed_area, text, tin};
use super::geo_dmt::LAYER_TIN;
use super::geo_rezy::station_text;
use super::{CommandSpec, bad, f64_or, point_value, str_param};
use crate::{Accept, Input, Interactive, Prompt, Result, Session, Step};

pub const LAYER_FALL: &str = "GEO_spadnice";
pub const LAYER_GRID: &str = "GEO_mrizka";
pub const LAYER_STATIONS: &str = "GEO_staniceni";

/// Steps of one fall line.
const MAX_STEPS: usize = 100_000;
/// Grid points at once.
const MAX_GRID: usize = 1_000_000;

pub(super) fn specs() -> Vec<CommandSpec> {
    vec![
        CommandSpec::new("geospadnice", "Fall Line", run_geospadnice)
            .menu(&["Geo", "Fall Line"])
            .alias(&["geofall"])
            .params("{at: [x, y] the start} → {length, from, to, points, message}: a 3D polyline down the terrain model on GEO_spadnice")
            .interactive(|_| Ok(Box::new(PickM { name: "GEOSPADNICE", cmd: "geospadnice", prompt: "Start point of the fall line" }))),
        CommandSpec::new("geohrana", "Flip Terrain Edge", run_geohrana)
            .menu(&["Geo", "Flip Terrain Edge"])
            .alias(&["geoflip"])
            .params("{at: [x, y] a point on (or next to) the edge to flip} → {handles, message}")
            .interactive(|_| Ok(Box::new(PickM { name: "GEOHRANA", cmd: "geohrana", prompt: "Pick the triangle edge to flip" }))),
        CommandSpec::new("geomrizka", "Grid of Terrain Points...", run_geomrizka)
            .menu(&["Geo", "Grid of Terrain Points..."])
            .alias(&["geogrid"])
            .params("{step?: spacing in metres (default 5), labels?: bool, write heights (default true), scale?: 1:N for labels (default 500)} → {points, message}")
            .interactive(|_| Ok(Box::new(NumberM { name: "GEOMRIZKA", cmd: "geomrizka", key: "step", prompt: "Grid spacing in metres", default: 5.0 }))),
        CommandSpec::new("geostaniceni", "Stationing...", run_geostaniceni)
            .menu(&["Geo", "Stationing..."])
            .alias(&["geostations"])
            .params("{path?: handle of a line or polyline (default: the selected one), step?: (default 10), start?: station of the first vertex (default 0), scale?: 1:N for text (default 500)} → {stations, length, message}")
            .interactive(|_| Ok(Box::new(NumberM { name: "GEOSTANICENI", cmd: "geostaniceni", key: "step", prompt: "Station spacing in metres (select the axis first)", default: 10.0 }))),
        CommandSpec::new("geostl", "Export STL...", run_geostl)
            .menu(&["Geo", "Export", "STL..."])
            .alias(&["geoexportstl"])
            .params("{path?: file to write (otherwise the text is returned), exaggeration?: vertical (default 1)} → {triangles, text?, message}"),
    ]
}

// --- GEOSPADNICE ---------------------------------------------------------------------------------

/// Index of the triangle containing `p`.
fn containing(tris: &[[Vec3; 3]], p: Vec2) -> Option<usize> {
    tris.iter().position(|t| {
        let (a, b, c) = (t[0].xy(), t[1].xy(), t[2].xy());
        let s = signed_area(&[a, b, c]).signum();
        let side = |u: Vec2, v: Vec2| ((v.x - u.x) * (p.y - u.y) - (v.y - u.y) * (p.x - u.x)) * s >= -1e-9;
        side(a, b) && side(b, c) && side(c, a)
    })
}

/// The fall line from `start`: points where it crosses triangle edges, down to a hollow or the edge.
pub fn fall_line(tris: &[[Vec3; 3]], start: Vec2) -> Vec<Vec3> {
    let mut out = Vec::new();
    let Some(mut ti) = containing(tris, start) else { return out };
    let mut p = start;
    let mut last_dir: Option<Vec2> = None;
    for _ in 0..MAX_STEPS {
        let Some(t) = tris.get(ti) else { break };
        let Some((a, b, c)) = plane(t) else { break };
        out.push(Vec3::new(p.x, p.y, a * p.x + b * p.y + c));
        let g = (a * a + b * b).sqrt();
        if g < 1e-9 {
            break; // flat: no way down
        }
        // Steepest descent.
        let d = Vec2::new(-a / g, -b / g);
        if let Some(l) = last_dir
            && d.x * l.x + d.y * l.y < -0.999
        {
            break;
        }
        // Exit through the triangle's boundary along d.
        let verts = [t[0].xy(), t[1].xy(), t[2].xy()];
        let mut best: Option<(f64, Vec2)> = None;
        for i in 0..3 {
            let (Some(&u), Some(&v)) = (verts.get(i), verts.get((i + 1) % 3)) else { continue };
            let (ex, ey) = (v.x - u.x, v.y - u.y);
            let den = d.x * ey - d.y * ex;
            if den.abs() < 1e-15 {
                continue;
            }
            let s = ((u.x - p.x) * ey - (u.y - p.y) * ex) / den;
            let r = ((u.x - p.x) * d.y - (u.y - p.y) * d.x) / den;
            if s > 1e-9 && (-1e-9..=1.0 + 1e-9).contains(&r) && best.is_none_or(|(bs, _)| s < bs) {
                best = Some((s, Vec2::new(p.x + s * d.x, p.y + s * d.y)));
            }
        }
        // No way on through this triangle (the descent points straight out of it): stop.
        let Some((_, q)) = best else { break };
        last_dir = Some(d);
        // Step just past the edge into the next triangle.
        let probe = Vec2::new(q.x + d.x * 1e-6, q.y + d.y * 1e-6);
        let next = tris.iter().enumerate().position(|(j, t)| {
            j != ti && {
                let (a, b, c) = (t[0].xy(), t[1].xy(), t[2].xy());
                let s = signed_area(&[a, b, c]).signum();
                let side = |u: Vec2, v: Vec2| ((v.x - u.x) * (probe.y - u.y) - (v.y - u.y) * (probe.x - u.x)) * s >= -1e-12;
                side(a, b) && side(b, c) && side(c, a)
            }
        });
        p = q;
        match next {
            Some(n) => ti = n,
            None => {
                if let Some((a, b, c)) = plane(t) {
                    out.push(Vec3::new(p.x, p.y, a * p.x + b * p.y + c));
                }
                break;
            }
        }
        // A hollow: the next triangle sends the water straight back.
        if let Some(nt) = tris.get(ti)
            && let Some((na, nb, _)) = plane(nt)
        {
            let ng = (na * na + nb * nb).sqrt();
            if ng > 1e-9 && (-na / ng) * d.x + (-nb / ng) * d.y < -0.2 {
                if let Some((a, b, c)) = plane(nt) {
                    out.push(Vec3::new(p.x, p.y, a * p.x + b * p.y + c));
                }
                break;
            }
        }
    }
    out.dedup_by(|a, b| (a.x - b.x).abs() < 1e-9 && (a.y - b.y).abs() < 1e-9);
    out
}

fn run_geospadnice(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "geospadnice";
    let at = p
        .get("at")
        .and_then(point_value)
        .filter(|a| a.x.is_finite() && a.y.is_finite())
        .ok_or_else(|| bad(CMD, "`at` ([x, y], the start) is required"))?;
    let space = s.space();
    let tris = tin(s, &space)?;
    if tris.is_empty() {
        return Err(bad(CMD, "no terrain model: run GEODMT first"));
    }
    let line = fall_line(&tris, at);
    if line.len() < 2 {
        return Err(bad(CMD, "the point is outside the terrain model, or on flat ground"));
    }
    let length: f64 = line.windows(2).map(|w| if let [a, b] = w { (b.x - a.x).hypot(b.y - a.y) } else { 0.0 }).sum();
    let (from, to) = (line.first().map_or(0.0, |v| v.z), line.last().map_or(0.0, |v| v.z));
    ensure_layer(s, LAYER_FALL, 5)?;
    let n = line.len();
    add(s, &space, LAYER_FALL, EntityKind::Polyline3d(Polyline3d { points: line, closed: false }))?;
    Ok(json!({ "length": length, "from": from, "to": to, "points": n, "message": format!("Fall line {length:.2} m from {from:.2} down to {to:.2}") }))
}

// --- GEOHRANA ------------------------------------------------------------------------------------

fn run_geohrana(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "geohrana";
    let at = p
        .get("at")
        .and_then(point_value)
        .filter(|a| a.x.is_finite() && a.y.is_finite())
        .ok_or_else(|| bad(CMD, "`at` (a point on the edge) is required"))?;
    let space = s.space();
    let faces: Vec<(Handle, [Vec3; 3])> = {
        let d = s.doc()?;
        let st = d.space(&space).ok_or_else(|| bad(CMD, "no drawing space"))?;
        st.iter()
            .filter(|e| e.common.layer.eq_ignore_ascii_case(LAYER_TIN))
            .filter_map(|e| if let EntityKind::Face3d(f) = &e.kind { Some((e.handle, [f.corners[0], f.corners[1], f.corners[2]])) } else { None })
            .collect()
    };
    // The edge nearest to the pick that two triangles share.
    let key = |v: Vec3| ((v.x * 1000.0).round() as i64, (v.y * 1000.0).round() as i64);
    let dist = |a: Vec2, b: Vec2| {
        let (dx, dy) = (b.x - a.x, b.y - a.y);
        let l2 = dx * dx + dy * dy;
        let t = if l2 > 0.0 { (((at.x - a.x) * dx + (at.y - a.y) * dy) / l2).clamp(0.0, 1.0) } else { 0.0 };
        (a.x + t * dx - at.x).hypot(a.y + t * dy - at.y)
    };
    let mut best: Option<(f64, usize, usize, Vec3, Vec3)> = None;
    let mut edges: std::collections::HashMap<((i64, i64), (i64, i64)), usize> = std::collections::HashMap::new();
    for (i, (_, t)) in faces.iter().enumerate() {
        for (a, b) in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
            let (ka, kb) = (key(a), key(b));
            let k = if ka < kb { (ka, kb) } else { (kb, ka) };
            if let Some(&j) = edges.get(&k) {
                let d = dist(a.xy(), b.xy());
                if best.is_none_or(|(bd, ..)| d < bd) {
                    best = Some((d, j, i, a, b));
                }
            } else {
                edges.insert(k, i);
            }
        }
    }
    let (_, i, j, a, b) = best.ok_or_else(|| bad(CMD, "no shared triangle edge here: run GEODMT first"))?;
    let (Some((hi, ti)), Some((hj, tj))) = (faces.get(i), faces.get(j)) else { return Err(bad(CMD, "triangles")) };
    let other = |t: &[Vec3; 3]| t.iter().copied().find(|v| key(*v) != key(a) && key(*v) != key(b));
    let (Some(c), Some(d)) = (other(ti), other(tj)) else { return Err(bad(CMD, "degenerate triangles")) };
    // The flip is only valid when the quadrilateral a-c-b-d is convex: c and d on either side of
    // the new diagonal's... i.e. a and b on either side of c–d.
    let side = |u: Vec2, v: Vec2, q: Vec2| (v.x - u.x) * (q.y - u.y) - (v.y - u.y) * (q.x - u.x);
    if side(c.xy(), d.xy(), a.xy()) * side(c.xy(), d.xy(), b.xy()) >= 0.0 {
        return Err(bad(CMD, "this edge can't be flipped: the two triangles don't make a convex quadrilateral"));
    }
    let (hi, hj) = (*hi, *hj);
    let d_mut = s.doc_mut()?;
    for (h, corners) in [(hi, [a, c, d]), (hj, [b, d, c])] {
        d_mut
            .modify_entity(h, |e: &mut cadcraft_doc::Entity| {
                if let EntityKind::Face3d(f) = &mut e.kind {
                    *f = Face3d { corners: [corners[0], corners[1], corners[2], corners[2]], hidden_edges: 0 };
                }
            })
            .map_err(|e| bad(CMD, e.to_string()))?;
    }
    Ok(json!({ "handles": [hi.hex(), hj.hex()], "message": "Edge flipped; run GEOVRSTEVNICE again to update the contours" }))
}

// --- GEOMRIZKA -----------------------------------------------------------------------------------

fn run_geomrizka(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "geomrizka";
    let step = f64_or(p, "step", 5.0);
    let scale = f64_or(p, "scale", 500.0);
    if !(step.is_finite() && step > 0.0 && scale.is_finite() && (1.0..=1e6).contains(&scale)) {
        return Err(bad(CMD, "`step` and `scale` must be positive"));
    }
    let space = s.space();
    let tris = tin(s, &space)?;
    if tris.is_empty() {
        return Err(bad(CMD, "no terrain model: run GEODMT first"));
    }
    let (lo, hi) =
        tris.iter().flatten().fold((Vec2::new(f64::INFINITY, f64::INFINITY), Vec2::new(f64::NEG_INFINITY, f64::NEG_INFINITY)), |(l, h), v| {
            (Vec2::new(l.x.min(v.x), l.y.min(v.y)), Vec2::new(h.x.max(v.x), h.y.max(v.y)))
        });
    let (x0, y0) = ((lo.x / step).ceil() * step, (lo.y / step).ceil() * step);
    let (nx, ny) = (((hi.x - x0) / step).floor() + 1.0, ((hi.y - y0) / step).floor() + 1.0);
    if !(nx.is_finite() && ny.is_finite()) || nx * ny > MAX_GRID as f64 {
        return Err(bad(CMD, "too many grid points: use a larger `step`"));
    }
    let labels = p.get("labels").and_then(Value::as_bool).unwrap_or(true);
    let th = 0.0015 * scale;
    ensure_layer(s, LAYER_GRID, 8)?;
    let mut n = 0;
    for i in 0..(nx.max(0.0) as usize) {
        for j in 0..(ny.max(0.0) as usize) {
            let q = Vec2::new(x0 + i as f64 * step, y0 + j as f64 * step);
            let Some(z) = height_at(&tris, q) else { continue };
            add(s, &space, LAYER_GRID, EntityKind::Point(Point { p: Vec3::new(q.x, q.y, z), angle: 0.0 }))?;
            if labels {
                text(s, &space, LAYER_GRID, Vec2::new(q.x + 0.5 * th, q.y + 0.3 * th), th, format!("{z:.2}"), 0.0, (HAlign::Left, VAlign::Baseline))?;
            }
            n += 1;
        }
    }
    Ok(json!({ "points": n, "message": format!("{n} grid points every {step} m") }))
}

// --- GEOSTANICENI --------------------------------------------------------------------------------

fn run_geostaniceni(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "geostaniceni";
    let space = s.space();
    let h = one_target(s, p, "path", CMD)?;
    let (path, _) = path_of(s, h, &space).ok_or_else(|| bad(CMD, "the axis must be a line or polyline"))?;
    let step = f64_or(p, "step", 10.0);
    let start = f64_or(p, "start", 0.0);
    let scale = f64_or(p, "scale", 500.0);
    if !(step.is_finite() && step > 0.0 && start.is_finite() && start.abs() < 1e7 && scale.is_finite() && (1.0..=1e6).contains(&scale)) {
        return Err(bad(CMD, "`step`, `start` and `scale` must be numbers (step and scale positive)"));
    }
    let length: f64 = path.windows(2).map(|w| if let [a, b] = w { (b.x - a.x).hypot(b.y - a.y) } else { 0.0 }).sum();
    if !(length.is_finite() && length > 0.0) || length / step > 100_000.0 {
        return Err(bad(CMD, "the axis has no length, or there would be too many stations"));
    }
    let th = 0.002 * scale;
    ensure_layer(s, LAYER_STATIONS, 1)?;
    // Stations at whole multiples of `step` in the axis's own stationing, plus its ends.
    let mut marks: Vec<f64> = Vec::new();
    let first = ((start / step).ceil() * step - start).max(0.0);
    let mut d = first;
    while d <= length + 1e-9 {
        marks.push(d);
        d += step;
    }
    if marks.first().is_none_or(|f| *f > 1e-9) {
        marks.insert(0, 0.0);
    }
    if marks.last().is_some_and(|l| length - l > 1e-6) {
        marks.push(length);
    }
    for m in &marks {
        // Point and direction at m.
        let mut left = *m;
        let mut frame = None;
        for w in path.windows(2) {
            let [a, b] = w else { continue };
            let len = (b.x - a.x).hypot(b.y - a.y);
            if len < 1e-12 {
                continue;
            }
            if left <= len + 1e-9 {
                let t = (left / len).clamp(0.0, 1.0);
                frame = Some((Vec2::new(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)), Vec2::new((b.x - a.x) / len, (b.y - a.y) / len)));
                break;
            }
            left -= len;
        }
        let Some((q, u)) = frame else { continue };
        let nrm = Vec2::new(-u.y, u.x);
        add(
            s,
            &space,
            LAYER_STATIONS,
            EntityKind::Line(Line { a: Vec3::new(q.x - nrm.x * th, q.y - nrm.y * th, 0.0), b: Vec3::new(q.x + nrm.x * th, q.y + nrm.y * th, 0.0) }),
        )?;
        let ang = nrm.y.atan2(nrm.x);
        let ang = if ang.abs() > std::f64::consts::FRAC_PI_2 { ang + std::f64::consts::PI } else { ang };
        text(
            s,
            &space,
            LAYER_STATIONS,
            Vec2::new(q.x + nrm.x * 1.5 * th, q.y + nrm.y * 1.5 * th),
            th * 0.8,
            station_text(start + m),
            ang,
            (HAlign::Left, VAlign::Middle),
        )?;
    }
    Ok(json!({ "stations": marks.len(), "length": length, "message": format!("{} stations along {length:.2} m", marks.len()) }))
}

// --- GEOSTL --------------------------------------------------------------------------------------

fn run_geostl(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "geostl";
    let ex = f64_or(p, "exaggeration", 1.0);
    if !(ex.is_finite() && (0.01..=100.0).contains(&ex)) {
        return Err(bad(CMD, "`exaggeration` must be between 0.01 and 100"));
    }
    let space = s.space();
    let tris = tin(s, &space)?;
    if tris.is_empty() {
        return Err(bad(CMD, "no terrain model: run GEODMT first"));
    }
    // Near the origin, so programs using single precision keep the detail.
    let (lo, zlo) = tris
        .iter()
        .flatten()
        .fold((Vec2::new(f64::INFINITY, f64::INFINITY), f64::INFINITY), |(l, z), v| (Vec2::new(l.x.min(v.x), l.y.min(v.y)), z.min(v.z)));
    let mut out = String::from("solid terrain\n");
    for t in &tris {
        let mut v = t.map(|q| Vec3::new(q.x - lo.x, q.y - lo.y, (q.z - zlo) * ex));
        if signed_area(&[v[0].xy(), v[1].xy(), v[2].xy()]) < 0.0 {
            v.swap(1, 2);
        }
        let (u, w) = (Vec3::new(v[1].x - v[0].x, v[1].y - v[0].y, v[1].z - v[0].z), Vec3::new(v[2].x - v[0].x, v[2].y - v[0].y, v[2].z - v[0].z));
        let nrm = Vec3::new(u.y * w.z - u.z * w.y, u.z * w.x - u.x * w.z, u.x * w.y - u.y * w.x);
        let l = (nrm.x * nrm.x + nrm.y * nrm.y + nrm.z * nrm.z).sqrt().max(1e-30);
        out.push_str(&format!("facet normal {:.6} {:.6} {:.6}\n outer loop\n", nrm.x / l, nrm.y / l, nrm.z / l));
        for q in v {
            out.push_str(&format!("  vertex {:.4} {:.4} {:.4}\n", q.x, q.y, q.z));
        }
        out.push_str(" endloop\nendfacet\n");
        if out.len() > 512 * 1024 * 1024 {
            return Err(bad(CMD, "the export would be larger than 512 MB"));
        }
    }
    out.push_str("endsolid terrain\n");
    let n = tris.len();
    let msg = format!("STL: {n} triangles (origin at {:.2}, {:.2}, height {zlo:.2})", lo.x, lo.y);
    match str_param(p, "path") {
        Some(path) => {
            #[cfg(not(target_arch = "wasm32"))]
            {
                std::fs::write(path, out.as_bytes()).map_err(|e| bad(CMD, format!("{path}: {e}")))?;
                Ok(json!({ "triangles": n, "message": format!("{msg}, written to {path}") }))
            }
            #[cfg(target_arch = "wasm32")]
            {
                let _ = (out, msg);
                Err(bad(CMD, format!("{path}: files can't be written on the web; leave out `path`")))
            }
        }
        None => Ok(json!({ "triangles": n, "text": out, "message": msg })),
    }
}

// --- prompts -------------------------------------------------------------------------------------

struct PickM {
    name: &'static str,
    cmd: &'static str,
    prompt: &'static str,
}

impl Interactive for PickM {
    fn name(&self) -> &'static str {
        self.name
    }
    fn prompt(&self, _s: &Session) -> Prompt {
        Prompt::new(self.prompt, Accept::POINT)
    }
    fn input(&mut self, s: &mut Session, i: Input) -> Result<Step> {
        match i {
            Input::Point(at) => {
                let r = s.execute(self.cmd, &json!({ "at": [at.x, at.y] }));
                say(s, r);
                Ok(Step::Continue)
            }
            Input::Enter | Input::Cancel => Ok(Step::Done),
            _ => Ok(Step::Continue),
        }
    }
}

struct NumberM {
    name: &'static str,
    cmd: &'static str,
    key: &'static str,
    prompt: &'static str,
    default: f64,
}

impl Interactive for NumberM {
    fn name(&self) -> &'static str {
        self.name
    }
    fn prompt(&self, _s: &Session) -> Prompt {
        Prompt::new(self.prompt, Accept::NUMBER).default(format!("{}", self.default))
    }
    fn input(&mut self, s: &mut Session, i: Input) -> Result<Step> {
        let v = match i {
            Input::Cancel => return Ok(Step::Cancel),
            Input::Enter => self.default,
            Input::Text(t) | Input::Keyword(t) => match super::geo_calc::typed_number(&t).filter(|v| *v > 0.0) {
                Some(v) => v,
                None => {
                    s.echo("Enter a positive number.");
                    return Ok(Step::Continue);
                }
            },
            _ => return Ok(Step::Continue),
        };
        let mut params = serde_json::Map::new();
        params.insert(self.key.into(), json!(v));
        let r = s.execute(self.cmd, &Value::Object(params));
        say(s, r);
        Ok(Step::Done)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A valley: z = |x − 10| / 5 + y / 20 over 0..20 × 0..20 on a 4 × 4 grid.
    fn valley(s: &mut Session) -> Vec<[Vec3; 3]> {
        let f = |x: f64, y: f64| 100.0 + (x - 10.0).abs() / 5.0 + y / 20.0;
        let v = |i: usize, j: usize| {
            let (x, y) = (i as f64 * 5.0, j as f64 * 5.0);
            Vec3::new(x, y, f(x, y))
        };
        let mut tris = Vec::new();
        for i in 0..4 {
            for j in 0..4 {
                let (a, b, c, d) = (v(i, j), v(i + 1, j), v(i + 1, j + 1), v(i, j + 1));
                tris.push([a, b, c]);
                tris.push([a, c, d]);
            }
        }
        let space = s.space();
        for t in &tris {
            add(s, &space, LAYER_TIN, EntityKind::Face3d(Face3d { corners: [t[0], t[1], t[2], t[2]], hidden_edges: 0 })).unwrap();
        }
        tris
    }

    #[test]
    fn water_runs_down_into_the_valley() {
        let mut s = Session::new();
        let tris = valley(&mut s);
        let line = fall_line(&tris, Vec2::new(2.0, 17.0));
        assert!(line.len() >= 2, "{line:?}");
        // It only goes down and ends lower than it started, nearer the valley floor x = 10.
        assert!(line.windows(2).all(|w| w[1].z <= w[0].z + 1e-9), "{line:?}");
        let end = line.last().unwrap();
        assert!(end.x > 2.0 && end.z < line[0].z, "{line:?}");
        let r = s.execute("geospadnice", &json!({ "at": [2, 17] })).unwrap();
        assert!(r["length"].as_f64().unwrap() > 0.0);
        assert!(s.execute("geospadnice", &json!({ "at": [50, 50] })).is_err());
    }

    #[test]
    fn an_edge_flips_and_flips_back() {
        let mut s = Session::new();
        valley(&mut s);
        // The diagonal of the first cell runs (0,0)–(5,5); flip it to (5,0)–(0,5).
        let r = s.execute("geohrana", &json!({ "at": [2.5, 2.5] })).unwrap();
        let hs: Vec<String> = r["handles"].as_array().unwrap().iter().map(|h| h.as_str().unwrap().to_string()).collect();
        let d = s.doc().unwrap();
        let has_new_diagonal = hs.iter().all(|h| {
            let e = d.model.get(Handle::parse_hex(h).unwrap()).unwrap();
            let EntityKind::Face3d(f) = &e.kind else { return false };
            let k: Vec<(i64, i64)> = f.corners[..3].iter().map(|v| (v.x as i64, v.y as i64)).collect();
            k.contains(&(5, 0)) && k.contains(&(0, 5))
        });
        assert!(has_new_diagonal);
        assert!(s.execute("geohrana", &json!({ "at": [2.5, 2.5] })).is_ok());
    }

    #[test]
    fn grid_stations_and_stl() {
        let mut s = Session::new();
        valley(&mut s);
        let r = s.execute("geomrizka", &json!({ "step": 10 })).unwrap();
        assert_eq!(r["points"], 9, "{r}");
        let axis = s.execute("pline", &json!({ "points": [[0, 0], [25, 0]] })).ok().and_then(|v| v["handle"].as_str().map(str::to_string));
        if let Some(axis) = axis {
            let r = s.execute("geostaniceni", &json!({ "path": axis, "step": 10, "start": 105 })).unwrap();
            // 0+105 (start), 0+110, 0+120, 0+130 (end at 25 m).
            assert_eq!(r["stations"], 4, "{r}");
        }
        let r = s.execute("geostl", &json!({})).unwrap();
        let t = r["text"].as_str().unwrap();
        assert_eq!(r["triangles"], 32);
        assert_eq!(t.matches("facet normal").count(), 32);
        assert!(t.starts_with("solid terrain") && t.trim_end().ends_with("endsolid terrain"));
    }

    #[test]
    fn hostile_input_is_refused_not_panicking() {
        let mut s = Session::new();
        for (c, p) in [
            ("geospadnice", json!({})),
            ("geospadnice", json!({ "at": [0, 0] })),
            ("geohrana", json!({ "at": [0, 0] })),
            ("geomrizka", json!({})),
            ("geostaniceni", json!({})),
            ("geostl", json!({})),
        ] {
            assert!(s.execute(c, &p).is_err(), "{c} {p}");
        }
        valley(&mut s);
        for (c, p) in [("geomrizka", json!({ "step": 1e-6 })), ("geomrizka", json!({ "step": 0 })), ("geostl", json!({ "exaggeration": -1 }))] {
            assert!(s.execute(c, &p).is_err(), "{c} {p}");
        }
        // The outer edge of the model has one triangle: picking far outside finds a shared edge
        // anyway (the nearest), never panics.
        let _ = s.execute("geohrana", &json!({ "at": [1e9, 1e9] }));
        let _ = fall_line(&[], Vec2::ZERO);
    }
}
