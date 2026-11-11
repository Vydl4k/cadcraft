//! Survey add-on, stage 2: the terrain model (GEODMT) and its contours (GEOVRSTEVNICE).
//!
//! GEODMT triangulates surveyed points (POINTs with their height as Z, by default those GEOBODY
//! put on `GEO_body`) into a constrained Delaunay TIN: selected 3D polylines and lines are break
//! lines the triangles may not cross (a kerb, a ditch, the top of an embankment). Each triangle
//! becomes a 3DFACE on `GEO_DMT`. `maxEdge` drops triangles with a longer side, which otherwise
//! bridge the hollows of the survey's outline.
//!
//! GEOVRSTEVNICE cuts the 3DFACEs of `GEO_DMT` (or the selected ones) at every `interval`
//! metres: every `main`-th contour is an index contour on `GEO_vrstevnice_hlavni` with its height
//! written along it, the others go on `GEO_vrstevnice_vedlejsi`. Each contour is one polyline
//! with its height as elevation.

use std::collections::HashMap;

use cadcraft_doc::{Common, EntityKind, Face3d, HAlign, Handle, LwPolyline, Space, Text, VAlign};
use cadcraft_geom::{PolyVertex, Vec2, Vec3};
use serde_json::{Value, json};
use spade::{ConstrainedDelaunayTriangulation, HasPosition, Point2, Triangulation};

use super::geo::LAYER_POINTS;
use super::{CommandSpec, bad, bool_or, f64_or};
use crate::{Accept, EngineError, Input, Interactive, Prompt, Result, Session, Step};

pub const LAYER_TIN: &str = "GEO_DMT";
pub const LAYER_MAIN: &str = "GEO_vrstevnice_hlavni";
pub const LAYER_MINOR: &str = "GEO_vrstevnice_vedlejsi";
pub const LAYER_LABELS: &str = "GEO_vrstevnice_popis";

/// Points triangulated at once.
const MAX_POINTS: usize = 500_000;
/// Contour levels cut at once (over the model's height range).
const MAX_LEVELS: usize = 5_000;
/// Contour segments produced at once.
const MAX_SEGMENTS: usize = 20_000_000;

pub(super) fn specs() -> Vec<CommandSpec> {
    vec![
        CommandSpec::new("geodmt", "Terrain Model (TIN)", run_geodmt)
            .menu(&["Geo", "Terrain Model"])
            .alias(&["geotin"])
            .params("{handles?: points and break lines (default: the selection, else every POINT on GEO_body), maxEdge?: longest triangle side kept in metres (default 0 = all), replace?: bool, erase the old GEO_DMT (default true)} → {triangles, points, breaklines, message}"),
        CommandSpec::new("geovrstevnice", "Contours...", run_geovrstevnice)
            .menu(&["Geo", "Contours..."])
            .alias(&["geocontours", "geovrst"])
            .params("{interval?: metres (default 0.5), main?: every n-th is an index contour (default 5), scale?: map scale 1:N for labels (default 500), labels?: bool (default true), handles?: 3DFACEs (default: the selection, else GEO_DMT), replace?: bool (default true)} → {contours, levels, message}")
            .interactive(|_| Ok(Box::new(VrstM::default()))),
    ]
}

/// A TIN vertex: its plan position and height.
#[derive(Clone, Copy, Debug)]
struct V {
    p: Point2<f64>,
    z: f64,
}

impl HasPosition for V {
    type Scalar = f64;
    fn position(&self) -> Point2<f64> {
        self.p
    }
}

fn ensure_layer(s: &mut Session, name: &str, aci: u8) -> Result<()> {
    let d = s.doc_mut()?;
    if d.ensure_layer(name)
        && let Some(l) = d.layer_mut(name)
    {
        l.color = cadcraft_color::Color::Index(aci);
    }
    Ok(())
}

/// Erase every object on `layer` in `space`.
fn clear_layer(s: &mut Session, space: &Space, layer: &str) -> Result<usize> {
    let d = s.doc_mut()?;
    let doomed: Vec<Handle> =
        d.space(space).map(|st| st.iter().filter(|e| e.common.layer.eq_ignore_ascii_case(layer)).map(|e| e.handle).collect()).unwrap_or_default();
    for h in &doomed {
        d.remove_entity(*h);
    }
    Ok(doomed.len())
}

fn handles_or_selection(s: &Session, p: &Value) -> Result<Vec<Handle>> {
    super::targets(s, p)
}

fn run_geodmt(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "geodmt";
    let max_edge = f64_or(p, "maxEdge", 0.0);
    if !max_edge.is_finite() || max_edge < 0.0 {
        return Err(bad(CMD, "`maxEdge` must be 0 or a positive length"));
    }
    let space = s.space();
    let chosen = handles_or_selection(s, p)?;
    let mut pts: Vec<Vec3> = Vec::new();
    let mut breaks: Vec<Vec<Vec3>> = Vec::new();
    {
        let d = s.doc()?;
        let st = d.space(&space).ok_or_else(|| bad(CMD, "no drawing space"))?;
        let mut take = |e: &cadcraft_doc::Entity| match &e.kind {
            EntityKind::Point(q) => pts.push(q.p),
            EntityKind::Polyline3d(pl) if pl.points.len() >= 2 => {
                let mut v = pl.points.clone();
                if pl.closed
                    && let Some(f) = v.first().copied()
                {
                    v.push(f);
                }
                breaks.push(v);
            }
            EntityKind::Line(l) => breaks.push(vec![l.a, l.b]),
            _ => {}
        };
        if chosen.is_empty() {
            for e in st.iter().filter(|e| e.common.layer.eq_ignore_ascii_case(LAYER_POINTS)) {
                take(e);
            }
        } else {
            for h in &chosen {
                if let Some(e) = st.get(*h) {
                    take(e);
                }
            }
        }
    }
    // Break line vertices are terrain points too.
    for b in &breaks {
        pts.extend(b.iter().copied());
    }
    pts.retain(|q| q.x.is_finite() && q.y.is_finite() && q.z.is_finite());
    if pts.len() < 3 {
        return Err(bad(CMD, "a terrain model needs at least 3 points with heights (select them, or import them with GEOBODY)"));
    }
    if pts.len() > MAX_POINTS {
        return Err(bad(CMD, format!("at most {MAX_POINTS} points at once")));
    }
    // Triangulate about the points' centre: survey coordinates are ~10^6 m, and spade's exact
    // predicates are happier near the origin.
    let n = pts.len() as f64;
    let c = pts.iter().fold(Vec2::ZERO, |a, q| Vec2::new(a.x + q.x / n, a.y + q.y / n));
    let mut cdt: ConstrainedDelaunayTriangulation<V> = ConstrainedDelaunayTriangulation::new();
    let mut index: HashMap<(i64, i64), spade::handles::FixedVertexHandle> = HashMap::new();
    let key = |q: Vec3| ((q.x * 1000.0).round() as i64, (q.y * 1000.0).round() as i64);
    for q in &pts {
        if index.contains_key(&key(*q)) {
            continue;
        }
        let v = V { p: Point2::new(q.x - c.x, q.y - c.y), z: q.z };
        let h = cdt.insert(v).map_err(|e| bad(CMD, format!("a point could not be triangulated: {e:?}")))?;
        index.insert(key(*q), h);
    }
    let mut constrained = 0;
    for b in &breaks {
        for w in b.windows(2) {
            let (Some(a), Some(z)) = (w.first(), w.get(1)) else { continue };
            let (Some(&ha), Some(&hz)) = (index.get(&key(*a)), index.get(&key(*z))) else { continue };
            if ha != hz && !cdt.try_add_constraint(ha, hz).is_empty() {
                constrained += 1;
            }
        }
    }
    let replace = bool_or(p, "replace", true);
    if replace {
        clear_layer(s, &space, LAYER_TIN)?;
    }
    ensure_layer(s, LAYER_TIN, 8)?;
    let common = Common { layer: LAYER_TIN.into(), ..s.current_common() };
    let mut faces = Vec::new();
    for f in cdt.inner_faces() {
        let vs = f.vertices().map(|v| *v.data());
        let corner = |v: &V| Vec3::new(v.p.x + c.x, v.p.y + c.y, v.z);
        let [a, b, d3] = [corner(&vs[0]), corner(&vs[1]), corner(&vs[2])];
        if max_edge > 0.0 {
            let len = |u: Vec3, w: Vec3| ((u.x - w.x).powi(2) + (u.y - w.y).powi(2)).sqrt();
            if len(a, b) > max_edge || len(b, d3) > max_edge || len(d3, a) > max_edge {
                continue;
            }
        }
        faces.push(Face3d { corners: [a, b, d3, d3], hidden_edges: 0 });
    }
    let d = s.doc_mut()?;
    let mut handles = Vec::with_capacity(faces.len());
    for f in faces {
        handles.push(d.add(&space, common.clone(), EntityKind::Face3d(f)).map_err(|e| bad(CMD, e.to_string()))?);
    }
    Ok(json!({
        "triangles": handles.len(),
        "points": index.len(),
        "breaklines": constrained,
        "message": format!("Terrain model: {} triangles from {} points{}", handles.len(), index.len(), if constrained > 0 { format!(", {constrained} break line segments") } else { String::new() }),
    }))
}

/// Where the level `l` crosses the edge a–b (heights on either side of it).
fn cross(a: Vec3, b: Vec3, l: f64) -> Vec2 {
    // The same edge is cut from both of its triangles: order its ends so both get the very same
    // point, and the contour joins up.
    let (a, b) = if (a.x, a.y, a.z) <= (b.x, b.y, b.z) { (a, b) } else { (b, a) };
    let t = (l - a.z) / (b.z - a.z);
    Vec2::new(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y))
}

/// The contour segments of one triangle at level `l` (heights exactly at `l` count as above).
fn tri_segment(t: [Vec3; 3], l: f64) -> Option<(Vec2, Vec2)> {
    let above = t.map(|v| v.z >= l);
    let n = above.iter().filter(|a| **a).count();
    if n == 0 || n == 3 {
        return None;
    }
    // The lone corner: the one above when one is, else the one below.
    let lone = (0..3).find(|i| above.get(*i).copied() == Some(n == 1))?;
    let a = *t.get(lone)?;
    let b = *t.get((lone + 1) % 3)?;
    let c = *t.get((lone + 2) % 3)?;
    let (p, q) = (cross(a, b, l), cross(a, c, l));
    // A corner exactly at the level touches it in a point, which is no contour.
    ((p.x - q.x).hypot(p.y - q.y) > 1e-9).then_some((p, q))
}

/// Join segments into polylines by their shared ends.
fn chain(segs: Vec<(Vec2, Vec2)>) -> Vec<(Vec<Vec2>, bool)> {
    let key = |p: Vec2| ((p.x * 1e5).round() as i64, (p.y * 1e5).round() as i64);
    let mut ends: HashMap<(i64, i64), Vec<usize>> = HashMap::new();
    for (i, (a, b)) in segs.iter().enumerate() {
        ends.entry(key(*a)).or_default().push(i);
        ends.entry(key(*b)).or_default().push(i);
    }
    let mut used = vec![false; segs.len()];
    let mut out = Vec::new();
    for start in 0..segs.len() {
        if used.get(start).copied().unwrap_or(true) {
            continue;
        }
        if let Some(u) = used.get_mut(start) {
            *u = true;
        }
        let Some(&(a, b)) = segs.get(start) else { continue };
        let mut line = std::collections::VecDeque::from([a, b]);
        // Grow forward from the back, then backward from the front.
        for forward in [true, false] {
            while let Some(&tip) = if forward { line.back() } else { line.front() } {
                let next = ends.get(&key(tip)).and_then(|c| c.iter().copied().find(|i| !used.get(*i).copied().unwrap_or(true)));
                let Some(i) = next else { break };
                if let Some(u) = used.get_mut(i) {
                    *u = true;
                }
                let Some(&(p, q)) = segs.get(i) else { break };
                let other = if key(p) == key(tip) { q } else { p };
                if forward {
                    line.push_back(other);
                } else {
                    line.push_front(other);
                }
            }
        }
        let closed = line.len() > 3 && line.front().zip(line.back()).is_some_and(|(f, l)| key(*f) == key(*l));
        let mut v: Vec<Vec2> = line.into_iter().collect();
        if closed {
            v.pop();
        }
        out.push((v, closed));
    }
    out
}

/// Where to write a contour's height: the middle of its longest segment, and the angle there
/// (kept upright).
fn label_spot(v: &[Vec2]) -> Option<(Vec2, f64)> {
    let w = v.windows(2).max_by(|a, b| {
        let l = |w: &[Vec2]| match w {
            [p, q] => (q.x - p.x).hypot(q.y - p.y),
            _ => 0.0,
        };
        l(a).total_cmp(&l(b))
    })?;
    let (p, q) = (*w.first()?, *w.get(1)?);
    let mut ang = (q.y - p.y).atan2(q.x - p.x);
    if ang > std::f64::consts::FRAC_PI_2 {
        ang -= std::f64::consts::PI;
    } else if ang < -std::f64::consts::FRAC_PI_2 {
        ang += std::f64::consts::PI;
    }
    Some((Vec2::new((p.x + q.x) / 2.0, (p.y + q.y) / 2.0), ang))
}

fn run_geovrstevnice(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "geovrstevnice";
    let interval = f64_or(p, "interval", 0.5);
    if !(interval.is_finite() && (0.001..=1000.0).contains(&interval)) {
        return Err(bad(CMD, "`interval` must be between 0.001 and 1000 m"));
    }
    let main = p.get("main").and_then(Value::as_u64).unwrap_or(5).clamp(1, 1000) as i64;
    let scale = f64_or(p, "scale", 500.0);
    if !(scale.is_finite() && (1.0..=1_000_000.0).contains(&scale)) {
        return Err(bad(CMD, "`scale` must be a map scale like 500 or 1000"));
    }
    let space = s.space();
    let chosen = handles_or_selection(s, p)?;
    let tris: Vec<[Vec3; 3]> = {
        let d = s.doc()?;
        let st = d.space(&space).ok_or_else(|| bad(CMD, "no drawing space"))?;
        let face = |e: &cadcraft_doc::Entity| match &e.kind {
            EntityKind::Face3d(f) => {
                let [a, b, c, _] = f.corners;
                Some([a, b, c])
            }
            _ => None,
        };
        let from_sel: Vec<_> = chosen.iter().filter_map(|h| st.get(*h)).filter_map(|e| face(e)).collect();
        if from_sel.is_empty() {
            st.iter().filter(|e| e.common.layer.eq_ignore_ascii_case(LAYER_TIN)).filter_map(|e| face(e)).collect()
        } else {
            from_sel
        }
    };
    let tris: Vec<_> = tris.into_iter().filter(|t| t.iter().all(|v| v.x.is_finite() && v.y.is_finite() && v.z.is_finite())).collect();
    if tris.is_empty() {
        return Err(bad(CMD, "no terrain model: run GEODMT first, or select its triangles"));
    }
    let (zmin, zmax) = tris.iter().flatten().fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), v| (lo.min(v.z), hi.max(v.z)));
    let first = (zmin / interval).ceil() as i64;
    let last = (zmax / interval).floor() as i64;
    let levels = last.saturating_sub(first).saturating_add(1);
    if levels > MAX_LEVELS as i64 {
        return Err(bad(CMD, format!("{levels} contour levels: use a larger interval")));
    }
    if bool_or(p, "replace", true) {
        for l in [LAYER_MAIN, LAYER_MINOR, LAYER_LABELS] {
            clear_layer(s, &space, l)?;
        }
    }
    ensure_layer(s, LAYER_MAIN, 34)?;
    ensure_layer(s, LAYER_MINOR, 32)?;
    {
        // Index contours are drawn heavier than the others.
        let d = s.doc_mut()?;
        for (name, lw) in [(LAYER_MAIN, 35), (LAYER_MINOR, 13)] {
            if let Some(l) = d.layer_mut(name) {
                l.lineweight = cadcraft_doc::Lineweight::from_dxf(lw);
            }
        }
    }
    ensure_layer(s, LAYER_LABELS, 34)?;
    let labels = bool_or(p, "labels", true);
    let h = 0.0015 * scale;
    let decimals = if interval.fract() == 0.0 {
        0
    } else if (interval * 10.0).fract().abs() < 1e-9 {
        1
    } else {
        2
    };
    let base = s.current_common();
    let mut contours = 0usize;
    let mut segments = 0usize;
    for k in first..=last {
        let l = k as f64 * interval;
        let segs: Vec<_> = tris.iter().filter_map(|t| tri_segment(*t, l)).collect();
        segments += segs.len();
        if segments > MAX_SEGMENTS {
            return Err(bad(CMD, "the model is too large for this interval: use a larger interval"));
        }
        let is_main = k.rem_euclid(main) == 0;
        let layer = if is_main { LAYER_MAIN } else { LAYER_MINOR };
        for (v, closed) in chain(segs) {
            if v.len() < 2 {
                continue;
            }
            let spot = if is_main && labels { label_spot(&v) } else { None };
            let pl = LwPolyline { vertices: v.into_iter().map(PolyVertex::new).collect(), closed, const_width: 0.0, elevation: l, plinegen: true };
            let common = Common { layer: layer.into(), ..base.clone() };
            s.doc_mut()?.add(&space, common, EntityKind::LwPolyline(pl)).map_err(|e| bad(CMD, e.to_string()))?;
            contours += 1;
            if let Some((at, ang)) = spot {
                let t = Text {
                    insert: Vec3::new(at.x, at.y, l),
                    align_pt: Some(Vec3::new(at.x, at.y, l)),
                    height: h,
                    value: format!("{l:.decimals$}"),
                    rotation: ang,
                    width_factor: 1.0,
                    oblique: 0.0,
                    style: "Standard".into(),
                    halign: HAlign::Center,
                    valign: VAlign::Middle,
                };
                let common = Common { layer: LAYER_LABELS.into(), ..base.clone() };
                s.doc_mut()?.add(&space, common, EntityKind::Text(t)).map_err(|e| bad(CMD, e.to_string()))?;
            }
        }
    }
    Ok(json!({
        "contours": contours,
        "levels": levels.max(0),
        "message": format!("{contours} contours every {interval} m ({} to {})", format_args!("{:.2}", first as f64 * interval), format_args!("{:.2}", last as f64 * interval)),
    }))
}

#[derive(Default)]
struct VrstM {
    interval: Option<f64>,
}

impl Interactive for VrstM {
    fn name(&self) -> &'static str {
        "GEOVRSTEVNICE"
    }

    fn prompt(&self, _s: &Session) -> Prompt {
        match self.interval {
            None => Prompt::new("Contour interval in metres", Accept::NUMBER).default("0.5"),
            Some(_) => Prompt::new("Index contour every", Accept::NUMBER).default("5"),
        }
    }

    fn input(&mut self, s: &mut Session, i: Input) -> Result<Step> {
        let t = match &i {
            Input::Cancel => return Ok(Step::Cancel),
            Input::Enter => None,
            Input::Text(t) | Input::Keyword(t) => Some(t.trim().replace(',', ".")),
            _ => return Ok(Step::Continue),
        };
        let Some(interval) = self.interval else {
            let v = t.as_deref().map_or(Some(0.5), |t| t.parse::<f64>().ok());
            match v {
                Some(v) if v.is_finite() && v > 0.0 => self.interval = Some(v),
                _ => s.echo("Enter the interval in metres, like 0.5 or 1."),
            }
            return Ok(Step::Continue);
        };
        let main = t.as_deref().map_or(Some(5), |t| t.parse::<u64>().ok()).filter(|n| *n >= 1);
        let Some(main) = main else {
            s.echo("Enter a whole number, like 5.");
            return Ok(Step::Continue);
        };
        match run_geovrstevnice(s, &json!({ "interval": interval, "main": main })) {
            Ok(r) => {
                if let Some(m) = r.get("message").and_then(Value::as_str) {
                    s.echo(m.to_string());
                }
            }
            Err(e) => s.echo(match e {
                EngineError::BadParams { msg, .. } => msg,
                other => other.to_string(),
            }),
        }
        Ok(Step::Done)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn faces(s: &Session) -> Vec<Face3d> {
        s.doc().unwrap().model.iter().filter_map(|e| if let EntityKind::Face3d(f) = &e.kind { Some(f.clone()) } else { None }).collect()
    }

    #[test]
    fn a_grid_becomes_triangles_and_contours() {
        let mut s = Session::new();
        // A plane rising 1 m per 10 m eastwards, on a 5 × 5 grid of 10 m.
        let mut text = String::new();
        for i in 0..5 {
            for j in 0..5 {
                text.push_str(&format!("{} {} {} {}\n", i * 5 + j, i * 10, j * 10, 100.0 + i as f64));
            }
        }
        s.execute("geobody", &json!({ "text": text, "sjtsk": false })).unwrap();
        let r = s.execute("geodmt", &json!({})).unwrap();
        assert_eq!(r["triangles"], 32);
        assert!(faces(&s).iter().all(|f| f.corners.iter().all(|c| (100.0..=104.0).contains(&c.z))));
        let r = s.execute("geovrstevnice", &json!({ "interval": 1, "main": 2 })).unwrap();
        // Levels 100..104, each one straight line across.
        assert_eq!(r["levels"], 5);
        let d = s.doc().unwrap();
        let contours: Vec<_> = d
            .model
            .iter()
            .filter_map(|e| if let EntityKind::LwPolyline(p) = &e.kind { Some((e.common.layer.clone(), p.clone())) } else { None })
            .collect();
        let at_102: Vec<_> = contours.iter().filter(|(_, p)| (p.elevation - 102.0).abs() < 1e-9).collect();
        assert_eq!(at_102.len(), 1, "{contours:?}");
        let (layer, pl) = at_102[0];
        assert_eq!(layer, LAYER_MAIN);
        assert!(pl.vertices.iter().all(|v| (v.p.x - 20.0).abs() < 1e-9), "{pl:?}");
        let labels = d.model.iter().filter(|e| e.common.layer == LAYER_LABELS).count();
        // 100 is the model's lowest edge (no crossing), 102 and 104 are labelled.
        assert!(labels >= 2, "{labels}");
        // Running it again replaces, not adds.
        let n = d.model.iter().count();
        s.execute("geovrstevnice", &json!({ "interval": 1, "main": 2 })).unwrap();
        assert_eq!(s.doc().unwrap().model.iter().count(), n);
    }

    #[test]
    fn long_triangles_can_be_dropped() {
        let mut s = Session::new();
        s.execute("geobody", &json!({ "text": "1 0 0 1\n2 10 0 1\n3 0 10 1\n4 100 100 2\n", "sjtsk": false })).unwrap();
        assert_eq!(s.execute("geodmt", &json!({})).unwrap()["triangles"], 2);
        assert_eq!(s.execute("geodmt", &json!({ "maxEdge": 20 })).unwrap()["triangles"], 1);
        assert_eq!(faces(&s).len(), 1);
    }

    #[test]
    fn a_corner_at_the_level_is_no_contour() {
        let t = [Vec3::new(0.0, 0.0, 5.0), Vec3::new(1.0, 0.0, 4.0), Vec3::new(0.0, 1.0, 4.0)];
        assert_eq!(tri_segment(t, 5.0), None);
        assert!(tri_segment(t, 4.5).is_some());
    }

    #[test]
    fn chains_and_closes() {
        let p = |x: f64, y: f64| Vec2::new(x, y);
        let ring = vec![(p(0.0, 0.0), p(1.0, 0.0)), (p(1.0, 1.0), p(0.0, 1.0)), (p(1.0, 0.0), p(1.0, 1.0)), (p(0.0, 1.0), p(0.0, 0.0))];
        let out = chain(ring);
        assert_eq!(out.len(), 1);
        assert!(out[0].1);
        assert_eq!(out[0].0.len(), 4);
    }

    #[test]
    fn hostile_input_is_refused_not_panicking() {
        let mut s = Session::new();
        assert!(s.execute("geodmt", &json!({})).is_err());
        assert!(s.execute("geovrstevnice", &json!({})).is_err());
        s.execute("geobody", &json!({ "text": "1 0 0 1\n2 0 0 1\n3 0 0 1\n4 1 1 1\n", "sjtsk": false })).unwrap();
        let _ = s.execute("geodmt", &json!({ "maxEdge": -1 }));
        let _ = s.execute("geodmt", &json!({}));
        for p in [json!({ "interval": 0 }), json!({ "interval": -1 }), json!({ "interval": 1e-9 }), json!({ "scale": 0 })] {
            assert!(s.execute("geovrstevnice", &p).is_err(), "{p}");
        }
    }
}
