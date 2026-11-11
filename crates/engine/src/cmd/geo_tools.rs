//! Survey add-on: building outlines from tape measurements, and batch work on surveyed points.
//!
//! - GEOOMERNE: a right-angled building outline from two surveyed corners and the measured side
//!   lengths (positive: turn left, negative: turn right). The outline's misclosure is reported
//!   and spread along it in proportion to the distance run (compass rule).
//! - GEOKONTROLA: surveyed points closer together than a tolerance, ringed on `GEO_kontrola`.
//! - GEOZMENAZ: shift the heights of the surveyed points, and their height labels with them.

use std::collections::HashMap;

use cadcraft_doc::{Circle, Common, EntityKind, Handle, LwPolyline, Space};
use cadcraft_geom::{PolyVertex, Vec2, Vec3};
use serde_json::{Value, json};

use super::geo::{LAYER_HEIGHTS, LAYER_POINTS};
use super::{CommandSpec, bad, bool_or, f64_or, point_value};
use crate::{Accept, EngineError, Input, Interactive, Prompt, Result, Session, Step};

pub const LAYER_OUTLINE: &str = "GEO_omerne";
pub const LAYER_CHECK: &str = "GEO_kontrola";

/// Sides of one outline.
const MAX_SIDES: usize = 1_000;

pub(super) fn specs() -> Vec<CommandSpec> {
    vec![
        CommandSpec::new("geoomerne", "Outline from Tape Measurements...", run_geoomerne)
            .menu(&["Geo", "Building", "Outline from Tape Measurements..."])
            .alias(&["geotape"])
            .params("{a: [x,y], b: [x,y] two surveyed corners (the first side), lengths: [metres...] the next sides in order, + turn left, - turn right, adjust?: spread the misclosure (default true)} → {misclosure, sides, area, message}")
            .interactive(|_| Ok(Box::new(OmerneM::default()))),
        CommandSpec::new("geokontrola", "Check Duplicate Points", run_geokontrola)
            .menu(&["Geo", "Points", "Check Duplicates"])
            .alias(&["geoduplicates"])
            .params("{tolerance?: metres in plan (default 0.01), mark?: ring them (default true)} → {pairs, message}"),
        CommandSpec::new("geozmenaz", "Shift Point Heights...", run_geozmenaz)
            .menu(&["Geo", "Points", "Shift Heights..."])
            .alias(&["geoshiftz"])
            .params("{delta: metres added to every height, handles?: points (default: the selected ones, else all on GEO_body), decimals?: label decimals (default 2)} → {points, labels, message}")
            .interactive(|_| Ok(Box::new(ZmenaM))),
    ]
}

fn say(s: &mut Session, r: Result<Value>) {
    match r {
        Ok(v) => {
            if let Some(m) = v.get("message").and_then(Value::as_str) {
                s.echo(m.to_string());
            }
        }
        Err(EngineError::BadParams { msg, .. }) => s.echo(msg),
        Err(e) => s.echo(e.to_string()),
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

fn add(s: &mut Session, space: &Space, layer: &str, kind: EntityKind) -> Result<Handle> {
    let common = Common { layer: layer.into(), ..s.current_common() };
    s.doc_mut()?.add(space, common, kind).map_err(|e| bad("geo", e.to_string()))
}

// --- GEOOMERNE -----------------------------------------------------------------------------------

/// The outline's corners from A, B and the side lengths, its misclosure (where it ends minus A),
/// before any adjustment.
pub fn outline(a: Vec2, b: Vec2, lengths: &[f64]) -> Option<(Vec<Vec2>, Vec2)> {
    let ab = Vec2::new(b.x - a.x, b.y - a.y);
    let l = ab.x.hypot(ab.y);
    if l < 1e-9 {
        return None;
    }
    let mut d = Vec2::new(ab.x / l, ab.y / l);
    let mut p = b;
    let mut pts = vec![a, b];
    for &s in lengths {
        // Turn a right angle: left for a positive length, right for a negative one.
        d = if s >= 0.0 { Vec2::new(-d.y, d.x) } else { Vec2::new(d.y, -d.x) };
        p = Vec2::new(p.x + d.x * s.abs(), p.y + d.y * s.abs());
        pts.push(p);
    }
    let end = *pts.last()?;
    Some((pts, Vec2::new(end.x - a.x, end.y - a.y)))
}

fn run_geoomerne(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "geoomerne";
    let a = p.get("a").and_then(point_value).ok_or_else(|| bad(CMD, "`a` ([x, y], the first surveyed corner) is required"))?;
    let b = p.get("b").and_then(point_value).ok_or_else(|| bad(CMD, "`b` ([x, y], the second surveyed corner) is required"))?;
    let lengths: Vec<f64> = p
        .get("lengths")
        .and_then(Value::as_array)
        .and_then(|v| v.iter().map(Value::as_f64).collect::<Option<Vec<_>>>())
        .ok_or_else(|| bad(CMD, "`lengths` must be a list of side lengths in metres"))?;
    if lengths.len() < 2 || lengths.len() > MAX_SIDES || lengths.iter().any(|l| !l.is_finite() || l.abs() < 1e-6 || l.abs() > 1e5) {
        return Err(bad(CMD, format!("give 2 to {MAX_SIDES} side lengths after the first side, each non-zero")));
    }
    let (mut pts, mis) = outline(a, b, &lengths).ok_or_else(|| bad(CMD, "the two corners are the same point"))?;
    let misclosure = mis.x.hypot(mis.y);
    let ab = (b.x - a.x).hypot(b.y - a.y);
    let total = ab + lengths.iter().map(|l| l.abs()).sum::<f64>();
    // The last corner should be A again: drop it, spreading the gap back along the outline.
    pts.pop();
    if bool_or(p, "adjust", true) {
        let mut run = ab;
        for (i, q) in pts.iter_mut().enumerate().skip(2) {
            run += lengths.get(i - 2).map_or(0.0, |l| l.abs());
            let k = run / total;
            *q = Vec2::new(q.x - mis.x * k, q.y - mis.y * k);
        }
    }
    let area = {
        let n = pts.len();
        ((0..n).map(|i| pts.get(i).zip(pts.get((i + 1) % n)).map_or(0.0, |(u, w)| u.x * w.y - w.x * u.y)).sum::<f64>() / 2.0).abs()
    };
    let space = s.space();
    ensure_layer(s, LAYER_OUTLINE, 2)?;
    add(
        s,
        &space,
        LAYER_OUTLINE,
        EntityKind::LwPolyline(LwPolyline {
            vertices: pts.iter().copied().map(PolyVertex::new).collect(),
            closed: true,
            const_width: 0.0,
            elevation: 0.0,
            plinegen: false,
        }),
    )?;
    Ok(json!({
        "misclosure": misclosure,
        "sides": pts.len(),
        "area": area,
        "message": format!(
            "Outline of {} sides, {area:.2} m²; misclosure {:.3} m ({:.0} mm per 100 m){}",
            pts.len(),
            misclosure,
            misclosure / total * 100_000.0,
            if bool_or(p, "adjust", true) { ", spread along the outline" } else { "" }
        ),
    }))
}

#[derive(Default)]
struct OmerneM {
    a: Option<Vec2>,
    b: Option<Vec2>,
}

impl Interactive for OmerneM {
    fn name(&self) -> &'static str {
        "GEOOMERNE"
    }
    fn prompt(&self, _s: &Session) -> Prompt {
        match (self.a, self.b) {
            (None, _) => Prompt::new("First surveyed corner", Accept::POINT),
            (Some(_), None) => Prompt::new("Second surveyed corner", Accept::POINT),
            _ => Prompt::new("Next side lengths in order (+ turn left, - turn right), e.g. 8.45 -3.2 4.1", Accept::TEXT),
        }
    }
    fn input(&mut self, s: &mut Session, i: Input) -> Result<Step> {
        match (self.a, self.b, i) {
            (_, _, Input::Cancel) => Ok(Step::Cancel),
            (None, _, Input::Point(q)) => {
                self.a = Some(q);
                Ok(Step::Continue)
            }
            (Some(_), None, Input::Point(q)) => {
                self.b = Some(q);
                Ok(Step::Continue)
            }
            (Some(a), Some(b), Input::Text(t) | Input::Keyword(t)) => {
                let v: Option<Vec<f64>> =
                    t.split([' ', ';', '\t']).filter(|w| !w.is_empty()).map(|w| w.replace(',', ".").parse::<f64>().ok()).collect();
                let Some(v) = v else {
                    s.echo("Enter the lengths as numbers separated by spaces, like 8.45 -3.2 4.1.");
                    return Ok(Step::Continue);
                };
                let r = run_geoomerne(s, &json!({ "a": [a.x, a.y], "b": [b.x, b.y], "lengths": v }));
                say(s, r);
                Ok(Step::Done)
            }
            _ => Ok(Step::Continue),
        }
    }
}

// --- GEOKONTROLA ---------------------------------------------------------------------------------

fn survey_points(s: &Session, space: &Space) -> Result<Vec<(Handle, Vec3)>> {
    let d = s.doc()?;
    let Some(st) = d.space(space) else { return Ok(Vec::new()) };
    Ok(st
        .iter()
        .filter(|e| e.common.layer.eq_ignore_ascii_case(LAYER_POINTS))
        .filter_map(|e| if let EntityKind::Point(q) = &e.kind { Some((e.handle, q.p)) } else { None })
        .filter(|(_, q)| q.x.is_finite() && q.y.is_finite() && q.z.is_finite())
        .collect())
}

fn run_geokontrola(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "geokontrola";
    let tol = f64_or(p, "tolerance", 0.01);
    if !(tol.is_finite() && (1e-6..=100.0).contains(&tol)) {
        return Err(bad(CMD, "`tolerance` must be between 0.000001 and 100 m"));
    }
    let space = s.space();
    let pts = survey_points(s, &space)?;
    if pts.is_empty() {
        return Err(bad(CMD, "no points on GEO_body"));
    }
    let key = |q: Vec3| ((q.x / tol).floor() as i64, (q.y / tol).floor() as i64);
    let mut grid: HashMap<(i64, i64), Vec<usize>> = HashMap::new();
    for (i, (_, q)) in pts.iter().enumerate() {
        grid.entry(key(*q)).or_default().push(i);
    }
    let mut pairs = Vec::new();
    for (i, (_, q)) in pts.iter().enumerate() {
        let (cx, cy) = key(*q);
        for dx in -1..=1 {
            for dy in -1..=1 {
                for &j in grid.get(&(cx + dx, cy + dy)).map(Vec::as_slice).unwrap_or(&[]) {
                    if j <= i {
                        continue;
                    }
                    let Some((_, r)) = pts.get(j) else { continue };
                    if (q.x - r.x).hypot(q.y - r.y) <= tol {
                        pairs.push((i, j, (q.z - r.z).abs()));
                    }
                }
            }
        }
        if pairs.len() > 100_000 {
            break;
        }
    }
    if bool_or(p, "mark", true) && !pairs.is_empty() {
        ensure_layer(s, LAYER_CHECK, 1)?;
        let ring = (tol * 20.0).max(0.25);
        for (i, _, _) in &pairs {
            let Some((_, q)) = pts.get(*i) else { continue };
            add(s, &space, LAYER_CHECK, EntityKind::Circle(Circle { center: Vec3::new(q.x, q.y, 0.0), radius: ring }))?;
        }
    }
    let worst = pairs.iter().map(|x| x.2).fold(0.0f64, f64::max);
    Ok(json!({
        "pairs": pairs.len(),
        "handles": pairs.iter().flat_map(|(i, j, _)| [pts.get(*i).map(|x| x.0.hex()), pts.get(*j).map(|x| x.0.hex())]).flatten().collect::<Vec<_>>(),
        "message": if pairs.is_empty() {
            format!("No two points closer than {tol} m")
        } else {
            format!("{} pair(s) of points closer than {tol} m (largest height difference {worst:.3} m), ringed on {LAYER_CHECK}", pairs.len())
        },
    }))
}

// --- GEOZMENAZ -----------------------------------------------------------------------------------

fn run_geozmenaz(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "geozmenaz";
    let delta = p
        .get("delta")
        .and_then(Value::as_f64)
        .filter(|v| v.is_finite() && v.abs() <= 10_000.0)
        .ok_or_else(|| bad(CMD, "`delta` (metres added to the heights) is required"))?;
    let decimals = p.get("decimals").and_then(Value::as_u64).unwrap_or(2).min(4) as usize;
    let space = s.space();
    let mut pts = survey_points(s, &space)?;
    let chosen = super::targets(s, p)?;
    if !chosen.is_empty() {
        let set: std::collections::HashSet<Handle> = chosen.into_iter().collect();
        let picked: Vec<_> = pts.iter().copied().filter(|(h, _)| set.contains(h)).collect();
        if !picked.is_empty() {
            pts = picked;
        }
    }
    if pts.is_empty() {
        return Err(bad(CMD, "no points on GEO_body"));
    }
    // A height label is the GEO_vysky text GEOBODY wrote below and right of its point.
    let labels: Vec<(Handle, Vec2, f64)> = {
        let d = s.doc()?;
        d.space(&space)
            .map(|st| {
                st.iter()
                    .filter(|e| e.common.layer.eq_ignore_ascii_case(LAYER_HEIGHTS))
                    .filter_map(|e| if let EntityKind::Text(t) = &e.kind { Some((e.handle, t.insert.xy(), t.height)) } else { None })
                    .collect()
            })
            .unwrap_or_default()
    };
    let cell = labels.iter().map(|l| l.2).fold(0.0f64, f64::max).max(0.01) * 4.0;
    let ckey = |q: Vec2| ((q.x / cell).floor() as i64, (q.y / cell).floor() as i64);
    let mut grid: HashMap<(i64, i64), Vec<usize>> = HashMap::new();
    for (i, l) in labels.iter().enumerate() {
        grid.entry(ckey(l.1)).or_default().push(i);
    }
    let mut relabelled = 0;
    let d = s.doc_mut()?;
    for (h, q) in &pts {
        let z = q.z + delta;
        d.modify_entity(*h, |e| {
            if let EntityKind::Point(pt) = &mut e.kind {
                pt.p.z = z;
            }
        })
        .map_err(|e| bad(CMD, e.to_string()))?;
        let (cx, cy) = ckey(q.xy());
        let mut best: Option<(f64, Handle)> = None;
        for dx in -1..=1 {
            for dy in -1..=1 {
                for &i in grid.get(&(cx + dx, cy + dy)).map(Vec::as_slice).unwrap_or(&[]) {
                    let Some((lh, at, th)) = labels.get(i) else { continue };
                    let want = Vec2::new(q.x + 0.6 * th, q.y - 1.25 * th);
                    let dist = (at.x - want.x).hypot(at.y - want.y);
                    if dist < 0.5 * th && best.is_none_or(|(bd, _)| dist < bd) {
                        best = Some((dist, *lh));
                    }
                }
            }
        }
        if let Some((_, lh)) = best {
            d.modify_entity(lh, |e| {
                if let EntityKind::Text(t) = &mut e.kind {
                    t.value = format!("{z:.decimals$}");
                }
            })
            .map_err(|e| bad(CMD, e.to_string()))?;
            relabelled += 1;
        }
    }
    Ok(
        json!({ "points": pts.len(), "labels": relabelled, "message": format!("{} point heights shifted by {delta:+.3} m ({relabelled} labels updated)", pts.len()) }),
    )
}

struct ZmenaM;

impl Interactive for ZmenaM {
    fn name(&self) -> &'static str {
        "GEOZMENAZ"
    }
    fn prompt(&self, _s: &Session) -> Prompt {
        Prompt::new("Add to every height (m, negative lowers)", Accept::NUMBER)
    }
    fn input(&mut self, s: &mut Session, i: Input) -> Result<Step> {
        match i {
            Input::Cancel | Input::Enter => Ok(Step::Cancel),
            Input::Text(t) | Input::Keyword(t) => match t.trim().replace(',', ".").parse::<f64>() {
                Ok(v) if v.is_finite() => {
                    let r = run_geozmenaz(s, &json!({ "delta": v }));
                    say(s, r);
                    Ok(Step::Done)
                }
                _ => {
                    s.echo("Enter a number of metres, like 0.25 or -1.10.");
                    Ok(Step::Continue)
                }
            },
            _ => Ok(Step::Continue),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_tape_measured_l_shaped_house() {
        // An L-shaped house measured 10 × 8 with a 4 × 3 notch, rotated 30°, with 4 cm of tape
        // error in one side.
        let (sn, cs) = (30f64.to_radians().sin(), 30f64.to_radians().cos());
        let a = Vec2::new(100.0, 200.0);
        let b = Vec2::new(100.0 + 10.0 * cs, 200.0 + 10.0 * sn);
        // East 10 (a→b), then north 5, west 4, north 3 (taped 3.04), west 6, south 8.
        let lengths = [5.0, 4.0, -3.04, 6.0, 8.0];
        let (_, mis) = outline(a, b, &lengths).unwrap();
        assert!((mis.x.hypot(mis.y) - 0.04).abs() < 1e-9, "{mis:?}");
        let mut s = Session::new();
        let r = s.execute("geoomerne", &json!({ "a": [a.x, a.y], "b": [b.x, b.y], "lengths": lengths })).unwrap();
        assert_eq!(r["sides"], 6);
        assert!((r["misclosure"].as_f64().unwrap() - 0.04).abs() < 1e-9);
        // 10 × 8 minus 4 × 3, give or take the spread misclosure.
        assert!((r["area"].as_f64().unwrap() - 68.0).abs() < 0.3, "{r}");
        let d = s.doc().unwrap();
        let pl = d.model.iter().find_map(|e| if let EntityKind::LwPolyline(p) = &e.kind { Some(p.clone()) } else { None }).unwrap();
        assert!(pl.closed);
        assert_eq!(pl.vertices[0].p, a);
        assert_eq!(pl.vertices[1].p, b);
    }

    #[test]
    fn duplicates_and_height_shifts() {
        let mut s = Session::new();
        s.execute("geobody", &json!({ "text": "1 0 0 10\n2 0.004 0.003 10.2\n3 5 5 11\n", "sjtsk": false })).unwrap();
        let r = s.execute("geokontrola", &json!({})).unwrap();
        assert_eq!(r["pairs"], 1, "{r}");
        s.set_selection(Vec::new());
        let r = s.execute("geozmenaz", &json!({ "delta": -0.5 })).unwrap();
        assert_eq!(r["points"], 3);
        assert_eq!(r["labels"], 3, "{r}");
        let d = s.doc().unwrap();
        let zs: Vec<f64> = d.model.iter().filter_map(|e| if let EntityKind::Point(q) = &e.kind { Some(q.p.z) } else { None }).collect();
        assert_eq!(zs, vec![9.5, 9.7, 10.5]);
        let hs: Vec<String> = d
            .model
            .iter()
            .filter(|e| e.common.layer == LAYER_HEIGHTS)
            .filter_map(|e| if let EntityKind::Text(t) = &e.kind { Some(t.value.clone()) } else { None })
            .collect();
        assert_eq!(hs, vec!["9.50", "9.70", "10.50"]);
    }

    #[test]
    fn hostile_input_is_refused_not_panicking() {
        let mut s = Session::new();
        for (cmd, p) in [
            ("geoomerne", json!({})),
            ("geoomerne", json!({ "a": [0, 0], "b": [0, 0], "lengths": [1, 1] })),
            ("geoomerne", json!({ "a": [0, 0], "b": [1, 0], "lengths": [1] })),
            ("geoomerne", json!({ "a": [0, 0], "b": [1, 0], "lengths": [1, 0] })),
            ("geoomerne", json!({ "a": [0, 0], "b": [1, 0], "lengths": [1, "x"] })),
            ("geokontrola", json!({})),
            ("geokontrola", json!({ "tolerance": 0 })),
            ("geozmenaz", json!({})),
            ("geozmenaz", json!({ "delta": 1 })),
        ] {
            assert!(s.execute(cmd, &p).is_err(), "{cmd} {p}");
        }
    }
}
