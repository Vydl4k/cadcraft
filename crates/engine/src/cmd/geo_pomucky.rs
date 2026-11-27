//! Survey add-on utilities.
//!
//! - GEOMERITKO changes the drawing's map scale: survey labels, map symbols and point blocks are
//!   resized (labels keep their place beside their point), and the settings take the new scale.
//! - GEOOTOCIT turns the whole situation (by an angle, or so a picked line runs left to right);
//!   point labels stay upright beside their points and other texts stay readable.
//! - GEOSTATISTIKA counts what the drawing holds: objects by layer, lengths and areas by layer,
//!   survey points by code.
//! - GEOOBLOUKY turns arcs (and the arc segments of polylines) into straight segments within a
//!   tolerance.
//! - GEOSPOJ3D joins 3D polylines and lines that meet end to end into single 3D polylines.
//! - GEOTEXTROT turns texts to the direction of a line or an angle.

use std::collections::BTreeMap;

use cadcraft_doc::{EntityKind, Handle, LwPolyline, Polyline3d};
use cadcraft_geom::{PolyVertex, Vec2, Vec3, bulge_to_arc};
use serde_json::{Value, json};

use super::geo_body::survey_points;
use super::geo_calc::{add, path_of, say, signed_area, typed_number};
use super::geo_nastaveni as nast;
use super::geo_pohled::POINT_BLOCK;
use super::{CommandSpec, bad, point_value, str_param, targets};
use crate::{Accept, Input, Interactive, Prompt, Result, Session, Step};

/// Segments one arc becomes, at most.
const MAX_ARC_SEGMENTS: usize = 1000;
/// Objects handled at once.
const MAX_OBJECTS: usize = 1_000_000;

pub(super) fn specs() -> Vec<CommandSpec> {
    vec![
        CommandSpec::new("geomeritko", "Change Map Scale...", run_meritko)
            .menu(&["Geo", "Tools", "Change Map Scale..."])
            .alias(&["geochangescale"])
            .params("{scale: new map scale 1:N, from?: the old one (default the drawing's setting)} → {texts, symbols, message}")
            .interactive(|_| Ok(Box::new(NumM { name: "GEOMERITKO", msg: "New map scale 1:", key: "scale" }))),
        CommandSpec::new("geootocit", "Rotate Situation...", run_otocit)
            .menu(&["Geo", "Tools", "Rotate Situation..."])
            .alias(&["georotate"])
            .params("{angle?: degrees counter-clockwise, line?: handle of a line to run left to right, base?: [x, y] (default the drawing's centre), upright?: bool keep labels upright (default true)} → {objects, angle, message}")
            .interactive(|_| Ok(Box::new(OtocM))),
        CommandSpec::new("geostatistika", "Drawing Statistics", run_statistika)
            .menu(&["Geo", "Tools", "Drawing Statistics"])
            .alias(&["geostats"])
            .params("{} → {layers: {name: {objects, length, area}}, points, codes: {code: count}, message}"),
        CommandSpec::new("geoobluky", "Arcs to Straight Segments...", run_oblouky)
            .menu(&["Geo", "Tools", "Arcs to Straight Segments..."])
            .alias(&["geoarcs"])
            .params("{handles?: (default the selection), tolerance?: largest gap between arc and segment in m (default 0.01)} → {changed, message}")
            .interactive(|_| Ok(Box::new(SelM { name: "GEOOBLOUKY", msg: "Select arcs and polylines", cmd: "geoobluky" }))),
        CommandSpec::new("geospoj3d", "Join 3D Polylines...", run_spoj3d)
            .menu(&["Geo", "Tools", "Join 3D Polylines..."])
            .alias(&["geojoin3d"])
            .params("{handles?: 3D polylines and lines (default the selection), tolerance?: m (default 0.001)} → {joined, made, message}")
            .interactive(|_| Ok(Box::new(SelM { name: "GEOSPOJ3D", msg: "Select 3D polylines and lines to join", cmd: "geospoj3d" }))),
        CommandSpec::new("geotextrot", "Rotate Texts...", run_textrot)
            .menu(&["Geo", "Tools", "Rotate Texts..."])
            .alias(&["geotextrotate"])
            .params("{handles?: texts (default the selection), angle?: degrees (default 0), line?: handle of a line to follow} → {changed, message}")
            .interactive(|_| Ok(Box::new(TextRotM::default()))),
    ]
}

fn scaled(base: Vec2, p: Vec3, k: f64) -> Vec3 {
    Vec3::new(base.x + (p.x - base.x) * k, base.y + (p.y - base.y) * k, p.z)
}

fn run_meritko(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "geomeritko";
    let get = |key: &str| p.get(key).and_then(|v| v.as_f64().or_else(|| v.as_str().and_then(|t| typed_number(t.rsplit(':').next().unwrap_or(t)))));
    let mut n = nast::load(s);
    let new = get("scale").filter(|v| (1.0..=1_000_000.0).contains(v)).ok_or_else(|| bad(CMD, "`scale` must be a map scale like 500 or 1000"))?;
    let old = match p.get("from") {
        None | Some(Value::Null) => n.scale,
        Some(_) => get("from").filter(|v| (1.0..=1_000_000.0).contains(v)).ok_or_else(|| bad(CMD, "`from` must be a map scale"))?,
    };
    let k = new / old;
    let space = s.space();
    // Each point label moves with its point's offset scaled.
    let mut owner: BTreeMap<Handle, Vec2> = BTreeMap::new();
    for q in survey_points(s, &space)? {
        for l in [&q.number, &q.height, &q.code, &q.z2].into_iter().flatten() {
            owner.insert(l.1, q.p.xy());
        }
    }
    let todo: Vec<Handle> = {
        let d = s.doc()?;
        let st = d.space(&space).ok_or_else(|| bad(CMD, "no drawing space"))?;
        st.iter()
            .filter(|e| match &e.kind {
                EntityKind::Text(_) | EntityKind::MText(_) => e.common.layer.to_ascii_uppercase().starts_with("GEO_"),
                EntityKind::Insert(i) => i.block.starts_with("GEO_Z_") || i.block.eq_ignore_ascii_case(POINT_BLOCK),
                _ => false,
            })
            .map(|e| e.handle)
            .take(MAX_OBJECTS)
            .collect()
    };
    let (mut texts, mut symbols) = (0, 0);
    let d = s.doc_mut()?;
    for h in todo {
        let base = owner.get(&h).copied();
        let _ = d.modify_entity(h, |e| match &mut e.kind {
            EntityKind::Text(t) => {
                let b = base.unwrap_or_else(|| t.align_pt.unwrap_or(t.insert).xy());
                t.insert = scaled(b, t.insert, k);
                t.align_pt = t.align_pt.map(|a| scaled(b, a, k));
                t.height *= k;
                texts += 1;
            }
            EntityKind::MText(m) => {
                m.height *= k;
                m.width *= k;
                texts += 1;
            }
            EntityKind::Insert(i) if i.block.eq_ignore_ascii_case(POINT_BLOCK) => {
                let b = i.insert.xy();
                for a in &mut i.attribs {
                    a.text.insert = scaled(b, a.text.insert, k);
                    a.text.align_pt = a.text.align_pt.map(|x| scaled(b, x, k));
                    a.text.height *= k;
                }
                texts += 1;
            }
            EntityKind::Insert(i) => {
                i.scale = Vec3::new(i.scale.x * k, i.scale.y * k, i.scale.z * k);
                symbols += 1;
            }
            _ => {}
        });
    }
    n.scale = new;
    nast::save(s, &n)?;
    Ok(json!({ "texts": texts, "symbols": symbols, "message": format!("Map scale 1:{old} → 1:{new}: {texts} labels and {symbols} symbols resized") }))
}

/// The direction of the first segment of a line or polyline, radians.
fn line_angle(s: &Session, h: Handle, cmd: &str) -> Result<f64> {
    let space = s.space();
    let (pts, _) = path_of(s, h, &space).ok_or_else(|| bad(cmd, "pick a line or a polyline"))?;
    let (a, b) = pts.first().zip(pts.get(1)).ok_or_else(|| bad(cmd, "the line has no length"))?;
    if (b.x - a.x).hypot(b.y - a.y) < 1e-12 {
        return Err(bad(cmd, "the line has no length"));
    }
    Ok((b.y - a.y).atan2(b.x - a.x))
}

fn angle_param(s: &Session, p: &Value, cmd: &str) -> Result<Option<f64>> {
    if let Some(l) = str_param(p, "line") {
        let h = Handle::parse_hex(l).ok_or_else(|| bad(cmd, "`line` must be an object handle"))?;
        return Ok(Some(line_angle(s, h, cmd)?));
    }
    match p.get("angle") {
        None | Some(Value::Null) => Ok(None),
        Some(v) => v
            .as_f64()
            .or_else(|| v.as_str().and_then(typed_number))
            .filter(|a| a.is_finite() && a.abs() <= 1e6)
            .map(|a| Some(a.to_radians()))
            .ok_or_else(|| bad(cmd, "`angle` must be degrees")),
    }
}

/// An angle turned so text reads left to right (−90° < a ≤ 90°).
fn readable(a: f64) -> f64 {
    let mut a = a.rem_euclid(std::f64::consts::TAU);
    if a > std::f64::consts::FRAC_PI_2 && a <= 3.0 * std::f64::consts::FRAC_PI_2 {
        a -= std::f64::consts::PI;
    }
    if a > std::f64::consts::PI {
        a -= std::f64::consts::TAU;
    }
    a
}

fn run_otocit(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "geootocit";
    let angle = match (str_param(p, "line"), angle_param(s, p, CMD)?) {
        // The line comes to run left to right.
        (Some(_), Some(a)) => -a,
        (None, Some(a)) => a,
        _ => return Err(bad(CMD, "`angle` or `line` is required")),
    };
    let upright = p.get("upright").and_then(Value::as_bool).unwrap_or(true);
    let space = s.space();
    let all: Vec<Handle> = s.doc()?.space(&space).map(|st| st.iter().map(|e| e.handle).take(MAX_OBJECTS).collect()).unwrap_or_default();
    if all.is_empty() {
        return Err(bad(CMD, "the drawing is empty"));
    }
    let base = match p.get("base") {
        Some(v) => point_value(v).ok_or_else(|| bad(CMD, "`base` must be [x, y]"))?,
        None => {
            let d = s.doc()?;
            let st = d.space(&space).ok_or_else(|| bad(CMD, "no drawing space"))?;
            let mut lo = Vec2::new(f64::INFINITY, f64::INFINITY);
            let mut hi = Vec2::new(f64::NEG_INFINITY, f64::NEG_INFINITY);
            for e in st.iter().take(MAX_OBJECTS) {
                if let EntityKind::Point(q) = &e.kind {
                    lo = Vec2::new(lo.x.min(q.p.x), lo.y.min(q.p.y));
                    hi = Vec2::new(hi.x.max(q.p.x), hi.y.max(q.p.y));
                }
            }
            if lo.x.is_finite() { Vec2::new((lo.x + hi.x) / 2.0, (lo.y + hi.y) / 2.0) } else { Vec2::ZERO }
        }
    };
    // Point labels: their offset from the point and their angle, kept.
    let mut labels: BTreeMap<Handle, (Handle, Vec2, Option<Vec2>, f64)> = BTreeMap::new();
    if upright {
        let pts = survey_points(s, &space)?;
        let d = s.doc()?;
        let st = d.space(&space).ok_or_else(|| bad(CMD, "no drawing space"))?;
        for q in &pts {
            for l in [&q.number, &q.height, &q.code, &q.z2].into_iter().flatten() {
                if let Some(e) = st.get(l.1)
                    && let EntityKind::Text(t) = &e.kind
                {
                    let off = t.insert.xy() - q.p.xy();
                    let aoff = t.align_pt.map(|a| a.xy() - q.p.xy());
                    labels.insert(l.1, (q.handle, off, aoff, t.rotation));
                }
            }
        }
    }
    let hs: Vec<String> = all.iter().map(|h| h.hex()).collect();
    s.execute("rotate", &json!({ "handles": hs, "base": [base.x, base.y], "angle": angle.to_degrees() }))?;
    if upright {
        let d = s.doc_mut()?;
        let pos: BTreeMap<Handle, Vec2> = d
            .space(&space)
            .map(|st| {
                labels
                    .values()
                    .filter_map(|(ph, ..)| st.get(*ph).and_then(|e| if let EntityKind::Point(q) = &e.kind { Some((*ph, q.p.xy())) } else { None }))
                    .collect()
            })
            .unwrap_or_default();
        for h in &all {
            let lab = labels.get(h).copied();
            let _ = d.modify_entity(*h, |e| match &mut e.kind {
                EntityKind::Text(t) => match lab.and_then(|(ph, off, aoff, rot)| pos.get(&ph).map(|q| (*q, off, aoff, rot))) {
                    Some((q, off, aoff, rot)) => {
                        t.insert = Vec3::new(q.x + off.x, q.y + off.y, t.insert.z);
                        t.align_pt = aoff.map(|a| Vec3::new(q.x + a.x, q.y + a.y, t.insert.z));
                        t.rotation = rot;
                    }
                    None => t.rotation = readable(t.rotation),
                },
                EntityKind::MText(m) => m.rotation = readable(m.rotation),
                _ => {}
            });
        }
    }
    Ok(json!({ "objects": all.len(), "angle": angle.to_degrees(), "message": format!("{} objects turned by {:.4}°", all.len(), angle.to_degrees()) }))
}

fn run_statistika(s: &mut Session, _p: &Value) -> Result<Value> {
    let space = s.space();
    let pts = survey_points(s, &space)?;
    let mut codes: BTreeMap<String, usize> = BTreeMap::new();
    for q in &pts {
        *codes.entry(q.code.as_ref().map(|c| c.0.clone()).unwrap_or_else(|| "(no code)".into())).or_default() += 1;
    }
    let mut layers: BTreeMap<String, (usize, f64, f64)> = BTreeMap::new();
    let d = s.doc()?;
    let st = d.space(&space).ok_or_else(|| bad("geostatistika", "no drawing space"))?;
    let len2 = |v: &[Vec2], closed: bool| -> f64 {
        let mut l: f64 = v.windows(2).map(|w| w.first().zip(w.get(1)).map_or(0.0, |(a, b)| (b.x - a.x).hypot(b.y - a.y))).sum();
        if closed && let (Some(a), Some(b)) = (v.first(), v.last()) {
            l += (b.x - a.x).hypot(b.y - a.y);
        }
        l
    };
    for e in st.iter().take(MAX_OBJECTS) {
        let row = layers.entry(e.common.layer.clone()).or_default();
        row.0 += 1;
        match &e.kind {
            EntityKind::Line(l) => row.1 += (l.b.x - l.a.x).hypot(l.b.y - l.a.y),
            EntityKind::LwPolyline(pl) => {
                let v: Vec<Vec2> = pl.vertices.iter().map(|v| v.p).collect();
                row.1 += len2(&v, pl.closed);
                if pl.closed {
                    row.2 += signed_area(&v).abs();
                }
            }
            EntityKind::Polyline3d(pl) => {
                let v: Vec<Vec2> = pl.points.iter().map(|v| v.xy()).collect();
                row.1 += len2(&v, pl.closed);
                if pl.closed {
                    row.2 += signed_area(&v).abs();
                }
            }
            EntityKind::Circle(c) => {
                row.1 += std::f64::consts::TAU * c.radius;
                row.2 += std::f64::consts::PI * c.radius * c.radius;
            }
            EntityKind::Arc(a) => row.1 += a.radius * (a.end - a.start).rem_euclid(std::f64::consts::TAU),
            _ => {}
        }
    }
    let mut lines = vec![format!("{} survey points", pts.len())];
    for (c, k) in &codes {
        lines.push(format!("  code {c}: {k}"));
    }
    for (l, (k, len, area)) in &layers {
        let mut t = format!("layer {l}: {k} objects");
        if *len > 0.0 {
            t.push_str(&format!(", length {len:.2} m"));
        }
        if *area > 0.0 {
            t.push_str(&format!(", area {area:.2} m²"));
        }
        lines.push(t);
    }
    let lj: serde_json::Map<String, Value> =
        layers.iter().map(|(l, (k, len, area))| (l.clone(), json!({ "objects": k, "length": len, "area": area }))).collect();
    Ok(json!({ "layers": lj, "points": pts.len(), "codes": codes, "message": lines.join("\n") }))
}

/// The points of a bulged segment from `a` (left out) to `b`, within `tol` of the arc.
fn arc_points(a: Vec2, b: Vec2, bulge: f64, tol: f64) -> Vec<Vec2> {
    let Some((arc, ccw)) = bulge_to_arc(a, b, bulge) else { return vec![b] };
    let sweep = (arc.end - arc.start).rem_euclid(std::f64::consts::TAU);
    let n = segments(arc.radius, sweep, tol);
    let (from, dir) = if ccw { (arc.start, 1.0) } else { (arc.end, -1.0) };
    let mut out: Vec<Vec2> = (1..n)
        .map(|i| {
            let t = from + dir * sweep * i as f64 / n as f64;
            Vec2::new(arc.center.x + arc.radius * t.cos(), arc.center.y + arc.radius * t.sin())
        })
        .collect();
    out.push(b);
    out
}

fn segments(r: f64, sweep: f64, tol: f64) -> usize {
    // A chord of angle φ leaves r(1 − cos φ/2) between it and the arc.
    let phi = if tol >= r { std::f64::consts::PI } else { 2.0 * (1.0 - tol / r).clamp(-1.0, 1.0).acos() };
    let n = if phi > 1e-9 { (sweep / phi).ceil() } else { MAX_ARC_SEGMENTS as f64 };
    (n as usize).clamp(1, MAX_ARC_SEGMENTS)
}

fn run_oblouky(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "geoobluky";
    let tol = p.get("tolerance").and_then(Value::as_f64).unwrap_or(0.01);
    if !(tol.is_finite() && (1e-6..=1000.0).contains(&tol)) {
        return Err(bad(CMD, "`tolerance` must be 0.000001 to 1000 m"));
    }
    let hs = targets(s, p)?;
    if hs.is_empty() {
        return Err(bad(CMD, "select arcs and polylines"));
    }
    let space = s.space();
    let mut changed = 0;
    let mut arcs: Vec<(Handle, LwPolyline)> = Vec::new();
    {
        let d = s.doc()?;
        let st = d.space(&space).ok_or_else(|| bad(CMD, "no drawing space"))?;
        for h in &hs {
            if let Some(e) = st.get(*h)
                && let EntityKind::Arc(a) = &e.kind
                && a.radius.is_finite()
                && a.radius > 0.0
            {
                let sweep = (a.end - a.start).rem_euclid(std::f64::consts::TAU);
                let n = segments(a.radius, sweep, tol);
                let vertices = (0..=n)
                    .map(|i| {
                        let t = a.start + sweep * i as f64 / n as f64;
                        PolyVertex::new(Vec2::new(a.center.x + a.radius * t.cos(), a.center.y + a.radius * t.sin()))
                    })
                    .collect();
                arcs.push((*h, LwPolyline { vertices, closed: false, const_width: 0.0, elevation: a.center.z, plinegen: false }));
            }
        }
    }
    let d = s.doc_mut()?;
    for h in &hs {
        let _ = d.modify_entity(*h, |e| {
            if let EntityKind::LwPolyline(pl) = &mut e.kind
                && pl.vertices.iter().any(|v| v.bulge.abs() > 1e-12)
            {
                let n = pl.vertices.len();
                let mut out: Vec<PolyVertex> = Vec::new();
                for i in 0..n {
                    let Some(v) = pl.vertices.get(i) else { continue };
                    out.push(PolyVertex::new(v.p));
                    let next = if i + 1 < n {
                        pl.vertices.get(i + 1)
                    } else if pl.closed {
                        pl.vertices.first()
                    } else {
                        None
                    };
                    if let Some(w) = next
                        && v.bulge.abs() > 1e-12
                    {
                        let mut mid = arc_points(v.p, w.p, v.bulge, tol);
                        mid.pop();
                        out.extend(mid.into_iter().map(PolyVertex::new));
                    }
                }
                pl.vertices = out;
                changed += 1;
            }
        });
    }
    for (h, pl) in arcs {
        let d = s.doc_mut()?;
        let layer = d.space(&space).and_then(|st| st.get(h)).map(|e| e.common.layer.clone()).unwrap_or_else(|| "0".into());
        d.remove_entity(h);
        add(s, &space, &layer, EntityKind::LwPolyline(pl))?;
        changed += 1;
    }
    Ok(json!({ "changed": changed, "message": format!("{changed} arcs and polylines now straight segments (within {tol} m)") }))
}

fn run_spoj3d(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "geospoj3d";
    let tol = p.get("tolerance").and_then(Value::as_f64).unwrap_or(0.001);
    if !(tol.is_finite() && (0.0..=100.0).contains(&tol)) {
        return Err(bad(CMD, "`tolerance` must be 0 to 100 m"));
    }
    let hs = targets(s, p)?;
    let space = s.space();
    let mut parts: Vec<(Handle, String, Vec<Vec3>)> = Vec::new();
    {
        let d = s.doc()?;
        let st = d.space(&space).ok_or_else(|| bad(CMD, "no drawing space"))?;
        for h in &hs {
            let Some(e) = st.get(*h) else { continue };
            let pts = match &e.kind {
                EntityKind::Polyline3d(pl) if !pl.closed && pl.points.len() >= 2 => pl.points.clone(),
                EntityKind::Line(l) => vec![l.a, l.b],
                _ => continue,
            };
            if pts.iter().all(|q| q.x.is_finite() && q.y.is_finite() && q.z.is_finite()) {
                parts.push((*h, e.common.layer.clone(), pts));
            }
        }
    }
    if parts.len() < 2 {
        return Err(bad(CMD, "select at least two 3D polylines or lines"));
    }
    let near = |a: &Vec3, b: &Vec3| (a.x - b.x).hypot(a.y - b.y) <= tol.max(1e-9) && (a.z - b.z).abs() <= tol.max(1e-9);
    let mut used = vec![false; parts.len()];
    let mut chains: Vec<(String, Vec<Vec3>, Vec<Handle>)> = Vec::new();
    for i in 0..parts.len() {
        if used.get(i).copied().unwrap_or(true) {
            continue;
        }
        let Some((h, layer, pts)) = parts.get(i).cloned() else { continue };
        if let Some(u) = used.get_mut(i) {
            *u = true;
        }
        let (mut chain, mut members) = (pts, vec![h]);
        // Grow at either end while some unused part meets it.
        loop {
            let mut grew = false;
            for j in 0..parts.len() {
                if used.get(j).copied().unwrap_or(true) {
                    continue;
                }
                let Some((hj, _, q)) = parts.get(j) else { continue };
                let (Some(cs), Some(ce), Some(qs), Some(qe)) = (chain.first().copied(), chain.last().copied(), q.first(), q.last()) else { continue };
                let mut q = q.clone();
                if near(&ce, qs) {
                    q.remove(0);
                    chain.extend(q);
                } else if near(&ce, qe) {
                    q.reverse();
                    q.remove(0);
                    chain.extend(q);
                } else if near(&cs, qe) {
                    q.pop();
                    q.extend(chain);
                    chain = q;
                } else if near(&cs, qs) {
                    q.reverse();
                    q.pop();
                    q.extend(chain);
                    chain = q;
                } else {
                    continue;
                }
                members.push(*hj);
                if let Some(u) = used.get_mut(j) {
                    *u = true;
                }
                grew = true;
            }
            if !grew {
                break;
            }
        }
        if members.len() > 1 {
            chains.push((layer, chain, members));
        }
    }
    let joined: usize = chains.iter().map(|c| c.2.len()).sum();
    let made = chains.len();
    for (layer, pts, members) in chains {
        let closed = pts.len() > 3 && pts.first().zip(pts.last()).is_some_and(|(a, b)| near(a, b));
        let mut pts = pts;
        if closed {
            pts.pop();
        }
        {
            let d = s.doc_mut()?;
            for h in members {
                d.remove_entity(h);
            }
        }
        add(s, &space, &layer, EntityKind::Polyline3d(Polyline3d { points: pts, closed }))?;
    }
    Ok(json!({ "joined": joined, "made": made, "message": format!("{joined} pieces joined into {made} 3D polylines") }))
}

fn run_textrot(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "geotextrot";
    let a = readable(angle_param(s, p, CMD)?.unwrap_or(0.0));
    let hs = targets(s, p)?;
    let d = s.doc_mut()?;
    let mut changed = 0;
    for h in hs {
        let _ = d.modify_entity(h, |e| match &mut e.kind {
            EntityKind::Text(t) => {
                t.rotation = a;
                changed += 1;
            }
            EntityKind::MText(m) => {
                m.rotation = a;
                changed += 1;
            }
            _ => {}
        });
    }
    if changed == 0 {
        return Err(bad(CMD, "select texts"));
    }
    Ok(json!({ "changed": changed, "message": format!("{changed} texts turned to {:.2}°", a.to_degrees()) }))
}

/// A command that asks for one number.
struct NumM {
    name: &'static str,
    msg: &'static str,
    key: &'static str,
}

impl Interactive for NumM {
    fn name(&self) -> &'static str {
        self.name
    }
    fn prompt(&self, s: &Session) -> Prompt {
        Prompt::new(self.msg, Accept::NUMBER).default(format!("{}", nast::scale(s)))
    }
    fn input(&mut self, s: &mut Session, i: Input) -> Result<Step> {
        match i {
            Input::Cancel | Input::Enter => Ok(Step::Cancel),
            Input::Text(t) | Input::Keyword(t) => {
                let r = s.execute(&self.name.to_ascii_lowercase(), &json!({ self.key: t.trim() }));
                say(s, r);
                Ok(Step::Done)
            }
            _ => Ok(Step::Continue),
        }
    }
}

/// A command that works on selected objects.
struct SelM {
    name: &'static str,
    msg: &'static str,
    cmd: &'static str,
}

impl Interactive for SelM {
    fn name(&self) -> &'static str {
        self.name
    }
    fn prompt(&self, _s: &Session) -> Prompt {
        Prompt::new(self.msg, Accept::SELECT)
    }
    fn input(&mut self, s: &mut Session, i: Input) -> Result<Step> {
        match i {
            Input::Cancel => Ok(Step::Cancel),
            Input::Pick(hs) => {
                let hs: Vec<String> = hs.iter().map(|h| h.hex()).collect();
                let r = s.execute(self.cmd, &json!({ "handles": hs }));
                say(s, r);
                Ok(Step::Done)
            }
            _ => Ok(Step::Continue),
        }
    }
}

/// GEOOTOCIT: a line to run left to right, or an angle.
#[derive(Default)]
struct OtocM;

impl Interactive for OtocM {
    fn name(&self) -> &'static str {
        "GEOOTOCIT"
    }
    fn prompt(&self, _s: &Session) -> Prompt {
        Prompt::new("Select the line to run left to right, or type the angle", Accept { number: true, text: true, ..Accept::SELECT })
    }
    fn input(&mut self, s: &mut Session, i: Input) -> Result<Step> {
        let r = match i {
            Input::Cancel | Input::Enter => return Ok(Step::Cancel),
            Input::Pick(hs) => match hs.first() {
                Some(h) => run_otocit(s, &json!({ "line": h.hex() })),
                None => return Ok(Step::Continue),
            },
            Input::Text(t) | Input::Keyword(t) => run_otocit(s, &json!({ "angle": t.trim() })),
            _ => return Ok(Step::Continue),
        };
        say(s, r);
        Ok(Step::Done)
    }
}

/// GEOTEXTROT: texts, then a line or an angle.
#[derive(Default)]
struct TextRotM {
    texts: Option<Vec<Handle>>,
}

impl Interactive for TextRotM {
    fn name(&self) -> &'static str {
        "GEOTEXTROT"
    }
    fn prompt(&self, _s: &Session) -> Prompt {
        match self.texts {
            None => Prompt::new("Select texts", Accept::SELECT),
            Some(_) => {
                Prompt::new("Select the line to follow, or type the angle", Accept { number: true, text: true, ..Accept::SELECT }).default("0")
            }
        }
    }
    fn input(&mut self, s: &mut Session, i: Input) -> Result<Step> {
        let Some(texts) = self.texts.clone() else {
            return match i {
                Input::Cancel => Ok(Step::Cancel),
                Input::Pick(hs) => {
                    self.texts = Some(hs);
                    Ok(Step::Continue)
                }
                _ => Ok(Step::Continue),
            };
        };
        let hs: Vec<String> = texts.iter().map(|h| h.hex()).collect();
        let r = match i {
            Input::Cancel => return Ok(Step::Cancel),
            Input::Pick(l) => match l.first() {
                Some(h) => run_textrot(s, &json!({ "handles": hs, "line": h.hex() })),
                None => return Ok(Step::Continue),
            },
            Input::Text(t) | Input::Keyword(t) => run_textrot(s, &json!({ "handles": hs, "angle": t.trim() })),
            Input::Enter => run_textrot(s, &json!({ "handles": hs, "angle": 0 })),
            _ => return Ok(Step::Continue),
        };
        say(s, r);
        Ok(Step::Done)
    }
}

#[cfg(test)]
mod tests {
    use super::super::geo::LAYER_NUMBERS;
    use super::*;

    fn points() -> Session {
        let mut s = Session::new();
        s.execute("geobody", &json!({ "text": "1 0 0 100\n2 10 0 110\n", "sjtsk": false })).unwrap();
        s
    }

    fn number_label(s: &Session, v: &str) -> cadcraft_doc::Text {
        s.doc()
            .unwrap()
            .model
            .iter()
            .find_map(|e| match &e.kind {
                EntityKind::Text(t) if e.common.layer == LAYER_NUMBERS && t.value == v => Some(t.clone()),
                _ => None,
            })
            .unwrap()
    }

    #[test]
    fn scale_change_resizes_labels_beside_their_points() {
        let mut s = points();
        let before = number_label(&s, "2");
        s.execute("geomeritko", &json!({ "scale": 1000 })).unwrap();
        let after = number_label(&s, "2");
        assert!((after.height - 2.0 * before.height).abs() < 1e-9);
        assert!((after.insert.x - 10.0 - 2.0 * (before.insert.x - 10.0)).abs() < 1e-9);
        assert_eq!(nast::scale(&s), 1000.0);
        assert!(s.execute("geomeritko", &json!({ "scale": 0 })).is_err());
    }

    #[test]
    fn rotating_keeps_labels_upright() {
        let mut s = points();
        s.execute("geootocit", &json!({ "angle": 90, "base": [0, 0] })).unwrap();
        let t = number_label(&s, "2");
        assert!(t.rotation.abs() < 1e-9);
        // Point 2 went from (10, 0) to (0, 10); its label stays to its right.
        assert!((t.insert.x - 0.6).abs() < 1e-6 && t.insert.y > 10.0, "{:?}", t.insert);
        assert!(s.execute("geootocit", &json!({})).is_err());
        assert!((readable(200f64.to_radians()).to_degrees() - 20.0).abs() < 1e-9);
    }

    #[test]
    fn statistics_count_points_and_lengths() {
        let mut s = points();
        let space = s.space();
        add(&mut s, &space, "plot", EntityKind::Line(cadcraft_doc::Line { a: Vec3::new(0.0, 0.0, 0.0), b: Vec3::new(3.0, 4.0, 0.0) })).unwrap();
        let r = s.execute("geostatistika", &json!({})).unwrap();
        assert_eq!(r["points"], 2);
        assert!((r["layers"]["plot"]["length"].as_f64().unwrap() - 5.0).abs() < 1e-9);
    }

    #[test]
    fn arcs_become_segments() {
        let mut s = Session::new();
        let space = s.space();
        let pl = LwPolyline {
            vertices: vec![PolyVertex::with_bulge(Vec2::new(0.0, 0.0), 1.0), PolyVertex::new(Vec2::new(10.0, 0.0))],
            closed: false,
            const_width: 0.0,
            elevation: 0.0,
            plinegen: false,
        };
        let h = add(&mut s, &space, "0", EntityKind::LwPolyline(pl)).unwrap();
        let a = add(
            &mut s,
            &space,
            "0",
            EntityKind::Arc(cadcraft_doc::Arc { center: Vec3::new(0.0, 0.0, 0.0), radius: 5.0, start: 0.0, end: std::f64::consts::PI }),
        )
        .unwrap();
        let r = s.execute("geoobluky", &json!({ "handles": [h.hex(), a.hex()], "tolerance": 0.01 })).unwrap();
        assert_eq!(r["changed"], 2);
        let d = s.doc().unwrap();
        let EntityKind::LwPolyline(pl) = &d.model.get(h).unwrap().kind else { panic!() };
        assert!(pl.vertices.len() > 10 && pl.vertices.iter().all(|v| v.bulge == 0.0));
        // Every vertex on the half circle of radius 5 round (5, 0).
        assert!(pl.vertices.iter().all(|v| ((v.p.x - 5.0).hypot(v.p.y) - 5.0).abs() < 1e-9));
        assert!(d.model.get(a).is_none());
    }

    #[test]
    fn join_3d_pieces() {
        let mut s = Session::new();
        let space = s.space();
        let p1 = add(
            &mut s,
            &space,
            "hrana",
            EntityKind::Polyline3d(Polyline3d { points: vec![Vec3::new(0.0, 0.0, 1.0), Vec3::new(5.0, 0.0, 2.0)], closed: false }),
        )
        .unwrap();
        let p2 =
            add(&mut s, &space, "hrana", EntityKind::Line(cadcraft_doc::Line { a: Vec3::new(9.0, 0.0, 3.0), b: Vec3::new(5.0, 0.0, 2.0) })).unwrap();
        let p3 = add(
            &mut s,
            &space,
            "hrana",
            EntityKind::Polyline3d(Polyline3d { points: vec![Vec3::new(50.0, 0.0, 1.0), Vec3::new(55.0, 0.0, 2.0)], closed: false }),
        )
        .unwrap();
        let r = s.execute("geospoj3d", &json!({ "handles": [p1.hex(), p2.hex(), p3.hex()] })).unwrap();
        assert_eq!(r["made"], 1);
        assert_eq!(r["joined"], 2);
        let d = s.doc().unwrap();
        let joined: Vec<usize> =
            d.model.iter().filter_map(|e| if let EntityKind::Polyline3d(p) = &e.kind { Some(p.points.len()) } else { None }).collect();
        assert!(joined.contains(&3) && joined.contains(&2), "{joined:?}");
    }

    #[test]
    fn texts_follow_a_line() {
        let mut s = points();
        let space = s.space();
        let l =
            add(&mut s, &space, "0", EntityKind::Line(cadcraft_doc::Line { a: Vec3::new(10.0, 10.0, 0.0), b: Vec3::new(0.0, 0.0, 0.0) })).unwrap();
        let hs: Vec<String> = s.doc().unwrap().model.iter().filter(|e| matches!(e.kind, EntityKind::Text(_))).map(|e| e.handle.hex()).collect();
        s.execute("geotextrot", &json!({ "handles": hs, "line": l.hex() })).unwrap();
        // The line runs at 225°, read as 45°.
        assert!((number_label(&s, "1").rotation.to_degrees() - 45.0).abs() < 1e-9);
    }

    #[test]
    fn hostile_input_is_refused_not_panicking() {
        let mut s = points();
        for (c, p) in [
            ("geomeritko", json!({ "scale": "x" })),
            ("geomeritko", json!({ "scale": 500, "from": -1 })),
            ("geootocit", json!({ "angle": 1e300 })),
            ("geootocit", json!({ "line": "zz" })),
            ("geoobluky", json!({ "handles": [], "tolerance": 0 })),
            ("geospoj3d", json!({ "handles": [] })),
            ("geotextrot", json!({ "handles": [] })),
        ] {
            assert!(s.execute(c, &p).is_err(), "{c} {p}");
        }
    }
}
