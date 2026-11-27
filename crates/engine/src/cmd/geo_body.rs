//! Survey add-on: work on the points already in the drawing.
//!
//! - GEOBOD adds one point (picked, with its number, height and code), labelled as GEOBODY
//!   labels points; the next free number is offered.
//! - GEOBODYHROMADNE adds points at the vertices of selected objects (lines, polylines, 3D
//!   polylines, circle centres, block insertion points), numbered on from a start number.
//! - GEOINTERPOLACE gives the height at a place from two points (along their line) or three
//!   (their plane), and adds a point there.
//! - GEOIDENT tells a point's number, coordinates, height and code.
//! - GEOVYSKABODU changes a point's height and its label.
//! - GEOZNULA puts points at height 0 (their height labels stay).
//! - GEOMIMO deletes the points outside (or inside) a closed boundary, with their labels.
//! - GEOCISLAKU writes twelve-digit cadastral point numbers as ZPMZ-point.

use std::collections::BTreeSet;

use cadcraft_doc::{EntityKind, Handle, Space};
use cadcraft_geom::{Vec2, Vec3};
use serde_json::{Value, json};

use super::geo::{LAYER_CODES, LAYER_HEIGHTS, LAYER_HEIGHTS2, LAYER_NUMBERS, LAYER_POINTS, ku_number, label_slots, run_geobody};
use super::geo_calc::{height_at, path_of, say, tin, typed_number};
use super::geo_nastaveni as nast;
use super::geo_pohled::{POINT_BLOCK, point_label_entries};
use super::geo_vypocty::{Frame, point_ref};
use super::{CommandSpec, bad, point_value, str_param, targets};
use crate::{Accept, Input, Interactive, Prompt, Result, Session, Step};

/// Points added or changed at once.
const MAX_POINTS: usize = 200_000;

pub(super) fn specs() -> Vec<CommandSpec> {
    vec![
        CommandSpec::new("geobod", "Add Point...", run_geobod)
            .menu(&["Geo", "Points", "Add Point..."])
            .alias(&["geoaddpoint", "geodoplnbod"])
            .params("{at: [x, y], number?: (default the next free), z?: height or \"tin\" (from the terrain model), code?} → {number, handle, message}")
            .interactive(|_| Ok(Box::new(BodM::default()))),
        CommandSpec::new("geobodyhromadne", "Add Points at Vertices...", run_hromadne)
            .menu(&["Geo", "Points", "Add Points at Vertices..."])
            .alias(&["geobulkpoints"])
            .params("{handles?: objects (default the selection), start?: first number (default the next free), step?: (default 1), prefix?: text before the number, z?: \"vertex\" (default) | \"tin\" | \"none\"} → {imported, skipped, message}")
            .interactive(|_| Ok(Box::new(HromM::default()))),
        CommandSpec::new("geointerpolace", "Interpolate Height...", run_interpolace)
            .menu(&["Geo", "Points", "Interpolate Height..."])
            .alias(&["geointerpolate"])
            .params("{a, b: points (number or [x, y, z]), c?: a third point (then the plane of the three), at: [x, y] or a list of them, insert?: bool add points there (default true), number?: first number} → {heights: [..], message}")
            .interactive(|_| Ok(Box::new(InterM::default()))),
        CommandSpec::new("geoident", "Identify Point", run_ident)
            .menu(&["Geo", "Points", "Identify Point"])
            .alias(&["geoidentify", "geoinfo"])
            .params("{at: [x, y] near the point | number} → {number, y, x, z, code, handle, message}")
            .interactive(|_| Ok(Box::new(PickM { name: "GEOIDENT", msg: "Pick a point to identify", cmd: Pick::Ident, at: None }))),
        CommandSpec::new("geovyskabodu", "Change Point Height...", run_vyska)
            .menu(&["Geo", "Points", "Change Point Height..."])
            .alias(&["geoeditheight"])
            .params("{point: number or [x, y] near it, z: new height} → {number, z, message}")
            .interactive(|_| Ok(Box::new(PickM { name: "GEOVYSKABODU", msg: "Pick the point", cmd: Pick::Height, at: None }))),
        CommandSpec::new("geoznula", "Points to Height 0", run_znula)
            .menu(&["Geo", "Points", "Points to Height 0"])
            .alias(&["geozeroz"])
            .params("{handles?: points (default the selected ones, else all on GEO_body)} → {changed, message}"),
        CommandSpec::new("geomimo", "Delete Points Outside...", run_mimo)
            .menu(&["Geo", "Points", "Delete Points Outside..."])
            .alias(&["geooutside"])
            .params("{boundary: handle of a closed polyline, inside?: bool, delete the points inside instead} → {deleted, message}")
            .interactive(|_| Ok(Box::new(MimoM::default()))),
        CommandSpec::new("geocislaku", "Point Numbers to Cadastral Form", run_cislaku)
            .menu(&["Geo", "Points", "Point Numbers to Cadastral Form"])
            .alias(&["geokunumbers"])
            .params("{} → {changed, message}"),
    ]
}

/// A survey point of the drawing and its labels.
#[derive(Clone, Debug)]
pub(super) struct Pt {
    pub(super) handle: Handle,
    pub(super) p: Vec3,
    pub(super) number: Option<(String, Handle)>,
    pub(super) height: Option<(String, Handle)>,
    pub(super) code: Option<(String, Handle)>,
    pub(super) z2: Option<(String, Handle)>,
}

pub(super) fn survey_points(s: &Session, space: &Space) -> Result<Vec<Pt>> {
    let n = nast::load(s);
    let d = s.doc()?;
    let Some(st) = d.space(space) else { return Ok(Vec::new()) };
    let pts: Vec<(Handle, Vec3)> = st
        .iter()
        .filter(|e| e.common.layer.eq_ignore_ascii_case(LAYER_POINTS))
        .filter_map(|e| match &e.kind {
            EntityKind::Point(q) if q.p.x.is_finite() && q.p.y.is_finite() && q.p.z.is_finite() => Some((e.handle, q.p)),
            _ => None,
        })
        .take(MAX_POINTS * 5)
        .collect();
    let at: Vec<Vec3> = pts.iter().map(|p| p.1).collect();
    let get = |layer: &str, tag: &str| point_label_entries(st, &at, layer, tag, &label_slots(&n, layer));
    let (nums, hs, cs, z2s) = (get(LAYER_NUMBERS, "CISLO"), get(LAYER_HEIGHTS, "VYSKA"), get(LAYER_CODES, "KOD"), get(LAYER_HEIGHTS2, "VYSKA2"));
    Ok(pts
        .into_iter()
        .enumerate()
        .map(|(i, (handle, p))| Pt {
            handle,
            p,
            number: nums.get(i).cloned().flatten(),
            height: hs.get(i).cloned().flatten(),
            code: cs.get(i).cloned().flatten(),
            z2: z2s.get(i).cloned().flatten(),
        })
        .collect())
}

/// The next free point number: one more than the largest number in the drawing.
fn next_free(pts: &[Pt]) -> u64 {
    pts.iter().filter_map(|p| p.number.as_ref()).filter_map(|(n, _)| n.parse::<u64>().ok()).max().unwrap_or(0).saturating_add(1)
}

fn check_number(t: &str, cmd: &str) -> Result<String> {
    let t = t.trim();
    if t.is_empty() || t.chars().count() > 64 || t.chars().any(char::is_whitespace) {
        return Err(bad(cmd, "a point number must be 1 to 64 characters without spaces"));
    }
    Ok(t.to_string())
}

/// The survey point nearest `at` within a few label heights.
fn nearest(s: &Session, pts: &[Pt], at: Vec2) -> Option<usize> {
    let n = nast::load(s);
    let reach = (5.0 * n.height(&n.numbers)).max(0.05);
    pts.iter()
        .enumerate()
        .map(|(i, p)| (i, (p.p.x - at.x).hypot(p.p.y - at.y)))
        .filter(|(_, d)| *d <= reach)
        .min_by(|a, b| a.1.total_cmp(&b.1))
        .map(|(i, _)| i)
}

fn fmt(v: f64) -> String {
    format!("{v:.4}")
}

/// Add points through GEOBODY's labelling: (number, drawing position, height, code).
fn add_points(s: &mut Session, list: &[(String, Vec3, Option<f64>, String)]) -> Result<Value> {
    if list.is_empty() {
        return Err(bad("geobod", "no points to add"));
    }
    let frame = Frame::of(&list.iter().map(|p| p.1.xy()).collect::<Vec<_>>());
    let mut text = String::new();
    for (num, at, z, code) in list {
        let (y, x) = frame.to_yx(at.xy());
        let z = z.map_or_else(|| "-".to_string(), fmt);
        // The code goes last; "-" marks a missing one (and a missing height).
        let code = if code.trim().is_empty() { "-".to_string() } else { code.trim().replace(['\n', '\r', ';', '\t'], " ") };
        text.push_str(&format!("{num} {} {} {z} {code}\n", fmt(y), fmt(x)));
    }
    let r = run_geobody(s, &json!({ "text": text, "columns": "PYXZK", "sjtsk": frame.sjtsk() }))?;
    // GEOBODY read "-" as a code and no height: take those labels back out.
    let space = s.space();
    let handles: Vec<Handle> = s.selection();
    let pts = survey_points(s, &space)?;
    let d = s.doc_mut()?;
    for p in pts.iter().filter(|p| handles.contains(&p.handle)) {
        if let Some((v, h)) = &p.code
            && v == "-"
        {
            d.remove_entity(*h);
        }
    }
    Ok(r)
}

fn run_geobod(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "geobod";
    let at = p.get("at").and_then(point_value).ok_or_else(|| bad(CMD, "`at`: [x, y] is required"))?;
    if at.x.abs() > 1e9 || at.y.abs() > 1e9 {
        return Err(bad(CMD, "`at` is too far away"));
    }
    let space = s.space();
    let pts = survey_points(s, &space)?;
    let number = match p.get("number") {
        None | Some(Value::Null) => next_free(&pts).to_string(),
        Some(Value::Number(n)) => check_number(&n.to_string(), CMD)?,
        Some(Value::String(t)) => check_number(t, CMD)?,
        Some(_) => return Err(bad(CMD, "`number` must be a point number")),
    };
    if pts.iter().any(|q| q.number.as_ref().is_some_and(|(n, _)| *n == number)) {
        return Err(bad(CMD, format!("point {number} is already in the drawing")));
    }
    let z = match p.get("z") {
        None | Some(Value::Null) => None,
        Some(Value::String(t)) if t.eq_ignore_ascii_case("tin") => {
            Some(height_at(&tin(s, &space)?, at).ok_or_else(|| bad(CMD, "the point is outside the terrain model (GEODMT)"))?)
        }
        Some(v) => Some(
            v.as_f64()
                .or_else(|| v.as_str().and_then(typed_number))
                .filter(|z| z.is_finite() && z.abs() < 1e6)
                .ok_or_else(|| bad(CMD, "`z` must be a height"))?,
        ),
    };
    let code = str_param(p, "code").unwrap_or("").chars().take(64).collect::<String>();
    add_points(s, &[(number.clone(), Vec3::new(at.x, at.y, 0.0), z, code)])?;
    let handle = s.selection().first().map(|h| h.hex()).unwrap_or_default();
    Ok(json!({
        "number": number,
        "handle": handle,
        "message": format!("Point {number} added{}", z.map(|z| format!(" at height {z:.3}")).unwrap_or_default()),
    }))
}

/// The vertices of an object, with their heights where it has them.
fn vertices(kind: &EntityKind) -> Vec<Vec3> {
    match kind {
        EntityKind::Line(l) => vec![l.a, l.b],
        EntityKind::LwPolyline(pl) => pl.vertices.iter().map(|v| Vec3::new(v.p.x, v.p.y, pl.elevation)).collect(),
        EntityKind::Polyline3d(pl) => pl.points.clone(),
        EntityKind::Circle(c) => vec![c.center],
        EntityKind::Arc(a) => vec![
            Vec3::new(a.center.x + a.radius * a.start.cos(), a.center.y + a.radius * a.start.sin(), a.center.z),
            Vec3::new(a.center.x + a.radius * a.end.cos(), a.center.y + a.radius * a.end.sin(), a.center.z),
        ],
        EntityKind::Point(q) => vec![q.p],
        EntityKind::Insert(i) => vec![i.insert],
        _ => Vec::new(),
    }
}

fn run_hromadne(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "geobodyhromadne";
    let hs = targets(s, p)?;
    if hs.is_empty() {
        return Err(bad(CMD, "select the objects whose vertices become points"));
    }
    let space = s.space();
    let pts = survey_points(s, &space)?;
    let zmode = str_param(p, "z").unwrap_or("vertex").to_ascii_lowercase();
    if !["vertex", "tin", "none"].contains(&zmode.as_str()) {
        return Err(bad(CMD, "`z` must be vertex, tin or none"));
    }
    let start = match p.get("start") {
        None | Some(Value::Null) => next_free(&pts),
        Some(v) => v.as_u64().or_else(|| v.as_str().and_then(|t| t.trim().parse().ok())).ok_or_else(|| bad(CMD, "`start` must be a whole number"))?,
    };
    let step = p.get("step").and_then(Value::as_u64).unwrap_or(1).clamp(1, 1_000_000);
    let prefix: String = str_param(p, "prefix").unwrap_or("").trim().chars().take(32).collect();
    if prefix.chars().any(char::is_whitespace) {
        return Err(bad(CMD, "`prefix` can't hold spaces"));
    }
    let tris = if zmode == "tin" { tin(s, &space)? } else { Vec::new() };
    let mut verts: Vec<Vec3> = Vec::new();
    {
        let d = s.doc()?;
        let st = d.space(&space).ok_or_else(|| bad(CMD, "no drawing space"))?;
        for h in &hs {
            let Some(e) = st.get(*h) else { continue };
            // Survey points are points already.
            if e.common.layer.eq_ignore_ascii_case(LAYER_POINTS) {
                continue;
            }
            verts.extend(
                vertices(&e.kind).into_iter().filter(|v| v.x.is_finite() && v.y.is_finite() && v.z.is_finite() && v.x.abs() < 1e9 && v.y.abs() < 1e9),
            );
            if verts.len() > MAX_POINTS {
                return Err(bad(CMD, format!("more than {MAX_POINTS} vertices")));
            }
        }
    }
    // One point per place: not where a point is already (within a millimetre).
    let key = |v: Vec3| ((v.x * 1000.0).round() as i64, (v.y * 1000.0).round() as i64);
    let mut seen: BTreeSet<(i64, i64)> = pts.iter().map(|q| key(q.p)).collect();
    let mut list = Vec::new();
    let mut skipped = 0;
    let mut num = start;
    for v in verts {
        if !seen.insert(key(v)) {
            skipped += 1;
            continue;
        }
        let z = match zmode.as_str() {
            "vertex" => Some(v.z),
            "tin" => height_at(&tris, v.xy()),
            _ => None,
        };
        list.push((format!("{prefix}{num}"), v, z, String::new()));
        num = num.saturating_add(step);
    }
    if list.is_empty() {
        return Err(bad(CMD, format!("no new points: all {skipped} vertices have a point already")));
    }
    let added = list.len();
    add_points(s, &list)?;
    Ok(json!({
        "imported": added,
        "skipped": skipped,
        "message": format!("{added} points added ({prefix}{start} to {prefix}{}){}", num.saturating_sub(step), if skipped > 0 { format!("; {skipped} vertices had a point already") } else { String::new() }),
    }))
}

/// The height at `at` from two points (along their line) or three (their plane).
fn interpolate(refs: &[Vec3], at: Vec2) -> Option<f64> {
    match refs {
        [a, b] => {
            let (dx, dy) = (b.x - a.x, b.y - a.y);
            let l2 = dx * dx + dy * dy;
            if l2 < 1e-12 {
                return None;
            }
            let t = ((at.x - a.x) * dx + (at.y - a.y) * dy) / l2;
            let z = a.z + t * (b.z - a.z);
            z.is_finite().then_some(z)
        }
        [a, b, c] => super::geo_calc::plane(&[*a, *b, *c]).map(|(pa, pb, pc)| pa * at.x + pb * at.y + pc).filter(|z| z.is_finite()),
        _ => None,
    }
}

fn run_interpolace(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "geointerpolace";
    let space = s.space();
    let mut refs = vec![point_ref(s, &space, p.get("a"), "a", CMD)?, point_ref(s, &space, p.get("b"), "b", CMD)?];
    if p.get("c").is_some_and(|v| !v.is_null()) {
        refs.push(point_ref(s, &space, p.get("c"), "c", CMD)?);
    }
    let at: Vec<Vec2> = match p.get("at") {
        Some(Value::Array(a)) if a.first().is_some_and(Value::is_array) => {
            a.iter().map(point_value).collect::<Option<Vec<_>>>().ok_or_else(|| bad(CMD, "`at` must be [x, y] or a list of them"))?
        }
        Some(v) => vec![point_value(v).ok_or_else(|| bad(CMD, "`at` must be [x, y] or a list of them"))?],
        None => return Err(bad(CMD, "`at` is required")),
    };
    if at.len() > MAX_POINTS {
        return Err(bad(CMD, format!("at most {MAX_POINTS} places")));
    }
    let mut heights = Vec::with_capacity(at.len());
    for q in &at {
        heights.push(
            interpolate(&refs, *q)
                .ok_or_else(|| bad(CMD, if refs.len() == 2 { "the two points are at the same place" } else { "the three points are in a line" }))?,
        );
    }
    let insert = p.get("insert").and_then(Value::as_bool).unwrap_or(true);
    if insert {
        let pts = survey_points(s, &space)?;
        let mut num = match p.get("number") {
            None | Some(Value::Null) => next_free(&pts),
            Some(v) => {
                v.as_u64().or_else(|| v.as_str().and_then(|t| t.trim().parse().ok())).ok_or_else(|| bad(CMD, "`number` must be a whole number"))?
            }
        };
        let mut list = Vec::new();
        for (q, z) in at.iter().zip(&heights) {
            list.push((num.to_string(), Vec3::new(q.x, q.y, 0.0), Some(*z), String::new()));
            num = num.saturating_add(1);
        }
        add_points(s, &list)?;
    }
    let shown: Vec<String> = heights.iter().map(|z| format!("{z:.3}")).collect();
    Ok(json!({
        "heights": heights,
        "message": format!("Height {}{}", shown.join(", "), if insert { " (points added)" } else { "" }),
    }))
}

fn find(s: &Session, p: &Value, key: &str, cmd: &str) -> Result<(Vec<Pt>, usize)> {
    let space = s.space();
    let pts = survey_points(s, &space)?;
    let i = match p.get(key) {
        Some(Value::Array(_)) => {
            let at = p.get(key).and_then(point_value).ok_or_else(|| bad(cmd, format!("`{key}` must be [x, y] or a point number")))?;
            nearest(s, &pts, at).ok_or_else(|| bad(cmd, "no survey point there"))?
        }
        Some(Value::String(t)) => {
            pts.iter().position(|q| q.number.as_ref().is_some_and(|(n, _)| n == t.trim())).ok_or_else(|| bad(cmd, format!("no point {t}")))?
        }
        Some(Value::Number(n)) => {
            let t = n.to_string();
            pts.iter().position(|q| q.number.as_ref().is_some_and(|(m, _)| *m == t)).ok_or_else(|| bad(cmd, format!("no point {t}")))?
        }
        _ => return Err(bad(cmd, format!("`{key}`: [x, y] or a point number is required"))),
    };
    Ok((pts, i))
}

fn run_ident(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "geoident";
    let key = if p.get("at").is_some() { "at" } else { "number" };
    let (pts, i) = find(s, p, key, CMD)?;
    let q = pts.get(i).ok_or_else(|| bad(CMD, "no survey point there"))?;
    let frame = Frame::of(&[q.p.xy()]);
    let (y, x) = frame.to_yx(q.p.xy());
    let number = q.number.as_ref().map(|n| n.0.clone()).unwrap_or_default();
    let code = q.code.as_ref().map(|c| c.0.clone()).unwrap_or_default();
    Ok(json!({
        "number": number, "y": y, "x": x, "z": q.p.z, "code": code, "handle": q.handle.hex(),
        "message": format!(
            "Point {}: Y = {y:.3}, X = {x:.3}, Z = {:.3}{}",
            if number.is_empty() { "(no number)" } else { number.as_str() },
            q.p.z,
            if code.is_empty() { String::new() } else { format!(", code {code}") }
        ),
    }))
}

/// Write `value` into a label (a text, or the GEO_BOD block's attribute `tag`).
fn set_label(s: &mut Session, h: Handle, tag: &str, value: &str) -> Result<()> {
    s.doc_mut()?
        .modify_entity(h, |e| match &mut e.kind {
            EntityKind::Text(t) => t.value = value.to_string(),
            EntityKind::Insert(i) if i.block.eq_ignore_ascii_case(POINT_BLOCK) => {
                if let Some(a) = i.attribs.iter_mut().find(|a| a.tag.eq_ignore_ascii_case(tag)) {
                    a.text.value = value.to_string();
                }
            }
            _ => {}
        })
        .map_err(|e| bad("geo", e.to_string()))
}

fn run_vyska(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "geovyskabodu";
    let z = p
        .get("z")
        .and_then(|v| v.as_f64().or_else(|| v.as_str().and_then(typed_number)))
        .filter(|z| z.is_finite() && z.abs() < 1e6)
        .ok_or_else(|| bad(CMD, "`z`: the new height is required"))?;
    let (pts, i) = find(s, p, "point", CMD)?;
    let q = pts.get(i).cloned().ok_or_else(|| bad(CMD, "no survey point there"))?;
    let n = nast::load(s);
    s.doc_mut()?
        .modify_entity(q.handle, |e| {
            if let EntityKind::Point(pt) = &mut e.kind {
                pt.p.z = if n.z3d { z } else { 0.0 };
            }
        })
        .map_err(|e| bad(CMD, e.to_string()))?;
    let code = q.code.as_ref().map(|c| c.0.clone()).unwrap_or_default();
    let label = n.height_text(z, &code);
    match &q.height {
        Some((_, h)) => set_label(s, *h, "VYSKA", &label)?,
        None => s.echo("The point has no height label; only its height changed."),
    }
    let number = q.number.map(|n| n.0).unwrap_or_default();
    Ok(json!({ "number": number, "z": z, "message": format!("Point {number}: height {label}") }))
}

fn run_znula(s: &mut Session, p: &Value) -> Result<Value> {
    let space = s.space();
    let pts = survey_points(s, &space)?;
    let chosen: Vec<Handle> = targets(s, p)?;
    let set: BTreeSet<Handle> = chosen.into_iter().collect();
    let pick: Vec<Handle> = pts.iter().map(|q| q.handle).filter(|h| set.is_empty() || set.contains(h)).collect();
    let d = s.doc_mut()?;
    let mut changed = 0;
    for h in &pick {
        let _ = d.modify_entity(*h, |e| {
            if let EntityKind::Point(pt) = &mut e.kind
                && pt.p.z != 0.0
            {
                pt.p.z = 0.0;
                changed += 1;
            }
        });
    }
    Ok(json!({ "changed": changed, "message": format!("{changed} points put at height 0 (their height labels stay)") }))
}

fn run_mimo(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "geomimo";
    let space = s.space();
    let b = super::geo_calc::one_target(s, p, "boundary", CMD)?;
    let (poly, closed) = path_of(s, b, &space).ok_or_else(|| bad(CMD, "the boundary must be a polyline"))?;
    if !closed && poly.first().zip(poly.last()).is_none_or(|(a, z)| (a.x - z.x).hypot(a.y - z.y) > 1e-9) || poly.len() < 3 {
        return Err(bad(CMD, "the boundary must be a closed polyline"));
    }
    let inside = p.get("inside").and_then(Value::as_bool).unwrap_or(false);
    let pts = survey_points(s, &space)?;
    let gone: Vec<&Pt> = pts.iter().filter(|q| super::geo_rek::inside(q.p.xy(), &poly) == inside).collect();
    let mut handles: BTreeSet<Handle> = BTreeSet::new();
    for q in &gone {
        handles.insert(q.handle);
        for l in [&q.number, &q.height, &q.code, &q.z2].into_iter().flatten() {
            handles.insert(l.1);
        }
    }
    let n = gone.len();
    let d = s.doc_mut()?;
    for h in handles {
        d.remove_entity(h);
    }
    Ok(json!({ "deleted": n, "message": format!("{n} points {} the boundary deleted with their labels", if inside { "inside" } else { "outside" }) }))
}

fn run_cislaku(s: &mut Session, _p: &Value) -> Result<Value> {
    let space = s.space();
    let pts = survey_points(s, &space)?;
    let mut changed = 0;
    for q in pts {
        if let Some((v, h)) = q.number {
            let k = ku_number(&v);
            if k != v {
                set_label(s, h, "CISLO", &k)?;
                changed += 1;
            }
        }
    }
    Ok(json!({ "changed": changed, "message": format!("{changed} point numbers written as ZPMZ-point") }))
}

fn echo_err(s: &mut Session, r: Result<Value>) -> bool {
    let ok = r.is_ok();
    say(s, r);
    ok
}

/// GEOBOD: pick, number, height, code; again until Enter.
#[derive(Default)]
struct BodM {
    at: Option<Vec2>,
    number: Option<String>,
    z: Option<Option<String>>,
}

impl Interactive for BodM {
    fn name(&self) -> &'static str {
        "GEOBOD"
    }
    fn prompt(&self, s: &Session) -> Prompt {
        let space = s.space();
        match (&self.at, &self.number, &self.z) {
            (None, ..) => Prompt::new("Pick the point's place", Accept::POINT),
            (Some(_), None, _) => {
                let next = survey_points(s, &space).map(|p| next_free(&p)).unwrap_or(1);
                Prompt::new("Point number", Accept::TEXT).default(next.to_string())
            }
            (Some(at), Some(_), None) => {
                let p = Prompt::new("Height (Enter: none)", Accept::NUMBER);
                match tin(s, &space).ok().and_then(|t| height_at(&t, *at)) {
                    Some(z) => p.default(format!("{z:.3}")),
                    None => p,
                }
            }
            _ => Prompt::new("Code (Enter: none)", Accept::TEXT),
        }
    }
    fn input(&mut self, s: &mut Session, i: Input) -> Result<Step> {
        if matches!(i, Input::Cancel) {
            return Ok(Step::Cancel);
        }
        let Some(at) = self.at else {
            return match i {
                Input::Point(q) => {
                    self.at = Some(q);
                    Ok(Step::Continue)
                }
                Input::Enter => Ok(Step::Done),
                _ => Ok(Step::Continue),
            };
        };
        let t = match &i {
            Input::Text(t) | Input::Keyword(t) => t.trim().to_string(),
            Input::Enter => String::new(),
            _ => return Ok(Step::Continue),
        };
        let prompt_default = self.prompt(s).default;
        let t = if t.is_empty() { prompt_default.unwrap_or_default() } else { t };
        if self.number.is_none() {
            self.number = Some(t);
            return Ok(Step::Continue);
        }
        if self.z.is_none() {
            if !t.is_empty() && typed_number(&t).is_none() {
                s.echo("Enter a height in metres, like 305.20, or Enter for none.");
                return Ok(Step::Continue);
            }
            self.z = Some((!t.is_empty()).then_some(t));
            return Ok(Step::Continue);
        }
        let z = self.z.clone().flatten();
        let r = run_geobod(s, &json!({ "at": [at.x, at.y], "number": self.number, "z": z, "code": t }));
        echo_err(s, r);
        *self = BodM::default();
        Ok(Step::Continue)
    }
}

/// GEOBODYHROMADNE: objects, then the first number.
#[derive(Default)]
struct HromM {
    picked: Option<Vec<Handle>>,
}

impl Interactive for HromM {
    fn name(&self) -> &'static str {
        "GEOBODYHROMADNE"
    }
    fn prompt(&self, s: &Session) -> Prompt {
        match self.picked {
            None => Prompt::new("Select objects whose vertices become points", Accept::SELECT),
            Some(_) => {
                let space = s.space();
                let next = survey_points(s, &space).map(|p| next_free(&p)).unwrap_or(1);
                Prompt::new("First point number", Accept::NUMBER).default(next.to_string())
            }
        }
    }
    fn input(&mut self, s: &mut Session, i: Input) -> Result<Step> {
        match (&self.picked, i) {
            (_, Input::Cancel) => Ok(Step::Cancel),
            (None, Input::Pick(hs)) => {
                self.picked = Some(hs);
                Ok(Step::Continue)
            }
            (Some(hs), Input::Text(t) | Input::Keyword(t)) => {
                let hs: Vec<String> = hs.iter().map(|h| h.hex()).collect();
                let r = run_hromadne(s, &json!({ "handles": hs, "start": t.trim() }));
                echo_err(s, r);
                Ok(Step::Done)
            }
            (Some(hs), Input::Enter) => {
                let hs: Vec<String> = hs.iter().map(|h| h.hex()).collect();
                let r = run_hromadne(s, &json!({ "handles": hs }));
                echo_err(s, r);
                Ok(Step::Done)
            }
            _ => Ok(Step::Continue),
        }
    }
}

/// GEOINTERPOLACE: two or three known points, then places, until Enter.
#[derive(Default)]
struct InterM {
    refs: Vec<Vec3>,
    done_refs: bool,
}

impl Interactive for InterM {
    fn name(&self) -> &'static str {
        "GEOINTERPOLACE"
    }
    fn prompt(&self, _s: &Session) -> Prompt {
        match (self.refs.len(), self.done_refs) {
            (0, _) => Prompt::new("Pick the first known point", Accept::POINT),
            (1, _) => Prompt::new("Pick the second known point", Accept::POINT),
            (2, false) => Prompt::new("Pick the third known point (Enter: along the line of two)", Accept::POINT),
            _ => Prompt::new("Pick where to add a point (Enter: done)", Accept::POINT),
        }
    }
    fn input(&mut self, s: &mut Session, i: Input) -> Result<Step> {
        let space = s.space();
        match i {
            Input::Cancel => Ok(Step::Cancel),
            Input::Enter if self.refs.len() == 2 && !self.done_refs => {
                self.done_refs = true;
                Ok(Step::Continue)
            }
            Input::Enter if self.done_refs => Ok(Step::Done),
            Input::Point(q) if !self.done_refs => {
                let pts = survey_points(s, &space)?;
                match nearest(s, &pts, q).and_then(|k| pts.get(k)) {
                    Some(pt) => {
                        self.refs.push(pt.p);
                        if self.refs.len() == 3 {
                            self.done_refs = true;
                        }
                    }
                    None => s.echo("Pick a survey point."),
                }
                Ok(Step::Continue)
            }
            Input::Point(q) => {
                let refs: Vec<Value> = self.refs.iter().map(|v| json!([v.x, v.y, v.z])).collect();
                let mut p = json!({ "a": refs.first(), "b": refs.get(1), "at": [q.x, q.y] });
                if let (Some(c), Some(o)) = (refs.get(2), p.as_object_mut()) {
                    o.insert("c".into(), c.clone());
                }
                let r = run_interpolace(s, &p);
                echo_err(s, r);
                Ok(Step::Continue)
            }
            _ => Ok(Step::Continue),
        }
    }
}

#[derive(Clone, Copy, PartialEq)]
enum Pick {
    Ident,
    Height,
}

/// Pick a point (and, for a height, type it).
struct PickM {
    name: &'static str,
    msg: &'static str,
    cmd: Pick,
    at: Option<Vec2>,
}

impl Interactive for PickM {
    fn name(&self) -> &'static str {
        self.name
    }
    fn prompt(&self, _s: &Session) -> Prompt {
        match self.at {
            None => Prompt::new(self.msg, Accept::POINT),
            Some(_) => Prompt::new("New height", Accept::NUMBER),
        }
    }
    fn input(&mut self, s: &mut Session, i: Input) -> Result<Step> {
        match (self.at, i) {
            (_, Input::Cancel) => Ok(Step::Cancel),
            (None, Input::Enter) => Ok(Step::Done),
            (None, Input::Point(q)) => {
                if self.cmd == Pick::Ident {
                    let r = run_ident(s, &json!({ "at": [q.x, q.y] }));
                    echo_err(s, r);
                } else {
                    let space = s.space();
                    let pts = survey_points(s, &space)?;
                    match nearest(s, &pts, q) {
                        Some(_) => self.at = Some(q),
                        None => s.echo("No survey point there."),
                    }
                }
                Ok(Step::Continue)
            }
            (Some(at), Input::Text(t) | Input::Keyword(t)) => {
                let r = run_vyska(s, &json!({ "point": [at.x, at.y], "z": t.trim() }));
                if echo_err(s, r) {
                    self.at = None;
                }
                Ok(Step::Continue)
            }
            (Some(_), Input::Enter) => {
                self.at = None;
                Ok(Step::Continue)
            }
            _ => Ok(Step::Continue),
        }
    }
}

/// GEOMIMO: the boundary, then inside or outside.
#[derive(Default)]
struct MimoM {
    boundary: Option<Handle>,
}

impl Interactive for MimoM {
    fn name(&self) -> &'static str {
        "GEOMIMO"
    }
    fn prompt(&self, _s: &Session) -> Prompt {
        match self.boundary {
            None => Prompt::new("Select the closed boundary polyline", Accept::SELECT),
            Some(_) => Prompt::new("Delete the points", Accept::TEXT).kw(&["Outside", "Inside"]).default("Outside"),
        }
    }
    fn input(&mut self, s: &mut Session, i: Input) -> Result<Step> {
        match (self.boundary, i) {
            (_, Input::Cancel) => Ok(Step::Cancel),
            (None, Input::Pick(hs)) => {
                self.boundary = hs.first().copied();
                Ok(Step::Continue)
            }
            (Some(b), Input::Enter) => {
                let r = run_mimo(s, &json!({ "boundary": b.hex() }));
                echo_err(s, r);
                Ok(Step::Done)
            }
            (Some(b), Input::Text(t) | Input::Keyword(t)) => {
                let inside = self.prompt(s).match_keyword(&t).as_deref() == Some("Inside");
                let r = run_mimo(s, &json!({ "boundary": b.hex(), "inside": inside }));
                echo_err(s, r);
                Ok(Step::Done)
            }
            _ => Ok(Step::Continue),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn texts(s: &Session, layer: &str) -> Vec<String> {
        s.doc()
            .unwrap()
            .model
            .iter()
            .filter(|e| e.common.layer == layer)
            .filter_map(|e| if let EntityKind::Text(t) = &e.kind { Some(t.value.clone()) } else { None })
            .collect()
    }

    fn with_points() -> Session {
        let mut s = Session::new();
        s.execute("geobody", &json!({ "text": "1 0 0 100\n2 10 0 110\n3 0 10 120\n", "sjtsk": false })).unwrap();
        s
    }

    #[test]
    fn add_a_point_and_identify_it() {
        let mut s = with_points();
        let r = s.execute("geobod", &json!({ "at": [5, 5], "z": 104.5, "code": "plot" })).unwrap();
        assert_eq!(r["number"], "4");
        assert!(texts(&s, LAYER_NUMBERS).contains(&"4".to_string()));
        assert!(texts(&s, LAYER_CODES).contains(&"plot".to_string()));
        let r = s.execute("geoident", &json!({ "at": [5.1, 5.1] })).unwrap();
        assert_eq!(r["number"], "4");
        assert_eq!(r["code"], "plot");
        assert!((r["z"].as_f64().unwrap() - 104.5).abs() < 1e-9);
        // A taken number is refused; no height and no code leave no labels for them.
        assert!(s.execute("geobod", &json!({ "at": [7, 7], "number": "4" })).is_err());
        s.execute("geobod", &json!({ "at": [8, 8], "number": "A1" })).unwrap();
        assert!(!texts(&s, LAYER_CODES).contains(&"-".to_string()));
    }

    #[test]
    fn the_point_prompt_adds_points_until_enter() {
        let mut s = with_points();
        s.start("geobod").unwrap();
        s.input(Input::Point(Vec2::new(20.0, 20.0))).unwrap();
        for t in ["", "200", "K1"] {
            s.input(Input::Text(t.into())).unwrap();
        }
        s.input(Input::Enter).unwrap();
        assert!(texts(&s, LAYER_NUMBERS).contains(&"4".to_string()));
        assert!(texts(&s, LAYER_HEIGHTS).contains(&"200.00".to_string()));
    }

    #[test]
    fn interpolation_along_a_line_and_in_a_plane() {
        let mut s = with_points();
        let r = s.execute("geointerpolace", &json!({ "a": "1", "b": "2", "at": [5, 3], "insert": false })).unwrap();
        assert!((r["heights"][0].as_f64().unwrap() - 105.0).abs() < 1e-9);
        let r = s.execute("geointerpolace", &json!({ "a": "1", "b": "2", "c": "3", "at": [[5, 5], [10, 10]] })).unwrap();
        assert!((r["heights"][0].as_f64().unwrap() - 115.0).abs() < 1e-9);
        assert!((r["heights"][1].as_f64().unwrap() - 130.0).abs() < 1e-9);
        assert_eq!(texts(&s, LAYER_NUMBERS).len(), 5);
        assert!(s.execute("geointerpolace", &json!({ "a": "1", "b": "1", "at": [1, 1] })).is_err());
    }

    #[test]
    fn bulk_points_from_vertices() {
        let mut s = with_points();
        let space = s.space();
        let pl = cadcraft_doc::Polyline3d {
            points: vec![Vec3::new(0.0, 0.0, 100.0), Vec3::new(20.0, 0.0, 101.0), Vec3::new(20.0, 20.0, 102.0)],
            closed: false,
        };
        let h = super::super::geo_calc::add(&mut s, &space, "0", EntityKind::Polyline3d(pl)).unwrap();
        let r = s.execute("geobodyhromadne", &json!({ "handles": [h.hex()], "start": 100, "prefix": "N" })).unwrap();
        assert_eq!(r["imported"], 2, "{r}");
        assert_eq!(r["skipped"], 1);
        let nums = texts(&s, LAYER_NUMBERS);
        assert!(nums.contains(&"N100".to_string()) && nums.contains(&"N101".to_string()));
        assert!(texts(&s, LAYER_HEIGHTS).contains(&"102.00".to_string()));
    }

    #[test]
    fn height_edit_zero_outside_and_ku() {
        let mut s = with_points();
        s.execute("geovyskabodu", &json!({ "point": "2", "z": 111.25 })).unwrap();
        assert!(texts(&s, LAYER_HEIGHTS).contains(&"111.25".to_string()));
        let r = s.execute("geoznula", &json!({})).unwrap();
        assert_eq!(r["changed"], 3);
        let space = s.space();
        let pl = cadcraft_doc::LwPolyline {
            vertices: [(-1.0, -1.0), (11.0, -1.0), (11.0, 1.0), (-1.0, 1.0)]
                .iter()
                .map(|(x, y)| cadcraft_geom::PolyVertex::new(Vec2::new(*x, *y)))
                .collect(),
            closed: true,
            const_width: 0.0,
            elevation: 0.0,
            plinegen: false,
        };
        let b = super::super::geo_calc::add(&mut s, &space, "0", EntityKind::LwPolyline(pl)).unwrap();
        let r = s.execute("geomimo", &json!({ "boundary": b.hex() })).unwrap();
        assert_eq!(r["deleted"], 1);
        assert_eq!(texts(&s, LAYER_NUMBERS), ["1", "2"]);
        assert_eq!(texts(&s, LAYER_HEIGHTS).len(), 2);
        let mut s = Session::new();
        s.execute("geobody", &json!({ "text": "001012500002 0 0 1\n", "sjtsk": false })).unwrap();
        assert_eq!(s.execute("geocislaku", &json!({})).unwrap()["changed"], 1);
        assert_eq!(texts(&s, LAYER_NUMBERS), ["1250-2"]);
    }

    #[test]
    fn hostile_input_is_refused_not_panicking() {
        let mut s = with_points();
        for (c, p) in [
            ("geobod", json!({})),
            ("geobod", json!({ "at": [1e300, 0] })),
            ("geobod", json!({ "at": [1, 1], "number": "a b" })),
            ("geobod", json!({ "at": [1, 1], "z": "x" })),
            ("geobod", json!({ "at": [1, 1], "z": "tin" })),
            ("geobodyhromadne", json!({ "handles": [] })),
            ("geobodyhromadne", json!({ "handles": ["FFFFFF"], "z": "boom" })),
            ("geointerpolace", json!({ "a": "1", "b": "2" })),
            ("geointerpolace", json!({ "a": "99", "b": "2", "at": [0, 0] })),
            ("geoident", json!({ "at": [500, 500] })),
            ("geovyskabodu", json!({ "point": "1", "z": "NaN" })),
            ("geomimo", json!({ "boundary": "FFFFFF" })),
        ] {
            assert!(s.execute(c, &p).is_err(), "{c} {p}");
        }
    }
}
