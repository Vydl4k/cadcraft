//! Survey add-on: hand edits of the terrain model, without computing it again.
//!
//! - GEODMTBOD: insert a point into the model. The triangles whose circumcircle holds the new
//!   point (and that touch the triangle it lies in) are replaced by a fan from the point, so the
//!   model stays a Delaunay triangulation around it (Bowyer–Watson).
//! - GEODMTODEBRAT: take a vertex out of the model. Inside the model its triangles are replaced
//!   by a triangulation of the hole; on the model's edge they are just removed.
//! - GEODMTSMAZAT: delete the triangle under the picked point (a hole, or a notch on the edge).
//! - GEODMTVYSKA: change the height of a vertex in every triangle that uses it.
//!
//! The model is the 3DFACEs on GEO_DMT (or `layer`). Vertices are matched to the millimetre.
//! Run GEOVRSTEVNICE again afterwards to update the contours.

use std::collections::{HashMap, HashSet};

use cadcraft_doc::{EntityKind, Face3d, Handle, Space};
use cadcraft_geom::{Vec2, Vec3};
use serde_json::{Value, json};

use super::geo_calc::{add, ear_clip, say, typed_number};
use super::geo_dmt::LAYER_TIN;
use super::{CommandSpec, bad, point_value};
use crate::{Accept, Input, Interactive, Prompt, Result, Session, Step};

/// Triangles touched by one edit.
const MAX_CAVITY: usize = 100_000;

pub(super) fn specs() -> Vec<CommandSpec> {
    vec![
        CommandSpec::new("geodmtbod", "Insert Point into Terrain Model", run_geodmtbod)
            .menu(&["Geo", "Edit Terrain Model", "Insert Point"])
            .alias(&["geotinadd"])
            .params("{at?: [x, y, z] the new point — or handles: POINTs with heights (default: the selection); layer?: (default GEO_DMT)} → {inserted, removed, added, message}")
            .interactive(|_| Ok(Box::new(EditM::new(Edit::Insert)))),
        CommandSpec::new("geodmtodebrat", "Remove Point from Terrain Model", run_geodmtodebrat)
            .menu(&["Geo", "Edit Terrain Model", "Remove Point"])
            .alias(&["geotinremove"])
            .params("{at: [x, y] near the vertex to remove, layer?: (default GEO_DMT)} → {vertex: [x, y, z], removed, added, message}")
            .interactive(|_| Ok(Box::new(EditM::new(Edit::Remove)))),
        CommandSpec::new("geodmtsmazat", "Delete Terrain Triangle", run_geodmtsmazat)
            .menu(&["Geo", "Edit Terrain Model", "Delete Triangle"])
            .alias(&["geotindelete"])
            .params("{at: [x, y] inside the triangle, layer?: (default GEO_DMT)} → {handle, message}")
            .interactive(|_| Ok(Box::new(EditM::new(Edit::Delete)))),
        CommandSpec::new("geodmtvyska", "Change Terrain Vertex Height", run_geodmtvyska)
            .menu(&["Geo", "Edit Terrain Model", "Change Vertex Height"])
            .alias(&["geotinheight"])
            .params("{at: [x, y] near the vertex, z?: new height — or dz: change, layer?: (default GEO_DMT)} → {vertex: [x, y], from, to, faces, message}")
            .interactive(|_| Ok(Box::new(EditM::new(Edit::Height)))),
    ]
}

type Key = (i64, i64);

fn key(v: Vec3) -> Key {
    ((v.x * 1000.0).round() as i64, (v.y * 1000.0).round() as i64)
}

/// The model's faces with their handles.
fn faces(s: &Session, space: &Space, layer: &str) -> Result<Vec<(Handle, [Vec3; 3])>> {
    let d = s.doc()?;
    let Some(st) = d.space(space) else { return Ok(Vec::new()) };
    Ok(st
        .iter()
        .filter(|e| e.common.layer.eq_ignore_ascii_case(layer))
        .filter_map(|e| if let EntityKind::Face3d(f) = &e.kind { Some((e.handle, [f.corners[0], f.corners[1], f.corners[2]])) } else { None })
        .filter(|(_, t)| t.iter().all(|v| v.x.is_finite() && v.y.is_finite() && v.z.is_finite()))
        .collect())
}

fn layer_of(p: &Value, cmd: &str) -> Result<String> {
    match p.get("layer") {
        None | Some(Value::Null) => Ok(LAYER_TIN.to_string()),
        Some(Value::String(l)) if !l.is_empty() && l.chars().count() <= 64 => Ok(l.clone()),
        Some(_) => Err(bad(cmd, "`layer` must be a layer name")),
    }
}

fn pick(p: &Value, cmd: &str) -> Result<Vec2> {
    p.get("at")
        .and_then(point_value)
        .filter(|a| a.x.is_finite() && a.y.is_finite() && a.x.abs() < 1e9 && a.y.abs() < 1e9)
        .ok_or_else(|| bad(cmd, "`at` ([x, y], a point on the model) is required"))
}

/// Twice the signed area of a, b, c relative to `o` (precision far from the origin).
fn orient(o: Vec2, a: Vec3, b: Vec3, c: Vec2) -> f64 {
    let (ax, ay, bx, by, cx, cy) = (a.x - o.x, a.y - o.y, b.x - o.x, b.y - o.y, c.x - o.x, c.y - o.y);
    (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)
}

fn contains(t: &[Vec3; 3], p: Vec2) -> bool {
    let (d0, d1, d2) = (orient(p, t[0], t[1], p), orient(p, t[1], t[2], p), orient(p, t[2], t[0], p));
    let e = 1e-9;
    (d0 >= -e && d1 >= -e && d2 >= -e) || (d0 <= e && d1 <= e && d2 <= e)
}

/// Whether `p` lies strictly inside the circumcircle of `t`.
fn in_circumcircle(t: &[Vec3; 3], p: Vec2) -> bool {
    let (ax, ay) = (t[0].x - p.x, t[0].y - p.y);
    let (bx, by) = (t[1].x - p.x, t[1].y - p.y);
    let (cx, cy) = (t[2].x - p.x, t[2].y - p.y);
    let det = (ax * ax + ay * ay) * (bx * cy - cx * by) - (bx * bx + by * by) * (ax * cy - cx * ay) + (cx * cx + cy * cy) * (ax * by - bx * ay);
    let ccw = orient(p, t[0], t[1], Vec2::new(t[2].x, t[2].y)) > 0.0;
    let scale = (ax * ax + ay * ay).max(bx * bx + by * by).max(cx * cx + cy * cy).powi(2);
    if ccw { det > 1e-12 * scale } else { det < -1e-12 * scale }
}

fn write_faces(s: &mut Session, space: &Space, layer: &str, remove: &[Handle], new: &[[Vec3; 3]], cmd: &str) -> Result<()> {
    let d = s.doc_mut()?;
    for h in remove {
        d.remove_entity(*h);
    }
    for t in new {
        add(s, space, layer, EntityKind::Face3d(Face3d { corners: [t[0], t[1], t[2], t[2]], hidden_edges: 0 }))
            .map_err(|e| bad(cmd, e.to_string()))?;
    }
    Ok(())
}

/// The faces to remove and add to insert `q` into `faces` (Bowyer–Watson), or why not.
pub fn insert_point(faces: &[(Handle, [Vec3; 3])], q: Vec3) -> std::result::Result<(Vec<Handle>, Vec<[Vec3; 3]>), String> {
    let p = q.xy();
    let start =
        faces.iter().position(|(_, t)| contains(t, p)).ok_or("the point lies outside the terrain model: add it to the points and run GEODMT")?;
    if faces.iter().flat_map(|(_, t)| t.iter()).any(|v| key(*v) == key(q)) {
        return Err("the model already has a vertex here: use GEODMTVYSKA to change its height".into());
    }
    // Edge → the faces using it.
    let mut by_edge: HashMap<(Key, Key), Vec<usize>> = HashMap::new();
    for (i, (_, t)) in faces.iter().enumerate() {
        for (a, b) in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
            let (ka, kb) = (key(a), key(b));
            by_edge.entry(if ka < kb { (ka, kb) } else { (kb, ka) }).or_default().push(i);
        }
    }
    // Grow the cavity from the face holding the point.
    let mut cavity: HashSet<usize> = HashSet::from([start]);
    let mut todo = vec![start];
    while let Some(i) = todo.pop() {
        if cavity.len() > MAX_CAVITY {
            return Err("too many triangles would change at once".into());
        }
        let Some((_, t)) = faces.get(i) else { continue };
        for (a, b) in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
            let (ka, kb) = (key(a), key(b));
            for &j in by_edge.get(&if ka < kb { (ka, kb) } else { (kb, ka) }).map(Vec::as_slice).unwrap_or(&[]) {
                if !cavity.contains(&j) && faces.get(j).is_some_and(|(_, u)| in_circumcircle(u, p)) {
                    cavity.insert(j);
                    todo.push(j);
                }
            }
        }
    }
    // Its boundary: edges used by one cavity face only, joined to the new point.
    let mut count: HashMap<(Key, Key), (usize, Vec3, Vec3)> = HashMap::new();
    for &i in &cavity {
        let Some((_, t)) = faces.get(i) else { continue };
        for (a, b) in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
            let (ka, kb) = (key(a), key(b));
            count.entry(if ka < kb { (ka, kb) } else { (kb, ka) }).or_insert((0, a, b)).0 += 1;
        }
    }
    let mut new = Vec::new();
    for (n, a, b) in count.into_values() {
        if n != 1 {
            continue;
        }
        // A degenerate sliver (the point on this edge) is left out.
        if orient(p, a, b, p).abs() > 1e-9 * ((a.x - b.x).hypot(a.y - b.y)).max(1e-9) {
            new.push([a, b, q]);
        }
    }
    if new.len() < 2 {
        return Err("the point could not be fitted into the model".into());
    }
    let mut removed: Vec<Handle> = cavity.into_iter().filter_map(|i| faces.get(i).map(|(h, _)| *h)).collect();
    removed.sort();
    Ok((removed, new))
}

fn run_geodmtbod(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "geodmtbod";
    let space = s.space();
    let layer = layer_of(p, CMD)?;
    let pts: Vec<Vec3> = match p.get("at") {
        Some(Value::Array(a)) => {
            let n: Vec<f64> = a.iter().filter_map(Value::as_f64).filter(|v| v.is_finite() && v.abs() < 1e9).collect();
            match n.as_slice() {
                [x, y, z] => vec![Vec3::new(*x, *y, *z)],
                _ => return Err(bad(CMD, "`at` must be [x, y, z]: the new point needs a height")),
            }
        }
        Some(_) => return Err(bad(CMD, "`at` must be [x, y, z]")),
        None => {
            let hs = super::targets(s, p)?;
            let d = s.doc()?;
            let st = d.space(&space).ok_or_else(|| bad(CMD, "no drawing space"))?;
            hs.iter()
                .filter_map(|h| st.get(*h))
                .filter_map(|e| if let EntityKind::Point(q) = &e.kind { Some(q.p) } else { None })
                .filter(|v| v.x.is_finite() && v.y.is_finite() && v.z.is_finite())
                .collect()
        }
    };
    if pts.is_empty() {
        return Err(bad(CMD, "give `at` [x, y, z], or select POINTs with heights"));
    }
    let (mut removed, mut added, mut inserted) = (0, 0, 0);
    let mut skipped = Vec::new();
    for q in pts.iter().take(10_000) {
        let fs = faces(s, &space, &layer)?;
        if fs.is_empty() {
            return Err(bad(CMD, format!("no terrain model on {layer}: run GEODMT first")));
        }
        match insert_point(&fs, *q) {
            Ok((gone, new)) => {
                write_faces(s, &space, &layer, &gone, &new, CMD)?;
                removed += gone.len();
                added += new.len();
                inserted += 1;
            }
            Err(e) if pts.len() == 1 => return Err(bad(CMD, e)),
            Err(e) => skipped.push(e),
        }
    }
    if inserted == 0 {
        return Err(bad(CMD, skipped.first().cloned().unwrap_or_else(|| "nothing inserted".into())));
    }
    Ok(json!({
        "inserted": inserted, "removed": removed, "added": added, "skipped": skipped.len(),
        "message": format!("{inserted} point(s) inserted into the terrain model ({removed} triangles replaced by {added}){}", if skipped.is_empty() { String::new() } else { format!("; {} skipped (outside the model or already in it)", skipped.len()) }),
    }))
}

/// The model vertex nearest to `p`.
fn nearest_vertex(faces: &[(Handle, [Vec3; 3])], p: Vec2) -> Option<Vec3> {
    faces.iter().flat_map(|(_, t)| t.iter().copied()).min_by(|a, b| (a.x - p.x).hypot(a.y - p.y).total_cmp(&(b.x - p.x).hypot(b.y - p.y)))
}

/// The faces to remove and add to take `v` out of the model.
pub fn remove_vertex(faces: &[(Handle, [Vec3; 3])], v: Vec3) -> std::result::Result<(Vec<Handle>, Vec<[Vec3; 3]>), String> {
    let kv = key(v);
    let star: Vec<&(Handle, [Vec3; 3])> = faces.iter().filter(|(_, t)| t.iter().any(|c| key(*c) == kv)).collect();
    if star.is_empty() {
        return Err("no model vertex there".into());
    }
    if star.len() > MAX_CAVITY {
        return Err("too many triangles would change at once".into());
    }
    // The link: each star face's edge opposite v, oriented counter-clockwise around v.
    let mut next: HashMap<Key, (Key, Vec3)> = HashMap::new();
    let mut verts: HashMap<Key, Vec3> = HashMap::new();
    for (_, t) in &star {
        let Some(i) = t.iter().position(|c| key(*c) == kv) else { continue };
        let (mut a, mut b) = (t[(i + 1) % 3], t[(i + 2) % 3]);
        if orient(v.xy(), v, a, b.xy()) < 0.0 {
            std::mem::swap(&mut a, &mut b);
        }
        verts.insert(key(a), a);
        verts.insert(key(b), b);
        next.insert(key(a), (key(b), b));
    }
    let removed: Vec<Handle> = star.iter().map(|(h, _)| *h).collect();
    // Interior vertex: the link closes into one loop.
    let Some(first) = next.keys().min().copied() else { return Ok((removed, Vec::new())) };
    let mut ring = Vec::new();
    let mut at = first;
    for _ in 0..=next.len() {
        let Some((k, _)) = next.get(&at) else { break };
        ring.push(at);
        at = *k;
        if at == first {
            break;
        }
    }
    let closed = at == first && ring.len() == next.len() && ring.len() >= 3;
    if !closed {
        // On the model's edge: the hole stays open.
        return Ok((removed, Vec::new()));
    }
    // Triangulate the hole about its first vertex for precision, then restore heights.
    let o = verts.get(&first).map(|q| q.xy()).unwrap_or(Vec2::ZERO);
    let poly: Vec<Vec2> = ring.iter().filter_map(|k| verts.get(k)).map(|q| Vec2::new(q.x - o.x, q.y - o.y)).collect();
    let lookup = |q: Vec2| -> Option<Vec3> {
        ring.iter().filter_map(|k| verts.get(k)).find(|w| ((w.x - o.x) - q.x).abs() < 1e-6 && ((w.y - o.y) - q.y).abs() < 1e-6).copied()
    };
    let mut new = Vec::new();
    for t in ear_clip(&poly) {
        let (Some(a), Some(b), Some(c)) = (lookup(t[0]), lookup(t[1]), lookup(t[2])) else { continue };
        new.push([a, b, c]);
    }
    if new.len() + 2 != ring.len() {
        return Err("the hole left by this vertex could not be triangulated".into());
    }
    Ok((removed, new))
}

fn run_geodmtodebrat(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "geodmtodebrat";
    let at = pick(p, CMD)?;
    let space = s.space();
    let layer = layer_of(p, CMD)?;
    let fs = faces(s, &space, &layer)?;
    let v = nearest_vertex(&fs, at).ok_or_else(|| bad(CMD, format!("no terrain model on {layer}: run GEODMT first")))?;
    let (gone, new) = remove_vertex(&fs, v).map_err(|e| bad(CMD, e))?;
    write_faces(s, &space, &layer, &gone, &new, CMD)?;
    Ok(json!({
        "vertex": [v.x, v.y, v.z], "removed": gone.len(), "added": new.len(),
        "message": if new.is_empty() {
            format!("Vertex at height {:.2} removed from the model's edge ({} triangles)", v.z, gone.len())
        } else {
            format!("Vertex at height {:.2} removed: {} triangles replaced by {}", v.z, gone.len(), new.len())
        },
    }))
}

fn run_geodmtsmazat(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "geodmtsmazat";
    let at = pick(p, CMD)?;
    let space = s.space();
    let layer = layer_of(p, CMD)?;
    let fs = faces(s, &space, &layer)?;
    let (h, _) = fs.iter().find(|(_, t)| contains(t, at)).ok_or_else(|| bad(CMD, "no terrain triangle there"))?;
    let h = *h;
    s.doc_mut()?.remove_entity(h);
    Ok(json!({ "handle": h.hex(), "message": "Triangle deleted from the terrain model" }))
}

fn run_geodmtvyska(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "geodmtvyska";
    let at = pick(p, CMD)?;
    let space = s.space();
    let layer = layer_of(p, CMD)?;
    let fs = faces(s, &space, &layer)?;
    let v = nearest_vertex(&fs, at).ok_or_else(|| bad(CMD, format!("no terrain model on {layer}: run GEODMT first")))?;
    let num = |k: &str| {
        p.get(k).map(|x| x.as_f64().filter(|v| v.is_finite() && v.abs() < 1e6).ok_or_else(|| bad(CMD, format!("`{k}` must be a number")))).transpose()
    };
    let to = match (num("z")?, num("dz")?) {
        (Some(z), _) => z,
        (None, Some(dz)) => v.z + dz,
        (None, None) => return Err(bad(CMD, "give the new height `z` or the change `dz`")),
    };
    let kv = key(v);
    let mut n = 0;
    let d = s.doc_mut()?;
    for (h, t) in &fs {
        if !t.iter().any(|c| key(*c) == kv) {
            continue;
        }
        d.modify_entity(*h, |e: &mut cadcraft_doc::Entity| {
            if let EntityKind::Face3d(f) = &mut e.kind {
                for c in f.corners.iter_mut() {
                    if key(*c) == kv {
                        c.z = to;
                    }
                }
            }
        })
        .map_err(|e| bad(CMD, e.to_string()))?;
        n += 1;
    }
    Ok(json!({
        "vertex": [v.x, v.y], "from": v.z, "to": to, "faces": n,
        "message": format!("Vertex height {:.2} → {to:.2} in {n} triangles", v.z),
    }))
}

#[derive(Clone, Copy, PartialEq)]
enum Edit {
    Insert,
    Remove,
    Delete,
    Height,
}

struct EditM {
    edit: Edit,
    at: Option<Vec2>,
}

impl EditM {
    fn new(edit: Edit) -> EditM {
        EditM { edit, at: None }
    }
}

impl Interactive for EditM {
    fn name(&self) -> &'static str {
        match self.edit {
            Edit::Insert => "GEODMTBOD",
            Edit::Remove => "GEODMTODEBRAT",
            Edit::Delete => "GEODMTSMAZAT",
            Edit::Height => "GEODMTVYSKA",
        }
    }
    fn prompt(&self, _s: &Session) -> Prompt {
        match (self.edit, self.at) {
            (Edit::Insert, None) => Prompt::new("New point of the model (Enter when done)", Accept::POINT),
            (Edit::Insert, Some(_)) => Prompt::new("Its height", Accept::NUMBER),
            (Edit::Remove, _) => Prompt::new("Model vertex to remove (Enter when done)", Accept::POINT),
            (Edit::Delete, _) => Prompt::new("Triangle to delete (Enter when done)", Accept::POINT),
            (Edit::Height, None) => Prompt::new("Model vertex to change (Enter when done)", Accept::POINT),
            (Edit::Height, Some(_)) => Prompt::new("New height", Accept::NUMBER),
        }
    }
    fn input(&mut self, s: &mut Session, i: Input) -> Result<Step> {
        match (self.at, i) {
            (_, Input::Cancel) => Ok(Step::Cancel),
            (None, Input::Enter) => Ok(Step::Done),
            (Some(_), Input::Enter) => {
                self.at = None;
                Ok(Step::Continue)
            }
            (None, Input::Point(q)) => {
                match self.edit {
                    Edit::Remove => {
                        let r = run_geodmtodebrat(s, &json!({ "at": [q.x, q.y] }));
                        say(s, r);
                    }
                    Edit::Delete => {
                        let r = run_geodmtsmazat(s, &json!({ "at": [q.x, q.y] }));
                        say(s, r);
                    }
                    Edit::Insert | Edit::Height => self.at = Some(q),
                }
                Ok(Step::Continue)
            }
            (Some(q), Input::Text(t) | Input::Keyword(t)) => {
                let Some(z) = typed_number(&t) else {
                    s.echo("Enter the height in metres.");
                    return Ok(Step::Continue);
                };
                let r = if self.edit == Edit::Insert {
                    run_geodmtbod(s, &json!({ "at": [q.x, q.y, z] }))
                } else {
                    run_geodmtvyska(s, &json!({ "at": [q.x, q.y], "z": z }))
                };
                say(s, r);
                self.at = None;
                Ok(Step::Continue)
            }
            _ => Ok(Step::Continue),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cmd::geo_calc::tin;

    /// A 4 × 4 grid of 10 m cells at height z = x / 10.
    fn model(s: &mut Session) {
        let space = s.space();
        let v = |i: usize, j: usize| Vec3::new(i as f64 * 10.0, j as f64 * 10.0, i as f64);
        for i in 0..4 {
            for j in 0..4 {
                let (a, b, c, d) = (v(i, j), v(i + 1, j), v(i + 1, j + 1), v(i, j + 1));
                for t in [[a, b, c], [a, c, d]] {
                    add(s, &space, LAYER_TIN, EntityKind::Face3d(Face3d { corners: [t[0], t[1], t[2], t[2]], hidden_edges: 0 })).unwrap();
                }
            }
        }
    }

    fn area(tris: &[[Vec3; 3]]) -> f64 {
        tris.iter().map(|t| ((t[1].x - t[0].x) * (t[2].y - t[0].y) - (t[1].y - t[0].y) * (t[2].x - t[0].x)).abs() / 2.0).sum()
    }

    #[test]
    fn insert_and_remove_keep_the_model_whole() {
        let mut s = Session::new();
        model(&mut s);
        let space = s.space();
        let r = s.execute("geodmtbod", &json!({ "at": [13, 17, 50] })).unwrap();
        assert_eq!(r["inserted"], 1, "{r}");
        let t = tin(&s, &space).unwrap();
        // Still covers 40 × 40 m exactly, and the point is a vertex at its height.
        assert!((area(&t) - 1600.0).abs() < 1e-6, "{}", area(&t));
        assert!(t.iter().flatten().any(|v| (v.x - 13.0).abs() < 1e-9 && (v.y - 17.0).abs() < 1e-9 && (v.z - 50.0).abs() < 1e-9));
        let n = t.len();
        // Taking it out again leaves a whole model with as many triangles as before.
        let r = s.execute("geodmtodebrat", &json!({ "at": [13.2, 16.9] })).unwrap();
        assert!((r["vertex"][2].as_f64().unwrap() - 50.0).abs() < 1e-9, "{r}");
        let t = tin(&s, &space).unwrap();
        assert!((area(&t) - 1600.0).abs() < 1e-6 && t.len() == n - 2, "{} {}", area(&t), t.len());
        assert!(t.iter().flatten().all(|v| (v.z - v.x / 10.0).abs() < 1e-9));
        // Twice in the same place is refused; outside the model too.
        s.execute("geodmtbod", &json!({ "at": [25, 25, 9] })).unwrap();
        assert!(s.execute("geodmtbod", &json!({ "at": [25, 25, 9] })).is_err());
        assert!(s.execute("geodmtbod", &json!({ "at": [100, 25, 9] })).is_err());
        assert!(s.execute("geodmtbod", &json!({ "at": [5, 5] })).is_err());
    }

    #[test]
    fn edge_vertex_triangle_and_height() {
        let mut s = Session::new();
        model(&mut s);
        let space = s.space();
        // A corner vertex on the edge: both triangles of its cell go, the model shrinks.
        let r = s.execute("geodmtodebrat", &json!({ "at": [40, 40] })).unwrap();
        assert_eq!((r["removed"].as_u64(), r["added"].as_u64()), (Some(2), Some(0)), "{r}");
        assert!((area(&tin(&s, &space).unwrap()) - 1500.0).abs() < 1e-6);
        s.execute("geodmtsmazat", &json!({ "at": [1, 5] })).unwrap();
        assert!((area(&tin(&s, &space).unwrap()) - 1450.0).abs() < 1e-6);
        assert!(s.execute("geodmtsmazat", &json!({ "at": [1, 5] })).is_err());
        // The vertex (20, 20) is in six triangles.
        let r = s.execute("geodmtvyska", &json!({ "at": [19.6, 20.3], "z": 7.5 })).unwrap();
        assert_eq!(r["faces"], 6, "{r}");
        let r = s.execute("geodmtvyska", &json!({ "at": [20, 20], "dz": -0.5 })).unwrap();
        assert!((r["to"].as_f64().unwrap() - 7.0).abs() < 1e-9);
        assert!(tin(&s, &space).unwrap().iter().flatten().filter(|v| v.x == 20.0 && v.y == 20.0).all(|v| (v.z - 7.0).abs() < 1e-9));
    }

    #[test]
    fn hostile_input_is_refused_not_panicking() {
        let mut s = Session::new();
        for (c, p) in [
            ("geodmtbod", json!({})),
            ("geodmtbod", json!({ "at": [1, 1, 1] })),
            ("geodmtodebrat", json!({})),
            ("geodmtodebrat", json!({ "at": [0, 0] })),
            ("geodmtsmazat", json!({ "at": [0, 0] })),
            ("geodmtvyska", json!({ "at": [0, 0], "z": 1 })),
        ] {
            assert!(s.execute(c, &p).is_err(), "{c} {p}");
        }
        model(&mut s);
        for (c, p) in [
            ("geodmtbod", json!({ "at": [1, 1, "x"] })),
            ("geodmtvyska", json!({ "at": [0, 0] })),
            ("geodmtvyska", json!({ "at": [0, 0], "z": "x" })),
            ("geodmtbod", json!({ "layer": 3, "at": [1, 1, 1] })),
        ] {
            assert!(s.execute(c, &p).is_err(), "{c} {p}");
        }
        let _ = insert_point(&[], Vec3::new(f64::NAN, 0.0, 0.0));
        let _ = remove_vertex(&[], Vec3::ZERO);
    }
}
