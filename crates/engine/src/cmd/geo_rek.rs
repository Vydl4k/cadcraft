//! Survey add-on: measured building plans (GEOZED, GEOOTVOR).
//!
//! GEOZED turns a measured wall face (a line or polyline, usually drawn through surveyed
//! corners) into a wall of a given thickness: both faces as polylines on `GEO_zdi` (mitred at
//! the corners, capped at the ends of an open wall) and a hatch on `GEO_zdi_srafy` with one
//! loop per straight piece, so an opening can later cut a single piece.
//!
//! GEOOTVOR puts an opening in such a wall: from a point on one face it finds the opposite face,
//! breaks both faces over the opening's width, closes the cut with jambs, cuts the hatch, and
//! draws a window (glass lines) or a door (leaf and swing) on `GEO_otvory`.

use cadcraft_doc::{Arc, Common, EntityKind, Handle, Hatch, HatchLoop, Line, LwPolyline, Space};
use cadcraft_geom::{PolyVertex, Vec2, Vec3};
use serde_json::{Value, json};

use super::{CommandSpec, bad, f64_or, point_value, str_param};
use crate::{Accept, EngineError, Input, Interactive, Prompt, Result, Session, Step};

pub const LAYER_WALLS: &str = "GEO_zdi";
pub const LAYER_WALL_HATCH: &str = "GEO_zdi_srafy";
pub const LAYER_OPENINGS: &str = "GEO_otvory";

/// Vertices of one wall.
const MAX_VERTS: usize = 10_000;
/// Thickest wall accepted, and the farthest the opposite face is looked for.
const MAX_THICKNESS: f64 = 3.0;

pub(super) fn specs() -> Vec<CommandSpec> {
    vec![
        CommandSpec::new("geozed", "Wall from Face...", run_geozed)
            .menu(&["Geo", "Building", "Wall from Face..."])
            .alias(&["geowall"])
            .params("{path?: handle of the face line/polyline (default: the selected one) | points?: [[x,y]...], closed?: bool, thickness?: metres (default 0.30), toward: [x,y] a point on the wall's side | side?: \"left\"|\"right\", scale?: drawing scale 1:N for the hatch (default 50), keep?: bool, keep the face object (default false)} → {faces, pieces}")
            .interactive(|_| Ok(Box::new(ZedM::default()))),
        CommandSpec::new("geootvor", "Window or Door in Wall...", run_geootvor)
            .menu(&["Geo", "Building", "Window or Door..."])
            .alias(&["geoopening", "geookno", "geodvere"])
            .params("{at: [x,y] on a wall face, width?: metres (default 1.2 window, 0.8 door), kind?: \"okno\"|\"dvere\" (window|door), hinge?: \"start\"|\"end\" (door), toward?: [x,y] the side the door opens to} → {message}")
            .interactive(|_| Ok(Box::new(OtvorM::default()))),
    ]
}

fn sub(a: Vec2, b: Vec2) -> Vec2 {
    Vec2::new(a.x - b.x, a.y - b.y)
}
fn addv(a: Vec2, b: Vec2) -> Vec2 {
    Vec2::new(a.x + b.x, a.y + b.y)
}
fn mul(a: Vec2, k: f64) -> Vec2 {
    Vec2::new(a.x * k, a.y * k)
}
fn dot(a: Vec2, b: Vec2) -> f64 {
    a.x * b.x + a.y * b.y
}
fn crs(a: Vec2, b: Vec2) -> f64 {
    a.x * b.y - a.y * b.x
}
fn len(a: Vec2) -> f64 {
    a.x.hypot(a.y)
}
fn unit(a: Vec2) -> Option<Vec2> {
    let l = len(a);
    (l > 1e-12).then(|| mul(a, 1.0 / l))
}
/// The left normal of a direction.
fn left(d: Vec2) -> Vec2 {
    Vec2::new(-d.y, d.x)
}

/// The polyline offset by `d` to its left (negative: right), mitred at the corners (a corner
/// sharper than ~15° is bevelled).
pub fn offset(pts: &[Vec2], closed: bool, d: f64) -> Option<Vec<Vec2>> {
    let n = pts.len();
    if n < 2 {
        return None;
    }
    let seg = |i: usize| -> Option<(Vec2, Vec2)> {
        let a = *pts.get(i % n)?;
        let b = *pts.get((i + 1) % n)?;
        let u = unit(sub(b, a))?;
        let o = mul(left(u), d);
        Some((addv(a, o), u))
    };
    let segs = if closed { n } else { n - 1 };
    let mut out = Vec::with_capacity(n);
    for i in 0..n {
        let (prev, next) = (if i == 0 { closed.then(|| segs - 1) } else { Some(i - 1) }, (i < segs).then_some(i));
        let p = *pts.get(i)?;
        let q = match (prev.and_then(seg), next.and_then(seg)) {
            (Some((pa, pu)), Some((na, nu))) => {
                let den = crs(pu, nu);
                if den.abs() < 1e-9 || dot(pu, nu) < -0.97 {
                    // Straight on (or folding back): the plain offset of the vertex.
                    addv(p, mul(left(nu), d))
                } else {
                    let t = crs(sub(na, pa), nu) / den;
                    addv(pa, mul(pu, t))
                }
            }
            (Some((_, pu)), None) => addv(p, mul(left(pu), d)),
            (None, Some((_, nu))) => addv(p, mul(left(nu), d)),
            (None, None) => return None,
        };
        out.push(q);
    }
    Some(out)
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
    s.doc_mut()?.add(space, common, kind).map_err(|e| bad("geozed", e.to_string()))
}

fn pline(v: &[Vec2], closed: bool) -> EntityKind {
    EntityKind::LwPolyline(LwPolyline {
        vertices: v.iter().copied().map(PolyVertex::new).collect(),
        closed,
        const_width: 0.0,
        elevation: 0.0,
        plinegen: false,
    })
}

fn line(a: Vec2, b: Vec2) -> EntityKind {
    EntityKind::Line(Line { a: Vec3::new(a.x, a.y, 0.0), b: Vec3::new(b.x, b.y, 0.0) })
}

fn hatch(loops: Vec<Vec<Vec2>>, scale: f64) -> EntityKind {
    EntityKind::Hatch(Hatch {
        pattern: "ANSI31".into(),
        solid: false,
        loops: loops
            .into_iter()
            .map(|l| HatchLoop { vertices: l.into_iter().map(PolyVertex::new).collect(), outer: true, sources: Vec::new() })
            .collect(),
        // ANSI31's lines are 3.175 units apart: 1.5 mm on paper at 1:scale, in metres.
        scale: 0.0015 * scale / 3.175,
        angle: 0.0,
        associative: false,
        style: 0,
        elevation: 0.0,
        gradient: None,
        origin: Vec2::ZERO,
        background: None,
        pattern_lines: Vec::new(),
    })
}

// --- GEOZED --------------------------------------------------------------------------------------

fn run_geozed(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "geozed";
    let space = s.space();
    let (mut pts, closed, source) = if let Some(a) = p.get("points").and_then(Value::as_array) {
        let v: Option<Vec<Vec2>> = a.iter().map(point_value).collect();
        (v.ok_or_else(|| bad(CMD, "`points` must be [[x, y], ...]"))?, p.get("closed").and_then(Value::as_bool).unwrap_or(false), None)
    } else {
        let h = match p.get("path") {
            Some(v) => v.as_str().and_then(Handle::parse_hex).ok_or_else(|| bad(CMD, "`path` must be an object handle"))?,
            None => match s.selection().as_slice() {
                [h] => *h,
                _ => return Err(bad(CMD, "select one wall face line or polyline (or pass `path`)")),
            },
        };
        let d = s.doc()?;
        let e = d.space(&space).and_then(|st| st.get(h)).ok_or_else(|| bad(CMD, "the face object doesn't exist"))?;
        let (v, c) = match &e.kind {
            EntityKind::LwPolyline(pl) => {
                if pl.vertices.iter().any(|v| v.bulge.abs() > 1e-9) {
                    return Err(bad(CMD, "curved walls aren't supported yet: the face has arcs"));
                }
                (pl.vertices.iter().map(|v| v.p).collect(), pl.closed)
            }
            EntityKind::Line(l) => (vec![l.a.xy(), l.b.xy()], false),
            _ => return Err(bad(CMD, "the face must be a line or polyline")),
        };
        (v, c, Some(h))
    };
    pts.dedup_by(|a, b| len(sub(*a, *b)) < 1e-9);
    if pts.len() < 2 || pts.len() > MAX_VERTS || pts.iter().any(|v| !v.x.is_finite() || !v.y.is_finite()) {
        return Err(bad(CMD, format!("a wall face needs 2 to {MAX_VERTS} points")));
    }
    let closed = closed && pts.len() >= 3;
    let thickness = f64_or(p, "thickness", 0.30);
    if !(thickness.is_finite() && (0.01..=MAX_THICKNESS).contains(&thickness)) {
        return Err(bad(CMD, format!("`thickness` must be between 0.01 and {MAX_THICKNESS} m")));
    }
    let scale = f64_or(p, "scale", 50.0);
    if !(scale.is_finite() && (1.0..=100_000.0).contains(&scale)) {
        return Err(bad(CMD, "`scale` must be a drawing scale like 50 or 100"));
    }
    // Which side the wall's body is on: left of the first segment, unless told otherwise.
    let sign = if let Some(t) = p.get("toward").and_then(point_value) {
        // The side of the nearest segment the point is on.
        let mut best = (f64::INFINITY, 1.0);
        let segs = if closed { pts.len() } else { pts.len() - 1 };
        for i in 0..segs {
            let (Some(&a), Some(&b)) = (pts.get(i), pts.get((i + 1) % pts.len())) else { continue };
            let ab = sub(b, a);
            let l2 = dot(ab, ab);
            if l2 < 1e-18 {
                continue;
            }
            let tt = (dot(sub(t, a), ab) / l2).clamp(0.0, 1.0);
            let dist = len(sub(t, addv(a, mul(ab, tt))));
            if dist < best.0 {
                best = (dist, if crs(ab, sub(t, a)) >= 0.0 { 1.0 } else { -1.0 });
            }
        }
        best.1
    } else {
        match str_param(p, "side").map(str::to_ascii_lowercase).as_deref() {
            None | Some("left" | "l" | "vlevo") => 1.0,
            Some("right" | "r" | "vpravo") => -1.0,
            Some(other) => return Err(bad(CMD, format!("`side` \"{other}\": use left or right, or give `toward`"))),
        }
    };
    let other = offset(&pts, closed, sign * thickness).ok_or_else(|| bad(CMD, "the face could not be offset"))?;
    ensure_layer(s, LAYER_WALLS, 7)?;
    ensure_layer(s, LAYER_WALL_HATCH, 8)?;
    {
        let d = s.doc_mut()?;
        if let Some(l) = d.layer_mut(LAYER_WALLS) {
            l.lineweight = cadcraft_doc::Lineweight::from_dxf(50);
        }
    }
    add(s, &space, LAYER_WALLS, pline(&pts, closed))?;
    add(s, &space, LAYER_WALLS, pline(&other, closed))?;
    if !closed {
        if let (Some(a), Some(b)) = (pts.first(), other.first()) {
            add(s, &space, LAYER_WALLS, line(*a, *b))?;
        }
        if let (Some(a), Some(b)) = (pts.last(), other.last()) {
            add(s, &space, LAYER_WALLS, line(*a, *b))?;
        }
    }
    let n = pts.len();
    let segs = if closed { n } else { n - 1 };
    let loops: Vec<Vec<Vec2>> = (0..segs)
        .filter_map(|i| {
            let j = (i + 1) % n;
            Some(vec![*pts.get(i)?, *pts.get(j)?, *other.get(j)?, *other.get(i)?])
        })
        .collect();
    add(s, &space, LAYER_WALL_HATCH, hatch(loops, scale))?;
    if let Some(h) = source
        && !p.get("keep").and_then(Value::as_bool).unwrap_or(false)
    {
        s.doc_mut()?.remove_entity(h);
    }
    Ok(json!({ "faces": 2, "pieces": segs, "message": format!("Wall {:.0} mm thick, {segs} straight piece(s)", thickness * 1000.0) }))
}

#[derive(Default)]
struct ZedM {
    face: Option<Handle>,
    thickness: Option<f64>,
}

fn typed_number(t: &str) -> Option<f64> {
    t.trim().replace(',', ".").parse::<f64>().ok().filter(|v| v.is_finite())
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

impl Interactive for ZedM {
    fn name(&self) -> &'static str {
        "GEOZED"
    }
    fn prompt(&self, _s: &Session) -> Prompt {
        match (self.face, self.thickness) {
            (None, _) => Prompt::new("Select the measured wall face (line or polyline)", Accept::SELECT),
            (Some(_), None) => Prompt::new("Wall thickness in metres", Accept::NUMBER).default("0.30"),
            (Some(_), Some(_)) => Prompt::new("Pick a point on the wall's side", Accept::POINT),
        }
    }
    fn input(&mut self, s: &mut Session, i: Input) -> Result<Step> {
        match (self.face, self.thickness, i) {
            (_, _, Input::Cancel) => Ok(Step::Cancel),
            (None, _, Input::Pick(hs)) => {
                self.face = hs.first().copied();
                Ok(Step::Continue)
            }
            (Some(_), None, Input::Enter) => {
                self.thickness = Some(0.30);
                Ok(Step::Continue)
            }
            (Some(_), None, Input::Text(t) | Input::Keyword(t)) => {
                match typed_number(&t) {
                    Some(v) if v > 0.0 => self.thickness = Some(v),
                    _ => s.echo("Enter the thickness in metres, like 0.45."),
                }
                Ok(Step::Continue)
            }
            (Some(face), Some(th), Input::Point(t)) => {
                let r = run_geozed(s, &json!({ "path": face.hex(), "thickness": th, "toward": [t.x, t.y] }));
                say(s, r);
                Ok(Step::Done)
            }
            _ => Ok(Step::Continue),
        }
    }
}

// --- GEOOTVOR ------------------------------------------------------------------------------------

/// A straight piece of a wall face polyline.
#[derive(Clone, Copy, Debug)]
struct Seg {
    handle: Handle,
    index: usize,
    a: Vec2,
    b: Vec2,
}

fn face_segments(s: &Session, space: &Space) -> Result<Vec<Seg>> {
    let d = s.doc()?;
    let Some(st) = d.space(space) else { return Ok(Vec::new()) };
    let mut out = Vec::new();
    for e in st.iter().filter(|e| e.common.layer.eq_ignore_ascii_case(LAYER_WALLS)) {
        if let EntityKind::LwPolyline(pl) = &e.kind {
            let n = pl.vertices.len();
            let segs = if pl.closed { n } else { n.saturating_sub(1) };
            for i in 0..segs {
                let (Some(a), Some(b)) = (pl.vertices.get(i), pl.vertices.get((i + 1) % n)) else { continue };
                out.push(Seg { handle: e.handle, index: i, a: a.p, b: b.p });
            }
        }
    }
    Ok(out)
}

/// Split a polyline's vertex list at two points on segment `i` (`p1` nearer the segment's start),
/// leaving out the stretch between them: one open polyline for a closed one, up to two for an
/// open one.
fn split(v: &[Vec2], closed: bool, i: usize, p1: Vec2, p2: Vec2) -> Vec<Vec<Vec2>> {
    let n = v.len();
    if closed {
        // From p2 round to p1.
        let mut out = vec![p2];
        for k in 1..=n {
            if let Some(q) = v.get((i + k) % n) {
                out.push(*q);
            }
        }
        out.push(p1);
        out.dedup_by(|a, b| len(sub(*a, *b)) < 1e-9);
        vec![out]
    } else {
        let mut first: Vec<Vec2> = v.get(..=i).map(<[Vec2]>::to_vec).unwrap_or_default();
        first.push(p1);
        let mut second = vec![p2];
        second.extend(v.get(i + 1..).unwrap_or_default());
        [first, second]
            .into_iter()
            .map(|mut p| {
                p.dedup_by(|a, b| len(sub(*a, *b)) < 1e-9);
                p
            })
            .filter(|p| p.len() >= 2)
            .collect()
    }
}

fn cut_face(s: &mut Session, space: &Space, sg: Seg, p1: Vec2, p2: Vec2) -> Result<()> {
    let (v, closed, common) = {
        let d = s.doc()?;
        let e = d.space(space).and_then(|st| st.get(sg.handle)).ok_or_else(|| bad("geootvor", "the wall face disappeared"))?;
        let EntityKind::LwPolyline(pl) = &e.kind else { return Err(bad("geootvor", "the wall face is not a polyline")) };
        (pl.vertices.iter().map(|v| v.p).collect::<Vec<_>>(), pl.closed, e.common.clone())
    };
    // p1 before p2 along the segment.
    let u = sub(sg.b, sg.a);
    let (p1, p2) = if dot(sub(p1, sg.a), u) <= dot(sub(p2, sg.a), u) { (p1, p2) } else { (p2, p1) };
    let pieces = split(&v, closed, sg.index, p1, p2);
    let d = s.doc_mut()?;
    d.remove_entity(sg.handle);
    for pc in pieces {
        d.add(space, common.clone(), pline(&pc, false)).map_err(|e| bad("geootvor", e.to_string()))?;
    }
    Ok(())
}

/// Clip a polygon to the half-plane `f >= 0`.
fn clip_half(poly: &[Vec2], f: impl Fn(Vec2) -> f64) -> Vec<Vec2> {
    let n = poly.len();
    let mut out = Vec::with_capacity(n + 2);
    for i in 0..n {
        let (Some(&a), Some(&b)) = (poly.get(i), poly.get((i + 1) % n)) else { continue };
        let (fa, fb) = (f(a), f(b));
        if fa >= 0.0 {
            out.push(a);
        }
        if (fa >= 0.0) != (fb >= 0.0) {
            let t = fa / (fa - fb);
            out.push(addv(a, mul(sub(b, a), t)));
        }
    }
    out
}

fn inside(p: Vec2, poly: &[Vec2]) -> bool {
    let n = poly.len();
    let mut c = false;
    for i in 0..n {
        let (Some(a), Some(b)) = (poly.get(i), poly.get((i + n - 1) % n)) else { continue };
        if (a.y > p.y) != (b.y > p.y) && p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x {
            c = !c;
        }
    }
    c
}

fn area(p: &[Vec2]) -> f64 {
    let n = p.len();
    ((0..n).map(|i| p.get(i).zip(p.get((i + 1) % n)).map_or(0.0, |(a, b)| crs(*a, *b))).sum::<f64>() / 2.0).abs()
}

/// Remove the opening (between the lines through `o1` and `o2` across direction `u`) from the
/// wall hatch loop that holds `mid`.
fn cut_hatch(s: &mut Session, space: &Space, mid: Vec2, u: Vec2, o1: f64, o2: f64) -> Result<bool> {
    let found = {
        let d = s.doc()?;
        let Some(st) = d.space(space) else { return Ok(false) };
        st.iter().filter(|e| e.common.layer.eq_ignore_ascii_case(LAYER_WALL_HATCH)).find_map(|e| match &e.kind {
            EntityKind::Hatch(h) => {
                h.loops.iter().position(|l| inside(mid, &l.vertices.iter().map(|v| v.p).collect::<Vec<_>>())).map(|i| (e.handle, i))
            }
            _ => None,
        })
    };
    let Some((h, li)) = found else { return Ok(false) };
    let o = mid;
    s.doc_mut()?
        .modify_entity(h, |e| {
            if let EntityKind::Hatch(ht) = &mut e.kind
                && let Some(l) = ht.loops.get(li).cloned()
            {
                let poly: Vec<Vec2> = l.vertices.iter().map(|v| v.p).collect();
                let before = clip_half(&poly, |q| o1 - dot(sub(q, o), u));
                let after = clip_half(&poly, |q| dot(sub(q, o), u) - o2);
                ht.loops.remove(li);
                for part in [before, after] {
                    if part.len() >= 3 && area(&part) > 1e-8 {
                        ht.loops.push(HatchLoop { vertices: part.into_iter().map(PolyVertex::new).collect(), outer: true, sources: Vec::new() });
                    }
                }
            }
        })
        .map_err(|e| bad("geootvor", e.to_string()))?;
    Ok(true)
}

fn run_geootvor(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "geootvor";
    let at = p.get("at").and_then(point_value).ok_or_else(|| bad(CMD, "`at` ([x, y], a point on a wall face) is required"))?;
    let door = match str_param(p, "kind").map(str::to_ascii_lowercase).as_deref() {
        None | Some("okno" | "window" | "o" | "w") => false,
        Some("dvere" | "dveře" | "door" | "d") => true,
        Some(other) => return Err(bad(CMD, format!("`kind` \"{other}\": use okno (window) or dvere (door)"))),
    };
    let width = f64_or(p, "width", if door { 0.8 } else { 1.2 });
    if !(width.is_finite() && (0.05..=30.0).contains(&width)) {
        return Err(bad(CMD, "`width` must be between 0.05 and 30 m"));
    }
    let space = s.space();
    let segs = face_segments(s, &space)?;
    // The face picked: the nearest wall face segment.
    let dist = |sg: &Seg, q: Vec2| {
        let ab = sub(sg.b, sg.a);
        let l2 = dot(ab, ab).max(1e-18);
        let t = (dot(sub(q, sg.a), ab) / l2).clamp(0.0, 1.0);
        (len(sub(q, addv(sg.a, mul(ab, t)))), t)
    };
    let (sa, ta) = segs
        .iter()
        .map(|sg| (*sg, dist(sg, at)))
        .min_by(|x, y| x.1.0.total_cmp(&y.1.0))
        .map(|(sg, (_, t))| (sg, t))
        .ok_or_else(|| bad(CMD, "no walls: make them with GEOZED first"))?;
    if dist(&sa, at).0 > MAX_THICKNESS {
        return Err(bad(CMD, "pick a point on a wall face"));
    }
    let seg_len = len(sub(sa.b, sa.a));
    let u = unit(sub(sa.b, sa.a)).ok_or_else(|| bad(CMD, "the wall face has a zero-length piece"))?;
    let centre = ta * seg_len;
    let (s1, s2) = (centre - width / 2.0, centre + width / 2.0);
    if s1 < -1e-9 || s2 > seg_len + 1e-9 {
        return Err(bad(CMD, format!("the opening doesn't fit on this straight piece of wall ({seg_len:.2} m)")));
    }
    let a1 = addv(sa.a, mul(u, s1));
    let a2 = addv(sa.a, mul(u, s2));
    // The opposite face: a parallel piece across the wall, covering the opening.
    let mid = addv(sa.a, mul(u, centre));
    let sb = segs
        .iter()
        .filter(|sg| !(sg.handle == sa.handle && sg.index == sa.index))
        .filter_map(|sg| {
            let v = unit(sub(sg.b, sg.a))?;
            if crs(u, v).abs() > 0.02 {
                return None;
            }
            let off = crs(u, sub(sg.a, mid)).abs();
            if !(0.005..=MAX_THICKNESS).contains(&off) {
                return None;
            }
            // Both ends of the opening fall on it.
            let along = |q: Vec2| dot(sub(q, sg.a), v);
            let l = len(sub(sg.b, sg.a));
            let fits = [a1, a2].iter().all(|q| (-1e-6..=l + 1e-6).contains(&along(*q)));
            fits.then_some((*sg, off))
        })
        .min_by(|x, y| x.1.total_cmp(&y.1))
        .map(|(sg, _)| sg)
        .ok_or_else(|| bad(CMD, "no opposite wall face found across this one"))?;
    let n = left(u);
    let thick = crs(u, sub(sb.a, mid));
    let b1 = addv(a1, mul(n, thick));
    let b2 = addv(a2, mul(n, thick));
    cut_face(s, &space, sa, a1, a2)?;
    cut_face(s, &space, sb, b1, b2)?;
    ensure_layer(s, LAYER_OPENINGS, 4)?;
    add(s, &space, LAYER_WALLS, line(a1, b1))?;
    add(s, &space, LAYER_WALLS, line(a2, b2))?;
    let centre_across = addv(mid, mul(n, thick / 2.0));
    cut_hatch(s, &space, centre_across, u, dot(sub(a1, centre_across), u), dot(sub(a2, centre_across), u))?;
    if door {
        // Hinged at the start or end jamb, on the face toward the side it opens to.
        let toward = p.get("toward").and_then(point_value).unwrap_or_else(|| addv(mid, mul(n, -1.0)));
        let on_b = crs(u, sub(toward, mid)).signum() == thick.signum() && crs(u, sub(toward, mid)).abs() > thick.abs() / 2.0;
        let (h1, h2) = if on_b { (b1, b2) } else { (a1, a2) };
        let at_end = matches!(str_param(p, "hinge").map(str::to_ascii_lowercase).as_deref(), Some("end" | "konec" | "e"));
        let (hinge, free) = if at_end { (h2, h1) } else { (h1, h2) };
        let out = unit(sub(toward, mid)).map_or(n, |t| if dot(t, n) >= 0.0 { n } else { mul(n, -1.0) });
        let leaf_end = addv(hinge, mul(out, width));
        add(s, &space, LAYER_OPENINGS, line(hinge, leaf_end))?;
        let ang = |q: Vec2| (q.y - hinge.y).atan2(q.x - hinge.x);
        let (a0, a3) = (ang(leaf_end), ang(free));
        // The quarter turn between the open leaf and the closed position.
        let mut sweep = a3 - a0;
        while sweep > std::f64::consts::PI {
            sweep -= std::f64::consts::TAU;
        }
        while sweep < -std::f64::consts::PI {
            sweep += std::f64::consts::TAU;
        }
        let (start, end) = if sweep >= 0.0 { (a0, a3) } else { (a3, a0) };
        add(s, &space, LAYER_OPENINGS, EntityKind::Arc(Arc { center: Vec3::new(hinge.x, hinge.y, 0.0), radius: width, start, end }))?;
    } else {
        // Glass: two lines at a third and two thirds of the way across.
        for f in [1.0 / 3.0, 2.0 / 3.0] {
            add(s, &space, LAYER_OPENINGS, line(addv(a1, mul(n, thick * f)), addv(a2, mul(n, thick * f))))?;
        }
    }
    Ok(json!({ "message": format!("{} {:.0} mm in a {:.0} mm wall", if door { "Door" } else { "Window" }, width * 1000.0, thick.abs() * 1000.0) }))
}

#[derive(Default)]
struct OtvorM {
    at: Option<Vec2>,
    door: Option<bool>,
    width: Option<f64>,
}

impl Interactive for OtvorM {
    fn name(&self) -> &'static str {
        "GEOOTVOR"
    }
    fn prompt(&self, _s: &Session) -> Prompt {
        match (self.at, self.door, self.width) {
            (None, _, _) => Prompt::new("Pick the middle of the opening on a wall face", Accept::POINT),
            (Some(_), None, _) => Prompt::new("Window or door", Accept::TEXT).kw(&["Okno", "Dvere"]).default("Okno"),
            (Some(_), Some(d), None) => Prompt::new("Width in metres", Accept::NUMBER).default(if d { "0.80" } else { "1.20" }),
            (Some(_), Some(_), Some(_)) => Prompt::new("Pick the side the door opens to", Accept::POINT),
        }
    }
    fn input(&mut self, s: &mut Session, i: Input) -> Result<Step> {
        if matches!(i, Input::Cancel) {
            return Ok(Step::Cancel);
        }
        match (self.at, self.door, self.width) {
            (None, _, _) => {
                if let Input::Point(q) = i {
                    self.at = Some(q);
                }
                Ok(Step::Continue)
            }
            (Some(_), None, _) => {
                self.door = Some(match &i {
                    Input::Keyword(t) | Input::Text(t) => t.trim().to_ascii_lowercase().starts_with('d'),
                    _ => false,
                });
                Ok(Step::Continue)
            }
            (Some(at), Some(door), None) => {
                let w = match &i {
                    Input::Enter => Some(if door { 0.8 } else { 1.2 }),
                    Input::Text(t) | Input::Keyword(t) => typed_number(t).filter(|v| *v > 0.0),
                    _ => return Ok(Step::Continue),
                };
                let Some(w) = w else {
                    s.echo("Enter the width in metres, like 0.9.");
                    return Ok(Step::Continue);
                };
                if door {
                    self.width = Some(w);
                    return Ok(Step::Continue);
                }
                let r = run_geootvor(s, &json!({ "at": [at.x, at.y], "kind": "okno", "width": w }));
                say(s, r);
                Ok(Step::Done)
            }
            (Some(at), Some(_), Some(w)) => {
                let Input::Point(t) = i else { return Ok(Step::Continue) };
                let r = run_geootvor(s, &json!({ "at": [at.x, at.y], "kind": "dvere", "width": w, "toward": [t.x, t.y] }));
                say(s, r);
                Ok(Step::Done)
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ents(s: &Session, layer: &str) -> Vec<EntityKind> {
        s.doc().unwrap().model.iter().filter(|e| e.common.layer == layer).map(|e| e.kind.clone()).collect()
    }

    #[test]
    fn offsets_mitre_corners() {
        let sq = [Vec2::new(0.0, 0.0), Vec2::new(4.0, 0.0), Vec2::new(4.0, 3.0), Vec2::new(0.0, 3.0)];
        // Counter-clockwise square, offset outward (right) by 0.3.
        let o = offset(&sq, true, -0.3).unwrap();
        assert!((o[0].x + 0.3).abs() < 1e-12 && (o[0].y + 0.3).abs() < 1e-12, "{o:?}");
        assert!((o[2].x - 4.3).abs() < 1e-12 && (o[2].y - 3.3).abs() < 1e-12, "{o:?}");
        let o = offset(&sq[..3], false, 0.3).unwrap();
        assert!(
            (o[0].y - 0.3).abs() < 1e-12 && (o[1].x - 3.7).abs() < 1e-12 && (o[1].y - 0.3).abs() < 1e-12 && (o[2].x - 3.7).abs() < 1e-12,
            "{o:?}"
        );
    }

    #[test]
    fn a_room_with_a_window_and_a_door() {
        let mut s = Session::new();
        // Inner faces of a 5 × 4 m room, walls 0.45 m thick outward.
        let r = s
            .execute("geozed", &json!({ "points": [[0, 0], [5, 0], [5, 4], [0, 4]], "closed": true, "thickness": 0.45, "toward": [2, -1] }))
            .unwrap();
        assert_eq!(r["pieces"], 4);
        let walls = ents(&s, LAYER_WALLS);
        assert_eq!(walls.len(), 2);
        let hatch = ents(&s, LAYER_WALL_HATCH);
        let EntityKind::Hatch(h) = &hatch[0] else { panic!() };
        let total: f64 = h.loops.iter().map(|l| area(&l.vertices.iter().map(|v| v.p).collect::<Vec<_>>())).sum();
        // (5.9 × 4.9) − (5 × 4)
        assert!((total - (5.9 * 4.9 - 20.0)).abs() < 1e-9, "{total}");
        // A 1.2 m window in the south wall, picked on the inner face.
        s.execute("geootvor", &json!({ "at": [2.5, 0], "kind": "okno" })).unwrap();
        let walls = ents(&s, LAYER_WALLS);
        // Two faces opened into one open polyline each, plus two jambs.
        assert_eq!(walls.iter().filter(|k| matches!(k, EntityKind::LwPolyline(p) if !p.closed)).count(), 2, "{walls:?}");
        assert_eq!(walls.iter().filter(|k| matches!(k, EntityKind::Line(_))).count(), 2);
        let EntityKind::Hatch(h) = &ents(&s, LAYER_WALL_HATCH)[0] else { panic!() };
        let total2: f64 = h.loops.iter().map(|l| area(&l.vertices.iter().map(|v| v.p).collect::<Vec<_>>())).sum();
        assert!((total - total2 - 1.2 * 0.45).abs() < 1e-9, "{total} {total2}");
        assert_eq!(ents(&s, LAYER_OPENINGS).len(), 2);
        // A 0.8 m door in the east wall, picked on the outer face, opening inward.
        s.execute("geootvor", &json!({ "at": [5.45, 2], "kind": "dvere", "toward": [4, 2] })).unwrap();
        let ops = ents(&s, LAYER_OPENINGS);
        let arc = ops.iter().find_map(|k| if let EntityKind::Arc(a) = k { Some(a.clone()) } else { None }).unwrap();
        assert!((arc.radius - 0.8).abs() < 1e-9);
        // Hinged on the inner face.
        assert!((arc.center.x - 5.0).abs() < 1e-9, "{arc:?}");
        let mut sweep = arc.end - arc.start;
        if sweep < 0.0 {
            sweep += std::f64::consts::TAU;
        }
        assert!((sweep - std::f64::consts::FRAC_PI_2).abs() < 1e-9, "{arc:?}");
    }

    #[test]
    fn hostile_input_is_refused_not_panicking() {
        let mut s = Session::new();
        for (cmd, p) in [
            ("geozed", json!({})),
            ("geozed", json!({ "points": [[0, 0]] })),
            ("geozed", json!({ "points": [[0, 0], [1, 0]], "thickness": 0 })),
            ("geozed", json!({ "points": [[0, 0], [1, 0]], "thickness": 99 })),
            ("geozed", json!({ "points": [[0, 0], [0, 0]] })),
            ("geozed", json!({ "points": [[0, 0], [1, 0]], "side": "up" })),
            ("geootvor", json!({})),
            ("geootvor", json!({ "at": [0, 0] })),
        ] {
            assert!(s.execute(cmd, &p).is_err(), "{cmd} {p}");
        }
        s.execute("geozed", &json!({ "points": [[0, 0], [2, 0]], "thickness": 0.3 })).unwrap();
        // Too wide for the 2 m piece, and an unknown kind.
        assert!(s.execute("geootvor", &json!({ "at": [1, 0], "width": 5 })).is_err());
        assert!(s.execute("geootvor", &json!({ "at": [1, 0], "kind": "gate" })).is_err());
        let _ = offset(&[Vec2::ZERO, Vec2::ZERO, Vec2::new(1.0, 0.0)], false, 0.1);
    }
}
