//! Survey add-on: calculations on the terrain model (the 3DFACEs GEODMT puts on `GEO_DMT`).
//!
//! - GEOKUBATURY: cut and fill volumes inside a closed boundary against a level. Every triangle
//!   is clipped exactly to the boundary (split into convex pieces) and to the level, and the
//!   volume of each piece is its area times its mean height above or below the level (the height
//!   at its centroid, the terrain being flat within a triangle).
//! - GEOVYSKA: the terrain height at a point, written there.
//! - GEOPROFIL: a long section along a line or polyline, drawn as a profile with stations and
//!   heights.
//! - GEOSEZNAM: the GEO_body points back to a coordinate list (number, Y, X, Z).

use cadcraft_doc::{Common, EntityKind, HAlign, Handle, Line, LwPolyline, Point, Space, Text, VAlign};
use cadcraft_geom::{PolyVertex, Vec2, Vec3};
use serde_json::{Value, json};

use super::geo_dmt::LAYER_TIN;
use super::{CommandSpec, bad, f64_or, point_value, str_param};
use crate::{Accept, EngineError, Input, Interactive, Prompt, Result, Session, Step};

pub const LAYER_VOLUMES: &str = "GEO_kubatury";
pub const LAYER_SPOT: &str = "GEO_vyskove_koty";
pub const LAYER_PROFILE: &str = "GEO_profil";

/// Vertices of a boundary.
const MAX_BOUNDARY: usize = 20_000;
/// Triangle × boundary-piece pairs clipped at once.
const MAX_WORK: usize = 200_000_000;
/// Profile points.
const MAX_PROFILE: usize = 2_000_000;

pub(super) fn specs() -> Vec<CommandSpec> {
    vec![
        CommandSpec::new("geokubatury", "Cut and Fill Volumes...", run_geokubatury)
            .menu(&["Geo", "Cut and Fill Volumes..."])
            .alias(&["geovolumes", "geokub"])
            .params("{level: height in metres, boundary?: handle of a closed polyline (default: the selected one), label?: bool, write the result in the drawing (default true)} → {cut, fill, net, area, covered, message}")
            .interactive(|_| Ok(Box::new(KubM::default()))),
        CommandSpec::new("geovyska", "Terrain Height at Point", run_geovyska)
            .menu(&["Geo", "Terrain Height at Point"])
            .alias(&["geoheight"])
            .params("{at: [x, y], label?: bool (default true), scale?: map scale 1:N (default 500), decimals?: (default 2)} → {z}")
            .interactive(|_| Ok(Box::new(VyskaM))),
        CommandSpec::new("geoprofil", "Long Section...", run_geoprofil)
            .menu(&["Geo", "Long Section..."])
            .alias(&["geoprofile"])
            .params("{path?: handle of a line or polyline (default: the selected one), at: [x, y] where the profile is drawn, exaggeration?: vertical (default 10), datum?: reference height (default: below the lowest point), step?: station label spacing in metres (default 10), scale?: map scale 1:N for text (default 500)} → {length, points, min, max}")
            .interactive(|_| Ok(Box::new(ProfM::default()))),
        CommandSpec::new("geoseznam", "Export Points (Coordinate List)...", run_geoseznam)
            .menu(&["Geo", "Export Points..."])
            .alias(&["geoexport"])
            .params("{path?: file to write (otherwise the list is returned), sjtsk?: \"auto\"|true|false, decimals?: (default 3)} → {points, text?, message}")
            .interactive(|_| Ok(Box::new(SeznamM))),
    ]
}

// --- shared --------------------------------------------------------------------------------------

/// The TIN's triangles in `space`.
pub(super) fn tin(s: &Session, space: &Space) -> Result<Vec<[Vec3; 3]>> {
    tin_on(s, space, LAYER_TIN)
}

/// The triangles (3DFACEs) on `layer` in `space`.
pub(super) fn tin_on(s: &Session, space: &Space, layer: &str) -> Result<Vec<[Vec3; 3]>> {
    let d = s.doc()?;
    let Some(st) = d.space(space) else { return Ok(Vec::new()) };
    Ok(st
        .iter()
        .filter(|e| e.common.layer.eq_ignore_ascii_case(layer))
        .filter_map(|e| match &e.kind {
            EntityKind::Face3d(f) => {
                let [a, b, c, _] = f.corners;
                [a, b, c].iter().all(|v| v.x.is_finite() && v.y.is_finite() && v.z.is_finite()).then_some([a, b, c])
            }
            _ => None,
        })
        .collect())
}

fn no_tin(cmd: &str) -> EngineError {
    bad(cmd, "no terrain model: run GEODMT first")
}

/// The plane z = a·x + b·y + c through a triangle, or `None` for a vertical or degenerate one.
pub(super) fn plane(t: &[Vec3; 3]) -> Option<(f64, f64, f64)> {
    let [p, q, r] = *t;
    let (ux, uy, uz) = (q.x - p.x, q.y - p.y, q.z - p.z);
    let (vx, vy, vz) = (r.x - p.x, r.y - p.y, r.z - p.z);
    let det = ux * vy - uy * vx;
    if det.abs() < 1e-12 {
        return None;
    }
    let a = (uz * vy - uy * vz) / det;
    let b = (ux * vz - uz * vx) / det;
    let c = p.z - a * p.x - b * p.y;
    [a, b, c].iter().all(|v| v.is_finite()).then_some((a, b, c))
}

pub(super) fn cross2(o: Vec2, a: Vec2, b: Vec2) -> f64 {
    (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x)
}

pub(super) fn signed_area(p: &[Vec2]) -> f64 {
    let n = p.len();
    (0..n).map(|i| p.get(i).zip(p.get((i + 1) % n)).map_or(0.0, |(a, b)| a.x * b.y - b.x * a.y)).sum::<f64>() / 2.0
}

/// Area and centroid of a polygon.
pub(super) fn area_centroid(p: &[Vec2]) -> (f64, Vec2) {
    let n = p.len();
    let (mut a2, mut cx, mut cy) = (0.0, 0.0, 0.0);
    for i in 0..n {
        let (Some(u), Some(w)) = (p.get(i), p.get((i + 1) % n)) else { continue };
        let c = u.x * w.y - w.x * u.y;
        a2 += c;
        cx += (u.x + w.x) * c;
        cy += (u.y + w.y) * c;
    }
    if a2.abs() < 1e-18 {
        let m = p.iter().fold(Vec2::ZERO, |s, v| Vec2::new(s.x + v.x, s.y + v.y));
        let k = p.len().max(1) as f64;
        return (0.0, Vec2::new(m.x / k, m.y / k));
    }
    ((a2 / 2.0).abs(), Vec2::new(cx / (3.0 * a2), cy / (3.0 * a2)))
}

fn point_in_tri(p: Vec2, a: Vec2, b: Vec2, c: Vec2) -> bool {
    let (d1, d2, d3) = (cross2(a, b, p), cross2(b, c, p), cross2(c, a, p));
    let neg = d1 < -1e-12 || d2 < -1e-12 || d3 < -1e-12;
    let pos = d1 > 1e-12 || d2 > 1e-12 || d3 > 1e-12;
    !(neg && pos)
}

/// Ear-clip a simple polygon into triangles (counter-clockwise).
pub(super) fn ear_clip(poly: &[Vec2]) -> Vec<[Vec2; 3]> {
    let mut v: Vec<Vec2> = poly.to_vec();
    v.dedup_by(|a, b| (a.x - b.x).abs() < 1e-9 && (a.y - b.y).abs() < 1e-9);
    if v.len() > 2 && v.first().zip(v.last()).is_some_and(|(f, l)| (f.x - l.x).abs() < 1e-9 && (f.y - l.y).abs() < 1e-9) {
        v.pop();
    }
    if signed_area(&v) < 0.0 {
        v.reverse();
    }
    let mut out = Vec::new();
    let mut guard = v.len() * v.len() + 10;
    while v.len() > 3 && guard > 0 {
        guard -= 1;
        let n = v.len();
        let mut clipped = false;
        for i in 0..n {
            let (Some(&a), Some(&b), Some(&c)) = (v.get((i + n - 1) % n), v.get(i), v.get((i + 1) % n)) else { continue };
            if cross2(a, b, c) <= 1e-12 {
                continue;
            }
            let blocked = v.iter().enumerate().any(|(j, &p)| j != i && j != (i + n - 1) % n && j != (i + 1) % n && point_in_tri(p, a, b, c));
            if !blocked {
                out.push([a, b, c]);
                v.remove(i);
                clipped = true;
                break;
            }
        }
        if !clipped {
            // Not simple (or numerically stuck): drop a collinear/reflex vertex and go on.
            v.remove(0);
        }
    }
    if let [a, b, c] = v.as_slice()
        && cross2(*a, *b, *c).abs() > 1e-12
    {
        out.push([*a, *b, *c]);
    }
    out
}

/// Clip a polygon to the half-plane where `f(p) >= 0` (f linear).
pub(super) fn clip_half(poly: &[Vec2], f: impl Fn(Vec2) -> f64) -> Vec<Vec2> {
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
            out.push(Vec2::new(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)));
        }
    }
    out
}

/// Clip a convex polygon to a counter-clockwise triangle.
pub(super) fn clip_to_tri(poly: &[Vec2], t: &[Vec2; 3]) -> Vec<Vec2> {
    let mut p = poly.to_vec();
    for i in 0..3 {
        let (Some(&a), Some(&b)) = (t.get(i), t.get((i + 1) % 3)) else { continue };
        p = clip_half(&p, |q| cross2(a, b, q));
        if p.len() < 3 {
            return Vec::new();
        }
    }
    p
}

pub(super) fn bbox(p: &[Vec2]) -> (Vec2, Vec2) {
    p.iter().fold((Vec2::new(f64::INFINITY, f64::INFINITY), Vec2::new(f64::NEG_INFINITY, f64::NEG_INFINITY)), |(lo, hi), v| {
        (Vec2::new(lo.x.min(v.x), lo.y.min(v.y)), Vec2::new(hi.x.max(v.x), hi.y.max(v.y)))
    })
}

pub(super) fn overlaps(a: (Vec2, Vec2), b: (Vec2, Vec2)) -> bool {
    a.0.x <= b.1.x && b.0.x <= a.1.x && a.0.y <= b.1.y && b.0.y <= a.1.y
}

pub(super) fn ensure_layer(s: &mut Session, name: &str, aci: u8) -> Result<()> {
    let d = s.doc_mut()?;
    if d.ensure_layer(name)
        && let Some(l) = d.layer_mut(name)
    {
        l.color = cadcraft_color::Color::Index(aci);
    }
    Ok(())
}

#[allow(clippy::too_many_arguments)]
pub(super) fn text(
    s: &mut Session,
    space: &Space,
    layer: &str,
    at: Vec2,
    h: f64,
    value: String,
    rot: f64,
    align: (HAlign, VAlign),
) -> Result<Handle> {
    let anchored = align != (HAlign::Left, VAlign::Baseline);
    let t = Text {
        insert: Vec3::new(at.x, at.y, 0.0),
        align_pt: anchored.then(|| Vec3::new(at.x, at.y, 0.0)),
        height: h,
        value,
        rotation: rot,
        width_factor: 1.0,
        oblique: 0.0,
        style: "Standard".into(),
        halign: align.0,
        valign: align.1,
    };
    let common = Common { layer: layer.into(), ..s.current_common() };
    s.doc_mut()?.add(space, common, EntityKind::Text(t)).map_err(|e| bad("geo", e.to_string()))
}

pub(super) fn add(s: &mut Session, space: &Space, layer: &str, kind: EntityKind) -> Result<Handle> {
    let common = Common { layer: layer.into(), ..s.current_common() };
    s.doc_mut()?.add(space, common, kind).map_err(|e| bad("geo", e.to_string()))
}

/// The one handle named by `key`, else the single selected object.
pub(super) fn one_target(s: &Session, p: &Value, key: &str, cmd: &str) -> Result<Handle> {
    if let Some(v) = p.get(key) {
        return v
            .as_str()
            .and_then(Handle::parse_hex)
            .or_else(|| v.as_u64().map(Handle))
            .ok_or_else(|| bad(cmd, format!("`{key}` must be an object handle")));
    }
    match s.selection().as_slice() {
        [h] => Ok(*h),
        [] => Err(bad(cmd, format!("select the {key} first (or pass `{key}`)"))),
        _ => Err(bad(cmd, format!("select one {key} only"))),
    }
}

/// The plan vertices of a line or polyline (arcs are taken as chords).
pub(super) fn path_of(s: &Session, h: Handle, space: &Space) -> Option<(Vec<Vec2>, bool)> {
    let d = s.doc().ok()?;
    let e = d.space(space)?.get(h)?;
    match &e.kind {
        EntityKind::LwPolyline(pl) => Some((pl.vertices.iter().map(|v| v.p).collect(), pl.closed)),
        EntityKind::Polyline3d(pl) => Some((pl.points.iter().map(|v| v.xy()).collect(), pl.closed)),
        EntityKind::Line(l) => Some((vec![l.a.xy(), l.b.xy()], false)),
        _ => None,
    }
}

// --- GEOKUBATURY ---------------------------------------------------------------------------------

/// Cut (terrain above `level`) and fill (below) volumes, the area covered by the model, inside
/// `boundary`.
pub fn volumes(tris: &[[Vec3; 3]], boundary: &[Vec2], level: f64) -> std::result::Result<(f64, f64, f64), String> {
    // Work relative to the boundary: S-JTSK coordinates are ~10^6 m, and areas from products of
    // such numbers lose the digits that matter.
    let o = boundary.first().copied().unwrap_or(Vec2::ZERO);
    let boundary: Vec<Vec2> = boundary.iter().map(|v| Vec2::new(v.x - o.x, v.y - o.y)).collect();
    let tris: Vec<[Vec3; 3]> = tris.iter().map(|t| t.map(|v| Vec3::new(v.x - o.x, v.y - o.y, v.z))).collect();
    let pieces = ear_clip(&boundary);
    if pieces.is_empty() {
        return Err("the boundary encloses no area".into());
    }
    if tris.len().saturating_mul(pieces.len()) > MAX_WORK.saturating_mul(4) {
        return Err("the model and boundary are too large to compute at once".into());
    }
    let piece_boxes: Vec<_> = pieces.iter().map(|t| bbox(t)).collect();
    let (mut cut, mut fill, mut covered) = (0.0, 0.0, 0.0);
    let mut work = 0usize;
    for t in &tris {
        let Some((a, b, c)) = plane(t) else { continue };
        let mut flat = [t[0].xy(), t[1].xy(), t[2].xy()];
        if signed_area(&flat) < 0.0 {
            flat.swap(1, 2);
        }
        let tb = bbox(&flat);
        for (piece, pb) in pieces.iter().zip(&piece_boxes) {
            if !overlaps(tb, *pb) {
                continue;
            }
            work += 1;
            if work > MAX_WORK {
                return Err("the model and boundary are too large to compute at once".into());
            }
            let clipped = clip_to_tri(&flat, piece);
            if clipped.len() < 3 {
                continue;
            }
            covered += area_centroid(&clipped).0;
            let h = |q: Vec2| a * q.x + b * q.y + c - level;
            for (part, sign) in [(clip_half(&clipped, h), 1.0), (clip_half(&clipped, |q| -h(q)), -1.0)] {
                if part.len() < 3 {
                    continue;
                }
                let (area, cen) = area_centroid(&part);
                let v = area * h(cen) * sign;
                if v > 0.0 {
                    if sign > 0.0 {
                        cut += v;
                    } else {
                        fill += v;
                    }
                }
            }
        }
    }
    Ok((cut, fill, covered))
}

fn run_geokubatury(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "geokubatury";
    let level = p
        .get("level")
        .and_then(Value::as_f64)
        .filter(|v| v.is_finite() && v.abs() < 1e6)
        .ok_or_else(|| bad(CMD, "`level` (height in metres) is required"))?;
    let space = s.space();
    let h = one_target(s, p, "boundary", CMD)?;
    let (poly, _) = path_of(s, h, &space).ok_or_else(|| bad(CMD, "the boundary must be a closed polyline"))?;
    if poly.len() < 3 || poly.len() > MAX_BOUNDARY {
        return Err(bad(CMD, format!("the boundary needs 3 to {MAX_BOUNDARY} vertices")));
    }
    let tris = tin(s, &space)?;
    if tris.is_empty() {
        return Err(no_tin(CMD));
    }
    let (cut, fill, covered) = volumes(&tris, &poly, level).map_err(|e| bad(CMD, e))?;
    let area = signed_area(&poly).abs();
    let partial = covered < area * 0.999;
    if p.get("label").and_then(Value::as_bool).unwrap_or(true) {
        ensure_layer(s, LAYER_VOLUMES, 6)?;
        let (_, cen) = area_centroid(&poly);
        let size = (area.sqrt() / 25.0).clamp(0.2, 5.0);
        let lines = [
            format!("Srovnání na {level:.2} m"),
            format!("Výkop: {cut:.1} m³"),
            format!("Násyp: {fill:.1} m³"),
            format!("Plocha: {area:.1} m²{}", if partial { format!(" (model jen {covered:.1} m²)") } else { String::new() }),
        ];
        for (i, l) in lines.into_iter().enumerate() {
            let at = Vec2::new(cen.x, cen.y + (1.5 - i as f64) * 1.6 * size);
            text(s, &space, LAYER_VOLUMES, at, size, l, 0.0, (HAlign::Center, VAlign::Middle))?;
        }
    }
    Ok(json!({
        "cut": cut, "fill": fill, "net": cut - fill, "area": area, "covered": covered,
        "message": format!(
            "Level {level:.2}: cut {cut:.1} m³, fill {fill:.1} m³ (net {:+.1} m³) over {area:.1} m²{}",
            cut - fill,
            if partial { format!("; the terrain model covers only {covered:.1} m² of it") } else { String::new() }
        ),
    }))
}

pub(super) fn say(s: &mut Session, r: Result<Value>) {
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

/// A number typed at a prompt (decimal comma accepted).
pub(super) fn typed_number(t: &str) -> Option<f64> {
    t.trim().replace(',', ".").parse::<f64>().ok().filter(|v| v.is_finite())
}

#[derive(Default)]
struct KubM {
    boundary: Option<Handle>,
}

impl Interactive for KubM {
    fn name(&self) -> &'static str {
        "GEOKUBATURY"
    }
    fn prompt(&self, _s: &Session) -> Prompt {
        match self.boundary {
            None => Prompt::new("Select the closed boundary polyline", Accept::SELECT),
            Some(_) => Prompt::new("Level to cut and fill to (m)", Accept::NUMBER),
        }
    }
    fn input(&mut self, s: &mut Session, i: Input) -> Result<Step> {
        match (&self.boundary, i) {
            (_, Input::Cancel) => Ok(Step::Cancel),
            (None, Input::Pick(hs)) => {
                self.boundary = hs.first().copied();
                Ok(Step::Continue)
            }
            (Some(b), Input::Text(t) | Input::Keyword(t)) => match typed_number(&t) {
                Some(level) => {
                    let r = run_geokubatury(s, &json!({ "level": level, "boundary": b.hex() }));
                    say(s, r);
                    Ok(Step::Done)
                }
                None => {
                    s.echo("Enter the level in metres, like 305.20.");
                    Ok(Step::Continue)
                }
            },
            _ => Ok(Step::Continue),
        }
    }
}

#[derive(Default)]
struct ProfM {
    path: Option<Handle>,
    at: Option<Vec2>,
}

impl Interactive for ProfM {
    fn name(&self) -> &'static str {
        "GEOPROFIL"
    }
    fn prompt(&self, _s: &Session) -> Prompt {
        match (self.path, self.at) {
            (None, _) => Prompt::new("Select the line or polyline of the section", Accept::SELECT),
            (Some(_), None) => Prompt::new("Pick where to draw the profile", Accept::POINT),
            (Some(_), Some(_)) => Prompt::new("Vertical exaggeration", Accept::NUMBER).default("10"),
        }
    }
    fn input(&mut self, s: &mut Session, i: Input) -> Result<Step> {
        match (self.path, self.at, i) {
            (_, _, Input::Cancel) => Ok(Step::Cancel),
            (None, _, Input::Pick(hs)) => {
                self.path = hs.first().copied();
                Ok(Step::Continue)
            }
            (Some(_), None, Input::Point(p)) => {
                self.at = Some(p);
                Ok(Step::Continue)
            }
            (Some(path), Some(at), i @ (Input::Enter | Input::Text(_) | Input::Keyword(_))) => {
                let ex = match i {
                    Input::Text(t) | Input::Keyword(t) => match typed_number(&t) {
                        Some(v) if v > 0.0 => v,
                        _ => {
                            s.echo("Enter a positive number, like 10.");
                            return Ok(Step::Continue);
                        }
                    },
                    _ => 10.0,
                };
                let r = run_geoprofil(s, &json!({ "path": path.hex(), "at": [at.x, at.y], "exaggeration": ex }));
                say(s, r);
                Ok(Step::Done)
            }
            _ => Ok(Step::Continue),
        }
    }
}

struct SeznamM;

impl Interactive for SeznamM {
    fn name(&self) -> &'static str {
        "GEOSEZNAM"
    }
    fn prompt(&self, _s: &Session) -> Prompt {
        Prompt::new("File name to write the coordinate list to", Accept::TEXT)
    }
    fn input(&mut self, s: &mut Session, i: Input) -> Result<Step> {
        match i {
            Input::Cancel | Input::Enter => Ok(Step::Cancel),
            Input::Text(t) | Input::Keyword(t) => {
                let path = t.trim().trim_matches('"').to_string();
                let r = run_geoseznam(s, &json!({ "path": path }));
                say(s, r);
                Ok(Step::Done)
            }
            _ => Ok(Step::Continue),
        }
    }
}

// --- GEOVYSKA ------------------------------------------------------------------------------------

/// The terrain height at `p`, if a triangle covers it.
pub fn height_at(tris: &[[Vec3; 3]], p: Vec2) -> Option<f64> {
    // Relative to the point, for precision far from the origin.
    tris.iter().find_map(|t| {
        let t = t.map(|v| Vec3::new(v.x - p.x, v.y - p.y, v.z));
        point_in_tri(Vec2::ZERO, t[0].xy(), t[1].xy(), t[2].xy()).then(|| plane(&t)).flatten().map(|(_, _, c)| c)
    })
}

fn run_geovyska(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "geovyska";
    let at = p.get("at").and_then(point_value).ok_or_else(|| bad(CMD, "`at` ([x, y]) is required"))?;
    let space = s.space();
    let tris = tin(s, &space)?;
    if tris.is_empty() {
        return Err(no_tin(CMD));
    }
    let z = height_at(&tris, at).ok_or_else(|| bad(CMD, "the point is outside the terrain model"))?;
    let decimals = p.get("decimals").and_then(Value::as_u64).unwrap_or(2).min(4) as usize;
    if p.get("label").and_then(Value::as_bool).unwrap_or(true) {
        let scale = f64_or(p, "scale", 500.0).clamp(1.0, 1e6);
        let h = 0.002 * scale;
        ensure_layer(s, LAYER_SPOT, 1)?;
        add(s, &space, LAYER_SPOT, EntityKind::Point(Point { p: Vec3::new(at.x, at.y, z), angle: 0.0 }))?;
        text(s, &space, LAYER_SPOT, Vec2::new(at.x + 0.6 * h, at.y - 0.5 * h), h, format!("{z:.decimals$}"), 0.0, (HAlign::Left, VAlign::Baseline))?;
    }
    Ok(json!({ "z": z }))
}

struct VyskaM;

impl Interactive for VyskaM {
    fn name(&self) -> &'static str {
        "GEOVYSKA"
    }
    fn prompt(&self, _s: &Session) -> Prompt {
        Prompt::new("Pick a point for its terrain height", Accept::POINT)
    }
    fn input(&mut self, s: &mut Session, i: Input) -> Result<Step> {
        match i {
            Input::Point(at) => {
                match run_geovyska(s, &json!({ "at": [at.x, at.y] })) {
                    Ok(r) => {
                        let z = r.get("z").and_then(Value::as_f64).unwrap_or(f64::NAN);
                        s.echo(format!("Height: {z:.3} m"));
                    }
                    Err(EngineError::BadParams { msg, .. }) => s.echo(msg),
                    Err(e) => s.echo(e.to_string()),
                }
                Ok(Step::Continue)
            }
            Input::Enter | Input::Cancel => Ok(Step::Done),
            _ => Ok(Step::Continue),
        }
    }
}

// --- GEOPROFIL -----------------------------------------------------------------------------------

/// Where the terrain crosses the path: (station, height) in station order.
pub fn section(tris: &[[Vec3; 3]], path: &[Vec2]) -> Vec<(f64, f64)> {
    let o = path.first().copied().unwrap_or(Vec2::ZERO);
    let path: Vec<Vec2> = path.iter().map(|v| Vec2::new(v.x - o.x, v.y - o.y)).collect();
    let tris: Vec<[Vec3; 3]> = tris.iter().map(|t| t.map(|v| Vec3::new(v.x - o.x, v.y - o.y, v.z))).collect();
    let mut out: Vec<(f64, f64)> = Vec::new();
    let mut start = 0.0;
    for w in path.windows(2) {
        let (Some(&a), Some(&b)) = (w.first(), w.get(1)) else { continue };
        let len = (b.x - a.x).hypot(b.y - a.y);
        if len < 1e-12 {
            continue;
        }
        let seg_box = bbox(&[a, b]);
        for t in &tris {
            let flat = [t[0].xy(), t[1].xy(), t[2].xy()];
            if !overlaps(seg_box, bbox(&flat)) {
                continue;
            }
            let Some((pa, pb, pc)) = plane(t) else { continue };
            let mut ccw = flat;
            if signed_area(&ccw) < 0.0 {
                ccw.swap(1, 2);
            }
            // Clip the segment's parameter range to the triangle.
            let (mut t0, mut t1) = (0.0f64, 1.0f64);
            for i in 0..3 {
                let (Some(&e0), Some(&e1)) = (ccw.get(i), ccw.get((i + 1) % 3)) else { continue };
                let fa = cross2(e0, e1, a);
                let fb = cross2(e0, e1, b);
                if fa < 0.0 && fb < 0.0 {
                    t0 = 1.0;
                    t1 = 0.0;
                    break;
                }
                if fa < 0.0 || fb < 0.0 {
                    let tc = fa / (fa - fb);
                    if fa < 0.0 {
                        t0 = t0.max(tc);
                    } else {
                        t1 = t1.min(tc);
                    }
                }
            }
            if t1 < t0 {
                continue;
            }
            for tt in [t0, t1] {
                let q = Vec2::new(a.x + tt * (b.x - a.x), a.y + tt * (b.y - a.y));
                out.push((start + tt * len, pa * q.x + pb * q.y + pc));
            }
            if out.len() > MAX_PROFILE {
                break;
            }
        }
        start += len;
    }
    out.sort_by(|p, q| p.0.total_cmp(&q.0));
    out.dedup_by(|p, q| (p.0 - q.0).abs() < 1e-6);
    out
}

fn run_geoprofil(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "geoprofil";
    let space = s.space();
    let h = one_target(s, p, "path", CMD)?;
    let (mut path, closed) = path_of(s, h, &space).ok_or_else(|| bad(CMD, "the path must be a line or polyline"))?;
    if closed && let Some(f) = path.first().copied() {
        path.push(f);
    }
    let at = p.get("at").and_then(point_value).ok_or_else(|| bad(CMD, "`at` ([x, y], where to draw the profile) is required"))?;
    let ex = f64_or(p, "exaggeration", 10.0);
    let step = f64_or(p, "step", 10.0);
    let scale = f64_or(p, "scale", 500.0);
    if !(ex.is_finite() && (0.01..=1000.0).contains(&ex) && step.is_finite() && step > 0.0 && scale.is_finite() && (1.0..=1e6).contains(&scale)) {
        return Err(bad(CMD, "`exaggeration`, `step` and `scale` must be positive"));
    }
    let tris = tin(s, &space)?;
    if tris.is_empty() {
        return Err(no_tin(CMD));
    }
    let prof = section(&tris, &path);
    if prof.len() < 2 {
        return Err(bad(CMD, "the path doesn't cross the terrain model"));
    }
    let length: f64 = path
        .windows(2)
        .map(|w| match w {
            [a, b] => (b.x - a.x).hypot(b.y - a.y),
            _ => 0.0,
        })
        .sum();
    if length / step > 10_000.0 {
        return Err(bad(CMD, "too many stations: use a larger `step`"));
    }
    let (zmin, zmax) = prof.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), (_, z)| (lo.min(*z), hi.max(*z)));
    let datum = p.get("datum").and_then(Value::as_f64).filter(|v| v.is_finite()).unwrap_or((zmin - 1.0).floor());
    let th = 0.002 * scale;
    let y = |z: f64| at.y + (z - datum) * ex;
    ensure_layer(s, LAYER_PROFILE, 4)?;
    // The terrain line, broken where the path leaves the model.
    let mut run: Vec<PolyVertex> = Vec::new();
    let mut runs = Vec::new();
    let mut last: Option<f64> = None;
    for (st, z) in &prof {
        if let Some(l) = last
            && st - l > 1e-3
            && height_at(&tris, along(&path, (st + l) / 2.0)).is_none()
        {
            runs.push(std::mem::take(&mut run));
        }
        run.push(PolyVertex::new(Vec2::new(at.x + st, y(*z))));
        last = Some(*st);
    }
    runs.push(run);
    for r in runs.into_iter().filter(|r| r.len() >= 2) {
        add(
            s,
            &space,
            LAYER_PROFILE,
            EntityKind::LwPolyline(LwPolyline { vertices: r, closed: false, const_width: 0.0, elevation: 0.0, plinegen: false }),
        )?;
    }
    // Datum line and its label.
    add(s, &space, LAYER_PROFILE, EntityKind::Line(Line { a: Vec3::new(at.x, at.y, 0.0), b: Vec3::new(at.x + length, at.y, 0.0) }))?;
    text(s, &space, LAYER_PROFILE, Vec2::new(at.x - th, at.y), th, format!("S.R. {datum:.2}"), 0.0, (HAlign::Right, VAlign::Middle))?;
    // GEONIVELETA reads the exaggeration back from here.
    text(
        s,
        &space,
        LAYER_PROFILE,
        Vec2::new(at.x - th, at.y - 1.5 * th),
        th * 0.8,
        format!("převýšení {ex}×"),
        0.0,
        (HAlign::Right, VAlign::Middle),
    )?;
    // Stations: a tick, the station below the datum and the height above the terrain.
    let n = (length / step).floor() as usize;
    let mut stations: Vec<f64> = (0..=n).map(|i| i as f64 * step).collect();
    if stations.last().is_some_and(|l| length - l > 1e-6) {
        stations.push(length);
    }
    for st in stations {
        let x = at.x + st;
        let Some(z) = height_at(&tris, along(&path, st)) else { continue };
        add(s, &space, LAYER_PROFILE, EntityKind::Line(Line { a: Vec3::new(x, at.y, 0.0), b: Vec3::new(x, y(z), 0.0) }))?;
        text(
            s,
            &space,
            LAYER_PROFILE,
            Vec2::new(x, at.y - 0.8 * th),
            th * 0.8,
            format!("{}+{:06.2}", (st / 1000.0).floor() as i64, st % 1000.0),
            // Read from below, ending just under the datum line.
            std::f64::consts::FRAC_PI_2,
            (HAlign::Right, VAlign::Middle),
        )?;
        text(
            s,
            &space,
            LAYER_PROFILE,
            Vec2::new(x - 0.3 * th, y(z) + 0.6 * th),
            th * 0.8,
            format!("{z:.2}"),
            std::f64::consts::FRAC_PI_2,
            (HAlign::Left, VAlign::Middle),
        )?;
    }
    Ok(json!({ "length": length, "points": prof.len(), "min": zmin, "max": zmax, "datum": datum,
        "message": format!("Long section {length:.2} m, heights {zmin:.2} to {zmax:.2}, drawn 1:{} vertically ×{ex}", scale) }))
}

/// The plan point at station `st` along `path`.
fn along(path: &[Vec2], st: f64) -> Vec2 {
    let mut left = st;
    for w in path.windows(2) {
        let (Some(&a), Some(&b)) = (w.first(), w.get(1)) else { continue };
        let len = (b.x - a.x).hypot(b.y - a.y);
        if left <= len && len > 0.0 {
            let t = left / len;
            return Vec2::new(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y));
        }
        left -= len;
    }
    path.last().copied().unwrap_or(Vec2::ZERO)
}

// --- GEOSEZNAM -----------------------------------------------------------------------------------

fn run_geoseznam(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "geoseznam";
    let space = s.space();
    let decimals = p.get("decimals").and_then(Value::as_u64).unwrap_or(3).min(6) as usize;
    let numbered = super::geo_pohled::numbered_points(s, &space, None)?;
    if numbered.is_empty() {
        return Err(bad(CMD, "no points on GEO_body"));
    }
    let pts: Vec<Vec3> = numbered.iter().map(|(_, q)| *q).collect();
    let sjtsk = match p.get("sjtsk") {
        None => pts.iter().all(|q| q.x < -300_000.0 && q.y < -900_000.0),
        Some(Value::Bool(b)) => *b,
        Some(Value::String(t)) if t.eq_ignore_ascii_case("auto") => pts.iter().all(|q| q.x < -300_000.0 && q.y < -900_000.0),
        Some(_) => return Err(bad(CMD, "`sjtsk` must be true, false or \"auto\"")),
    };
    let mut out = String::new();
    for (num, q) in &numbered {
        let (yy, xx) = if sjtsk { (-q.x, -q.y) } else { (q.x, q.y) };
        out.push_str(&format!("{num} {yy:.decimals$} {xx:.decimals$} {:.decimals$}\n", q.z));
    }
    let count = pts.len();
    if let Some(path) = str_param(p, "path") {
        #[cfg(not(target_arch = "wasm32"))]
        {
            std::fs::write(path, &out).map_err(|e| bad(CMD, format!("{path}: {e}")))?;
            return Ok(json!({ "points": count, "message": format!("{count} points written to {path}") }));
        }
        #[cfg(target_arch = "wasm32")]
        return Err(bad(CMD, format!("{path}: files can't be written on the web; leave out `path`")));
    }
    Ok(json!({ "points": count, "text": out, "message": format!("{count} points") }))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A 20 × 20 m plane rising 1 m per 10 m eastwards from 100 m, as two triangles.
    fn ramp() -> Vec<[Vec3; 3]> {
        let (a, b, c, d) = (Vec3::new(0.0, 0.0, 100.0), Vec3::new(20.0, 0.0, 102.0), Vec3::new(20.0, 20.0, 102.0), Vec3::new(0.0, 20.0, 100.0));
        vec![[a, b, c], [a, c, d]]
    }

    fn sq(x0: f64, y0: f64, x1: f64, y1: f64) -> Vec<Vec2> {
        vec![Vec2::new(x0, y0), Vec2::new(x1, y0), Vec2::new(x1, y1), Vec2::new(x0, y1)]
    }

    #[test]
    fn volumes_are_exact_far_from_the_origin() {
        let shift = |t: &[Vec3; 3]| t.map(|v| Vec3::new(v.x - 742_000.0, v.y - 1_043_000.0, v.z));
        let tris: Vec<_> = ramp().iter().map(shift).collect();
        let b: Vec<Vec2> = sq(0.0, 0.0, 20.0, 20.0).iter().map(|v| Vec2::new(v.x - 742_000.0, v.y - 1_043_000.0)).collect();
        let (cut, fill, cov) = volumes(&tris, &b, 101.0).unwrap();
        assert!((cut - 100.0).abs() < 1e-9 && (fill - 100.0).abs() < 1e-9 && (cov - 400.0).abs() < 1e-9, "{cut} {fill} {cov}");
        assert!((height_at(&tris, Vec2::new(5.0 - 742_000.0, 7.0 - 1_043_000.0)).unwrap() - 100.5).abs() < 1e-9);
    }

    #[test]
    fn volumes_are_exact() {
        // Level 101 cuts the ramp in half: a wedge of 10 × 20 × 1 / 2 = 100 m³ each side.
        let (cut, fill, cov) = volumes(&ramp(), &sq(0.0, 0.0, 20.0, 20.0), 101.0).unwrap();
        assert!((cut - 100.0).abs() < 1e-9 && (fill - 100.0).abs() < 1e-9 && (cov - 400.0).abs() < 1e-9, "{cut} {fill} {cov}");
        // All above 99: mean height 2 m over 400 m².
        let (cut, fill, _) = volumes(&ramp(), &sq(0.0, 0.0, 20.0, 20.0), 99.0).unwrap();
        assert!((cut - 800.0).abs() < 1e-9 && fill.abs() < 1e-12);
        // A concave (L-shaped) boundary, clockwise; partly off the model.
        let l = vec![
            Vec2::new(0.0, 0.0),
            Vec2::new(0.0, 30.0),
            Vec2::new(10.0, 30.0),
            Vec2::new(10.0, 10.0),
            Vec2::new(20.0, 10.0),
            Vec2::new(20.0, 0.0),
        ];
        let (cut, _, cov) = volumes(&ramp(), &l, 100.0).unwrap();
        // Covered: 10×20 + 10×10 = 300 m². Heights: x/10 above 100 → left strip mean 0.5 over 200, right block mean 1.5 over 100.
        assert!((cov - 300.0).abs() < 1e-9, "{cov}");
        assert!((cut - (0.5 * 200.0 + 1.5 * 100.0)).abs() < 1e-9, "{cut}");
    }

    #[test]
    fn heights_and_sections() {
        let t = ramp();
        assert!((height_at(&t, Vec2::new(5.0, 7.0)).unwrap() - 100.5).abs() < 1e-12);
        assert_eq!(height_at(&t, Vec2::new(30.0, 7.0)), None);
        let prof = section(&t, &[Vec2::new(-5.0, 5.0), Vec2::new(25.0, 5.0)]);
        assert!((prof.first().unwrap().0 - 5.0).abs() < 1e-9 && (prof.first().unwrap().1 - 100.0).abs() < 1e-9, "{prof:?}");
        assert!((prof.last().unwrap().0 - 25.0).abs() < 1e-9 && (prof.last().unwrap().1 - 102.0).abs() < 1e-9, "{prof:?}");
    }

    #[test]
    fn commands_end_to_end() {
        let mut s = Session::new();
        let mut text = String::new();
        for i in 0..5 {
            for j in 0..5 {
                text.push_str(&format!("{} {} {} {}\n", 100 + i * 5 + j, i * 5, j * 5, 100.0 + i as f64 * 0.5));
            }
        }
        s.execute("geobody", &json!({ "text": text, "sjtsk": false })).unwrap();
        s.execute("geodmt", &json!({})).unwrap();
        let b = s.execute("pline", &json!({ "points": [[0, 0], [20, 0], [20, 20], [0, 20]], "closed": true }));
        let h = match b {
            Ok(v) => v.get("handle").and_then(Value::as_str).map(str::to_string),
            Err(_) => None,
        };
        let boundary = match h {
            Some(h) => h,
            None => {
                let space = s.space();
                let pl = LwPolyline {
                    vertices: sq(0.0, 0.0, 20.0, 20.0).into_iter().map(PolyVertex::new).collect(),
                    closed: true,
                    const_width: 0.0,
                    elevation: 0.0,
                    plinegen: false,
                };
                add(&mut s, &space, "0", EntityKind::LwPolyline(pl)).unwrap().hex()
            }
        };
        let r = s.execute("geokubatury", &json!({ "level": 101, "boundary": boundary })).unwrap();
        // Heights 100..102 linear in x: half above, half below 101: 100 m³ each.
        assert!((r["cut"].as_f64().unwrap() - 100.0).abs() < 1e-6, "{r}");
        assert!((r["fill"].as_f64().unwrap() - 100.0).abs() < 1e-6, "{r}");
        let r = s.execute("geovyska", &json!({ "at": [12.5, 3] })).unwrap();
        assert!((r["z"].as_f64().unwrap() - 101.25).abs() < 1e-9);
        let r = s.execute("geoseznam", &json!({})).unwrap();
        let t = r["text"].as_str().unwrap();
        assert!(t.lines().any(|l| l.starts_with("100 0.000 0.000 100.000")), "{t}");
        assert_eq!(t.lines().count(), 25);
        assert!(s.execute("geoprofil", &json!({ "path": boundary, "at": [0, -50] })).is_ok());
    }

    #[test]
    fn ear_clip_handles_shapes() {
        let total = |p: &[Vec2]| ear_clip(p).iter().map(|t| signed_area(t).abs()).sum::<f64>();
        assert!((total(&sq(0.0, 0.0, 2.0, 3.0)) - 6.0).abs() < 1e-12);
        let l = vec![Vec2::new(0.0, 0.0), Vec2::new(2.0, 0.0), Vec2::new(2.0, 1.0), Vec2::new(1.0, 1.0), Vec2::new(1.0, 2.0), Vec2::new(0.0, 2.0)];
        assert!((total(&l) - 3.0).abs() < 1e-12);
        assert!(ear_clip(&[Vec2::ZERO, Vec2::ZERO, Vec2::ZERO]).is_empty());
    }

    #[test]
    fn hostile_input_is_refused_not_panicking() {
        let mut s = Session::new();
        for (cmd, p) in [
            ("geokubatury", json!({})),
            ("geokubatury", json!({ "level": "x" })),
            ("geokubatury", json!({ "level": 1, "boundary": "zz" })),
            ("geovyska", json!({})),
            ("geovyska", json!({ "at": [0, 0] })),
            ("geoprofil", json!({})),
            ("geoseznam", json!({ "sjtsk": 3 })),
        ] {
            assert!(s.execute(cmd, &p).is_err(), "{cmd} {p}");
        }
        let _ = volumes(&[[Vec3::new(0.0, 0.0, f64::NAN); 3]], &sq(0.0, 0.0, 1.0, 1.0), 0.0);
        let _ = section(&[], &[]);
    }
}
