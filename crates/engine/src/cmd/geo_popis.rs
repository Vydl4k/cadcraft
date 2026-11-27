//! Survey add-on: describing the situation and the building survey.
//!
//! - GEOPARCELA writes a parcel number inside a parcel's boundary (a building parcel's number
//!   gets the short line above it the cadastral map gives it).
//! - GEOPOPISSITE labels a utility line along its course (once, or every so many metres).
//! - GEOPOPIS writes a description with a leader line.
//! - GEOMISTNOST labels a room: its number and name, floor area, and clear height.
//! - GEOSCHODY draws a straight flight of stairs in plan: outline, treads, walking line with its
//!   start circle and end arrow, and the step count with rise and going.
//! - GEOSIPKA draws an arrow.
//!
//! Text sizes come from the survey settings (the "notes" lettering, 2.5 mm on paper by default).

use cadcraft_doc::{Circle, EntityKind, HAlign, Handle, Line, LwPolyline, VAlign};
use cadcraft_geom::{PolyVertex, Vec2, Vec3, polygon_centroid};
use serde_json::{Value, json};

use super::geo_calc::{add, ensure_layer, one_target, path_of, say, signed_area, text, typed_number};
use super::geo_nastaveni as nast;
use super::geo_rek::inside;
use super::{CommandSpec, bad, point_value, str_param};
use crate::{Accept, Input, Interactive, Prompt, Result, Session, Step};

pub const LAYER_PARCELS: &str = "GEO_parcely";
pub const LAYER_UTILITY: &str = "GEO_popis_site";
pub const LAYER_NOTES: &str = "GEO_popis";
pub const LAYER_ROOMS: &str = "GEO_mistnosti";
pub const LAYER_STAIRS: &str = "GEO_schody";
pub const LAYER_ARROWS: &str = "GEO_sipky";
/// Labels along one line, at most.
const MAX_LABELS: usize = 10_000;

pub(super) fn specs() -> Vec<CommandSpec> {
    vec![
        CommandSpec::new("geoparcela", "Parcel Number...", run_parcela)
            .menu(&["Geo", "Labels", "Parcel Number..."])
            .alias(&["geoparcel"])
            .params("{boundary: handle of the parcel's closed polyline, number: e.g. \"123/4\", building?: bool (a building parcel: a line above the number), at?: [x, y] (default inside the parcel)} → {handle, message}")
            .interactive(|_| Ok(Box::new(ParcM::default()))),
        CommandSpec::new("geopopissite", "Utility Line Label...", run_site)
            .menu(&["Geo", "Labels", "Utility Line Label..."])
            .alias(&["geoutilitylabel"])
            .params("{line: handle of a line or polyline, text: e.g. \"V DN150\", at?: [x, y] near the line (default its middle), every?: metres between repeated labels} → {labels, message}")
            .interactive(|_| Ok(Box::new(SiteM::default()))),
        CommandSpec::new("geopopis", "Label with Leader...", run_popis)
            .menu(&["Geo", "Labels", "Label with Leader..."])
            .alias(&["geolabel"])
            .params("{from: [x, y] what is described, to: [x, y] where the text goes, text} → {message}")
            .interactive(|_| Ok(Box::new(PopisM::default()))),
        CommandSpec::new("geomistnost", "Room Label...", run_mistnost)
            .menu(&["Geo", "Building", "Room Label..."])
            .alias(&["georoom"])
            .params("{boundary: handle of the room's closed polyline, name?: e.g. \"1.01 Kuchyně\", floor?: floor finish, height?: clear height (m), at?: [x, y]} → {area, message}")
            .interactive(|_| Ok(Box::new(RoomM::default()))),
        CommandSpec::new("geoschody", "Stairs...", run_schody)
            .menu(&["Geo", "Building", "Stairs..."])
            .alias(&["geostairs"])
            .params("{from: [x, y] foot of the walking line, to: [x, y] its head, width: m, steps: count, rise?: total height (m)} → {going, riser, message}")
            .interactive(|_| Ok(Box::new(StairsM::default()))),
        CommandSpec::new("geosipka", "Arrow...", run_sipka)
            .menu(&["Geo", "Labels", "Arrow..."])
            .alias(&["geoarrow"])
            .params("{from: [x, y], to: [x, y] the tip, size?: head length (m, default from the scale)} → {message}")
            .interactive(|_| Ok(Box::new(ArrowM::default()))),
    ]
}

fn notes_h(s: &Session) -> f64 {
    let n = nast::load(s);
    n.height(&n.notes)
}

fn pt(p: &Value, key: &str, cmd: &str) -> Result<Vec2> {
    p.get(key).and_then(point_value).filter(|v| v.x.abs() < 1e9 && v.y.abs() < 1e9).ok_or_else(|| bad(cmd, format!("`{key}`: [x, y] is required")))
}

/// The closed outline of `h`.
fn outline(s: &Session, h: Handle, cmd: &str) -> Result<Vec<Vec2>> {
    let space = s.space();
    let (mut v, closed) = path_of(s, h, &space).ok_or_else(|| bad(cmd, "the boundary must be a polyline"))?;
    let joined = v.first().zip(v.last()).is_some_and(|(a, b)| (a.x - b.x).hypot(a.y - b.y) < 1e-9);
    if !closed && !joined {
        return Err(bad(cmd, "the boundary must be a closed polyline"));
    }
    if joined && v.len() > 3 {
        v.pop();
    }
    if v.len() < 3 || signed_area(&v).abs() < 1e-12 {
        return Err(bad(cmd, "the boundary has no area"));
    }
    Ok(v)
}

/// A point well inside a polygon: its centroid, or the middle of the widest stretch inside it on
/// the centroid's level.
fn inner_point(poly: &[Vec2]) -> Vec2 {
    let c = polygon_centroid(poly).filter(|c| c.x.is_finite() && c.y.is_finite()).unwrap_or_else(|| poly.first().copied().unwrap_or(Vec2::ZERO));
    if inside(c, poly) {
        return c;
    }
    let n = poly.len();
    let mut xs: Vec<f64> = (0..n)
        .filter_map(|i| {
            let (a, b) = (poly.get(i)?, poly.get((i + 1) % n)?);
            ((a.y > c.y) != (b.y > c.y)).then(|| a.x + (c.y - a.y) * (b.x - a.x) / (b.y - a.y))
        })
        .collect();
    xs.sort_by(f64::total_cmp);
    xs.chunks(2)
        .filter_map(|w| w.first().zip(w.get(1)))
        .max_by(|a, b| (a.1 - a.0).total_cmp(&(b.1 - b.0)))
        .map_or(c, |(l, r)| Vec2::new((l + r) / 2.0, c.y))
}

/// An angle turned so text reads left to right.
fn readable(a: f64) -> f64 {
    let a = a.rem_euclid(std::f64::consts::TAU);
    if a > std::f64::consts::FRAC_PI_2 && a <= 3.0 * std::f64::consts::FRAC_PI_2 {
        a - std::f64::consts::PI
    } else if a > std::f64::consts::PI {
        a - std::f64::consts::TAU
    } else {
        a
    }
}

fn line(s: &mut Session, layer: &str, a: Vec2, b: Vec2) -> Result<Handle> {
    let space = s.space();
    add(s, &space, layer, EntityKind::Line(Line { a: Vec3::new(a.x, a.y, 0.0), b: Vec3::new(b.x, b.y, 0.0) }))
}

fn run_parcela(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "geoparcela";
    let b = one_target(s, p, "boundary", CMD)?;
    let poly = outline(s, b, CMD)?;
    let number = str_param(p, "number")
        .map(str::trim)
        .filter(|t| !t.is_empty() && t.chars().count() <= 32)
        .ok_or_else(|| bad(CMD, "`number`: the parcel number is required"))?
        .to_string();
    let at = match p.get("at") {
        Some(_) => pt(p, "at", CMD)?,
        None => inner_point(&poly),
    };
    let building = p.get("building").and_then(Value::as_bool).unwrap_or(false);
    let h = notes_h(s);
    ensure_layer(s, LAYER_PARCELS, 7)?;
    let space = s.space();
    let th = text(s, &space, LAYER_PARCELS, at, h, number.clone(), 0.0, (HAlign::Center, VAlign::Middle))?;
    if building {
        // The line above a building parcel's number, about as wide as the number.
        let w = 0.45 * h * number.chars().count().max(1) as f64;
        line(s, LAYER_PARCELS, Vec2::new(at.x - w / 2.0, at.y + 0.8 * h), Vec2::new(at.x + w / 2.0, at.y + 0.8 * h))?;
    }
    Ok(json!({ "handle": th.hex(), "message": format!("Parcel {}{number}", if building { "st. " } else { "" }) }))
}

/// Where along `path` (open) a label goes, and the direction there.
fn along(path: &[Vec2], at: Vec2) -> Option<(Vec2, f64)> {
    path.windows(2)
        .filter_map(|w| {
            let (a, b) = (*w.first()?, *w.get(1)?);
            let (dx, dy) = (b.x - a.x, b.y - a.y);
            let l2 = dx * dx + dy * dy;
            if l2 < 1e-18 {
                return None;
            }
            let t = (((at.x - a.x) * dx + (at.y - a.y) * dy) / l2).clamp(0.0, 1.0);
            let q = Vec2::new(a.x + t * dx, a.y + t * dy);
            Some(((q.x - at.x).hypot(q.y - at.y), q, dy.atan2(dx)))
        })
        .min_by(|x, y| x.0.total_cmp(&y.0))
        .map(|(_, q, a)| (q, a))
}

fn length(path: &[Vec2]) -> f64 {
    path.windows(2).map(|w| w.first().zip(w.get(1)).map_or(0.0, |(a, b)| (b.x - a.x).hypot(b.y - a.y))).sum()
}

/// The point `d` metres along `path`, and the direction there.
fn at_distance(path: &[Vec2], d: f64) -> Option<(Vec2, f64)> {
    let mut left = d;
    for w in path.windows(2) {
        let (a, b) = (*w.first()?, *w.get(1)?);
        let l = (b.x - a.x).hypot(b.y - a.y);
        if l < 1e-12 {
            continue;
        }
        if left <= l {
            let t = left / l;
            return Some((Vec2::new(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)), (b.y - a.y).atan2(b.x - a.x)));
        }
        left -= l;
    }
    let (a, b) = (path.get(path.len().checked_sub(2)?)?, path.last()?);
    Some((*b, (b.y - a.y).atan2(b.x - a.x)))
}

fn run_site(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "geopopissite";
    let h_line = one_target(s, p, "line", CMD)?;
    let space = s.space();
    let (mut path, closed) = path_of(s, h_line, &space).ok_or_else(|| bad(CMD, "pick a line or polyline"))?;
    if closed && let Some(f) = path.first().copied() {
        path.push(f);
    }
    let value = str_param(p, "text")
        .map(str::trim)
        .filter(|t| !t.is_empty() && t.chars().count() <= 200)
        .ok_or_else(|| bad(CMD, "`text`: the label is required"))?
        .to_string();
    let total = length(&path);
    if total < 1e-9 {
        return Err(bad(CMD, "the line has no length"));
    }
    let mut places: Vec<(Vec2, f64)> = Vec::new();
    match p.get("every") {
        Some(v) => {
            let step = v.as_f64().filter(|x| x.is_finite() && *x > 0.0).ok_or_else(|| bad(CMD, "`every` must be a distance in metres"))?;
            if total / step > MAX_LABELS as f64 {
                return Err(bad(CMD, format!("that makes more than {MAX_LABELS} labels")));
            }
            let mut d = step / 2.0;
            while d < total {
                places.extend(at_distance(&path, d));
                d += step;
            }
        }
        None => match p.get("at") {
            Some(_) => places.extend(along(&path, pt(p, "at", CMD)?)),
            None => places.extend(at_distance(&path, total / 2.0)),
        },
    }
    let h = notes_h(s);
    ensure_layer(s, LAYER_UTILITY, 7)?;
    for (q, a) in &places {
        let r = readable(*a);
        // Just above the line.
        let off = Vec2::new(-r.sin() * 0.6 * h, r.cos() * 0.6 * h);
        text(s, &space, LAYER_UTILITY, Vec2::new(q.x + off.x, q.y + off.y), h, value.clone(), r, (HAlign::Center, VAlign::Bottom))?;
    }
    Ok(json!({ "labels": places.len(), "message": format!("{} labels \"{value}\"", places.len()) }))
}

fn run_popis(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "geopopis";
    let (from, to) = (pt(p, "from", CMD)?, pt(p, "to", CMD)?);
    let value = str_param(p, "text")
        .map(str::trim)
        .filter(|t| !t.is_empty() && t.chars().count() <= 200)
        .ok_or_else(|| bad(CMD, "`text` is required"))?
        .to_string();
    let h = notes_h(s);
    ensure_layer(s, LAYER_NOTES, 7)?;
    let right = to.x >= from.x;
    let w = 0.75 * h * value.chars().count() as f64;
    let end = Vec2::new(if right { to.x + w } else { to.x - w }, to.y);
    line(s, LAYER_NOTES, from, to)?;
    line(s, LAYER_NOTES, to, end)?;
    let space = s.space();
    add(s, &space, LAYER_NOTES, EntityKind::Circle(Circle { center: Vec3::new(from.x, from.y, 0.0), radius: 0.15 * h }))?;
    let mid = Vec2::new((to.x + end.x) / 2.0, to.y + 0.3 * h);
    text(s, &space, LAYER_NOTES, mid, h, value.clone(), 0.0, (HAlign::Center, VAlign::Bottom))?;
    Ok(json!({ "message": format!("Label \"{value}\"") }))
}

/// "12,45" (Czech decimal comma).
fn cz(v: f64, decimals: usize) -> String {
    format!("{v:.decimals$}").replace('.', ",")
}

fn run_mistnost(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "geomistnost";
    let b = one_target(s, p, "boundary", CMD)?;
    let poly = outline(s, b, CMD)?;
    let area = signed_area(&poly).abs();
    let at = match p.get("at") {
        Some(_) => pt(p, "at", CMD)?,
        None => inner_point(&poly),
    };
    let field = |key: &str| str_param(p, key).map(str::trim).filter(|t| !t.is_empty()).map(|t| t.chars().take(100).collect::<String>());
    let height = match p.get("height") {
        None | Some(Value::Null) => None,
        Some(v) => Some(
            v.as_f64()
                .or_else(|| v.as_str().and_then(typed_number))
                .filter(|z| z.is_finite() && (0.0..1000.0).contains(z))
                .ok_or_else(|| bad(CMD, "`height` must be metres"))?,
        ),
    };
    let mut rows: Vec<(String, f64)> = Vec::new();
    let h = notes_h(s);
    if let Some(n) = field("name") {
        rows.push((n, h));
    }
    rows.push((format!("{} m²", cz(area, 2)), 0.8 * h));
    if let Some(f) = field("floor") {
        rows.push((f, 0.8 * h));
    }
    if let Some(z) = height {
        rows.push((format!("SV = {}", cz(z, 2)), 0.8 * h));
    }
    ensure_layer(s, LAYER_ROOMS, 7)?;
    let space = s.space();
    let total: f64 = rows.iter().map(|r| 1.5 * r.1).sum();
    let mut y = at.y + total / 2.0;
    for (value, th) in rows {
        y -= 1.5 * th;
        text(s, &space, LAYER_ROOMS, Vec2::new(at.x, y + 0.25 * th), th, value, 0.0, (HAlign::Center, VAlign::Baseline))?;
    }
    Ok(json!({ "area": area, "message": format!("Room: {} m²", cz(area, 2)) }))
}

fn run_schody(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "geoschody";
    let (a, b) = (pt(p, "from", CMD)?, pt(p, "to", CMD)?);
    let len = (b.x - a.x).hypot(b.y - a.y);
    if len < 1e-6 {
        return Err(bad(CMD, "the walking line has no length"));
    }
    let width = p
        .get("width")
        .and_then(|v| v.as_f64().or_else(|| v.as_str().and_then(typed_number)))
        .filter(|w| w.is_finite() && *w > 0.0 && *w < 1000.0)
        .ok_or_else(|| bad(CMD, "`width` must be metres"))?;
    let steps = p
        .get("steps")
        .and_then(|v| v.as_u64().or_else(|| v.as_str().and_then(|t| t.trim().parse().ok())))
        .filter(|n| (1..=500).contains(n))
        .ok_or_else(|| bad(CMD, "`steps` must be 1 to 500"))? as usize;
    let rise = match p.get("rise") {
        None | Some(Value::Null) => None,
        Some(v) => Some(
            v.as_f64()
                .or_else(|| v.as_str().and_then(typed_number))
                .filter(|z| z.is_finite() && *z > 0.0 && *z < 1000.0)
                .ok_or_else(|| bad(CMD, "`rise` must be metres"))?,
        ),
    };
    let u = Vec2::new((b.x - a.x) / len, (b.y - a.y) / len);
    let nrm = Vec2::new(-u.y * width / 2.0, u.x * width / 2.0);
    let at = |t: f64, side: f64| Vec2::new(a.x + u.x * len * t + nrm.x * side, a.y + u.y * len * t + nrm.y * side);
    ensure_layer(s, LAYER_STAIRS, 7)?;
    let space = s.space();
    let outline = LwPolyline {
        vertices: [at(0.0, -1.0), at(1.0, -1.0), at(1.0, 1.0), at(0.0, 1.0)].into_iter().map(PolyVertex::new).collect(),
        closed: true,
        const_width: 0.0,
        elevation: 0.0,
        plinegen: false,
    };
    add(s, &space, LAYER_STAIRS, EntityKind::LwPolyline(outline))?;
    for i in 1..steps {
        let t = i as f64 / steps as f64;
        line(s, LAYER_STAIRS, at(t, -1.0), at(t, 1.0))?;
    }
    // Walking line: a circle at its foot, an arrow at its head.
    let h = notes_h(s);
    let r = (0.15 * width).min(0.4 * h).max(1e-3);
    add(s, &space, LAYER_STAIRS, EntityKind::Circle(Circle { center: Vec3::new(a.x, a.y, 0.0), radius: r }))?;
    let head = (3.0 * r).min(0.3 * len);
    let base = Vec2::new(b.x - u.x * head, b.y - u.y * head);
    line(s, LAYER_STAIRS, Vec2::new(a.x + u.x * r, a.y + u.y * r), base)?;
    let arrow = LwPolyline {
        vertices: vec![PolyVertex { p: base, bulge: 0.0, start_width: 2.0 * r, end_width: 0.0 }, PolyVertex::new(b)],
        closed: false,
        const_width: 0.0,
        elevation: 0.0,
        plinegen: false,
    };
    add(s, &space, LAYER_STAIRS, EntityKind::LwPolyline(arrow))?;
    let going = len / steps as f64;
    let label = match rise {
        Some(z) => format!("{steps} × {:.0}/{:.0}", z / steps as f64 * 1000.0, going * 1000.0),
        None => format!("{steps} × {:.0}", going * 1000.0),
    };
    let ang = readable(u.y.atan2(u.x));
    let mid = at(0.5, 0.0);
    let off = Vec2::new(-ang.sin() * (width / 2.0 + 0.5 * h), ang.cos() * (width / 2.0 + 0.5 * h));
    text(s, &space, LAYER_STAIRS, Vec2::new(mid.x + off.x, mid.y + off.y), 0.8 * h, label.clone(), ang, (HAlign::Center, VAlign::Bottom))?;
    Ok(json!({ "going": going, "riser": rise.map(|z| z / steps as f64), "message": format!("Stairs {label} (mm)") }))
}

fn run_sipka(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "geosipka";
    let (a, b) = (pt(p, "from", CMD)?, pt(p, "to", CMD)?);
    let len = (b.x - a.x).hypot(b.y - a.y);
    if len < 1e-9 {
        return Err(bad(CMD, "the arrow has no length"));
    }
    let size = match p.get("size") {
        None | Some(Value::Null) => 1.2 * notes_h(s),
        Some(v) => v.as_f64().filter(|x| x.is_finite() && *x > 0.0 && *x < 1e6).ok_or_else(|| bad(CMD, "`size` must be metres"))?,
    }
    .min(0.5 * len);
    let u = Vec2::new((b.x - a.x) / len, (b.y - a.y) / len);
    let base = Vec2::new(b.x - u.x * size, b.y - u.y * size);
    ensure_layer(s, LAYER_ARROWS, 7)?;
    line(s, LAYER_ARROWS, a, base)?;
    let space = s.space();
    let head = LwPolyline {
        vertices: vec![PolyVertex { p: base, bulge: 0.0, start_width: 0.6 * size, end_width: 0.0 }, PolyVertex::new(b)],
        closed: false,
        const_width: 0.0,
        elevation: 0.0,
        plinegen: false,
    };
    add(s, &space, LAYER_ARROWS, EntityKind::LwPolyline(head))?;
    Ok(json!({ "message": "Arrow drawn" }))
}

fn text_of(i: &Input) -> Option<String> {
    match i {
        Input::Text(t) | Input::Keyword(t) => Some(t.trim().to_string()),
        _ => None,
    }
}

/// GEOPARCELA: boundary, number, building or not; again until Enter.
#[derive(Default)]
struct ParcM {
    boundary: Option<Handle>,
    number: Option<String>,
}

impl Interactive for ParcM {
    fn name(&self) -> &'static str {
        "GEOPARCELA"
    }
    fn prompt(&self, _s: &Session) -> Prompt {
        match (&self.boundary, &self.number) {
            (None, _) => Prompt::new("Select the parcel boundary", Accept::SELECT),
            (Some(_), None) => Prompt::new("Parcel number", Accept::TEXT),
            _ => Prompt::new("Building parcel?", Accept::TEXT).kw(&["Yes", "No"]).default("No"),
        }
    }
    fn input(&mut self, s: &mut Session, i: Input) -> Result<Step> {
        if matches!(i, Input::Cancel) {
            return Ok(Step::Cancel);
        }
        match (&self.boundary, &self.number) {
            (None, _) => match i {
                Input::Pick(hs) => self.boundary = hs.first().copied(),
                Input::Enter => return Ok(Step::Done),
                _ => {}
            },
            (Some(_), None) => match text_of(&i) {
                Some(t) if !t.is_empty() => self.number = Some(t),
                _ => s.echo("Type the parcel number, like 123/4."),
            },
            (Some(b), Some(n)) => {
                let yes = text_of(&i).and_then(|t| self.prompt(s).match_keyword(&t)).as_deref() == Some("Yes");
                let r = run_parcela(s, &json!({ "boundary": b.hex(), "number": n, "building": yes }));
                say(s, r);
                *self = ParcM::default();
            }
        }
        Ok(Step::Continue)
    }
}

/// GEOPOPISSITE: the line, the label, then places on it until Enter.
#[derive(Default)]
struct SiteM {
    line: Option<Handle>,
    label: Option<String>,
}

impl Interactive for SiteM {
    fn name(&self) -> &'static str {
        "GEOPOPISSITE"
    }
    fn prompt(&self, _s: &Session) -> Prompt {
        match (&self.line, &self.label) {
            (None, _) => Prompt::new("Select the utility line", Accept::SELECT),
            (Some(_), None) => Prompt::new("Label text (e.g. V DN150)", Accept::TEXT),
            _ => Prompt::new("Pick where on the line (Enter: done)", Accept::POINT),
        }
    }
    fn input(&mut self, s: &mut Session, i: Input) -> Result<Step> {
        if matches!(i, Input::Cancel) {
            return Ok(Step::Cancel);
        }
        match (&self.line, &self.label) {
            (None, _) => match i {
                Input::Pick(hs) => self.line = hs.first().copied(),
                Input::Enter => return Ok(Step::Done),
                _ => {}
            },
            (Some(_), None) => match text_of(&i) {
                Some(t) if !t.is_empty() => self.label = Some(t),
                _ => s.echo("Type the label."),
            },
            (Some(l), Some(t)) => match i {
                Input::Point(q) => {
                    let r = run_site(s, &json!({ "line": l.hex(), "text": t, "at": [q.x, q.y] }));
                    say(s, r);
                }
                Input::Enter => return Ok(Step::Done),
                _ => {}
            },
        }
        Ok(Step::Continue)
    }
}

/// GEOPOPIS: what, where, the text; again until Enter.
#[derive(Default)]
struct PopisM {
    from: Option<Vec2>,
    to: Option<Vec2>,
}

impl Interactive for PopisM {
    fn name(&self) -> &'static str {
        "GEOPOPIS"
    }
    fn prompt(&self, _s: &Session) -> Prompt {
        match (self.from, self.to) {
            (None, _) => Prompt::new("Pick what to describe", Accept::POINT),
            (Some(f), None) => Prompt::new("Pick where the text goes", Accept::POINT).base(f),
            _ => Prompt::new("Text", Accept::TEXT),
        }
    }
    fn input(&mut self, s: &mut Session, i: Input) -> Result<Step> {
        match (self.from, self.to, i) {
            (_, _, Input::Cancel) => Ok(Step::Cancel),
            (None, _, Input::Enter) => Ok(Step::Done),
            (None, _, Input::Point(q)) => {
                self.from = Some(q);
                Ok(Step::Continue)
            }
            (Some(_), None, Input::Point(q)) => {
                self.to = Some(q);
                Ok(Step::Continue)
            }
            (Some(f), Some(t), Input::Text(v) | Input::Keyword(v)) => {
                let r = run_popis(s, &json!({ "from": [f.x, f.y], "to": [t.x, t.y], "text": v }));
                say(s, r);
                *self = PopisM::default();
                Ok(Step::Continue)
            }
            _ => Ok(Step::Continue),
        }
    }
}

/// GEOMISTNOST: the room outline, name, floor, clear height.
#[derive(Default)]
struct RoomM {
    boundary: Option<Handle>,
    answers: Vec<String>,
}

impl Interactive for RoomM {
    fn name(&self) -> &'static str {
        "GEOMISTNOST"
    }
    fn prompt(&self, _s: &Session) -> Prompt {
        match (self.boundary, self.answers.len()) {
            (None, _) => Prompt::new("Select the room outline", Accept::SELECT),
            (Some(_), 0) => Prompt::new("Room number and name (Enter: none)", Accept::TEXT),
            (Some(_), 1) => Prompt::new("Floor finish (Enter: none)", Accept::TEXT),
            _ => Prompt::new("Clear height (Enter: none)", Accept::NUMBER),
        }
    }
    fn input(&mut self, s: &mut Session, i: Input) -> Result<Step> {
        if matches!(i, Input::Cancel) {
            return Ok(Step::Cancel);
        }
        let Some(b) = self.boundary else {
            match i {
                Input::Pick(hs) => self.boundary = hs.first().copied(),
                Input::Enter => return Ok(Step::Done),
                _ => {}
            }
            return Ok(Step::Continue);
        };
        let t = match &i {
            Input::Enter => String::new(),
            other => match text_of(other) {
                Some(t) => t,
                None => return Ok(Step::Continue),
            },
        };
        self.answers.push(t);
        if self.answers.len() < 3 {
            return Ok(Step::Continue);
        }
        let get = |k: usize| self.answers.get(k).filter(|t| !t.is_empty()).cloned();
        let r = run_mistnost(s, &json!({ "boundary": b.hex(), "name": get(0), "floor": get(1), "height": get(2) }));
        say(s, r);
        *self = RoomM::default();
        Ok(Step::Continue)
    }
}

/// GEOSCHODY: foot and head of the walking line, width, steps, rise.
#[derive(Default)]
struct StairsM {
    from: Option<Vec2>,
    to: Option<Vec2>,
    answers: Vec<String>,
}

impl Interactive for StairsM {
    fn name(&self) -> &'static str {
        "GEOSCHODY"
    }
    fn prompt(&self, _s: &Session) -> Prompt {
        match (self.from, self.to, self.answers.len()) {
            (None, ..) => Prompt::new("Pick the foot of the walking line", Accept::POINT),
            (Some(f), None, _) => Prompt::new("Pick the head of the walking line", Accept::POINT).base(f),
            (_, _, 0) => Prompt::new("Stair width (m)", Accept::NUMBER).default("1.0"),
            (_, _, 1) => Prompt::new("Number of steps", Accept::NUMBER),
            _ => Prompt::new("Total rise (m, Enter: none)", Accept::NUMBER),
        }
    }
    fn input(&mut self, s: &mut Session, i: Input) -> Result<Step> {
        if matches!(i, Input::Cancel) {
            return Ok(Step::Cancel);
        }
        match (self.from, self.to, i) {
            (None, _, Input::Point(q)) => self.from = Some(q),
            (Some(_), None, Input::Point(q)) => self.to = Some(q),
            (Some(f), Some(t), i) => {
                let v = match &i {
                    Input::Enter => self.prompt(s).default.unwrap_or_default(),
                    other => match text_of(other) {
                        Some(v) => v,
                        None => return Ok(Step::Continue),
                    },
                };
                if self.answers.len() == 1 && v.is_empty() {
                    s.echo("Type the number of steps.");
                    return Ok(Step::Continue);
                }
                self.answers.push(v);
                if self.answers.len() == 3 {
                    let rise = self.answers.get(2).filter(|t| !t.is_empty()).cloned();
                    let r = run_schody(
                        s,
                        &json!({ "from": [f.x, f.y], "to": [t.x, t.y], "width": self.answers.first(), "steps": self.answers.get(1), "rise": rise }),
                    );
                    say(s, r);
                    return Ok(Step::Done);
                }
            }
            _ => {}
        }
        Ok(Step::Continue)
    }
}

/// GEOSIPKA: tail and tip; again until Enter.
#[derive(Default)]
struct ArrowM {
    from: Option<Vec2>,
}

impl Interactive for ArrowM {
    fn name(&self) -> &'static str {
        "GEOSIPKA"
    }
    fn prompt(&self, _s: &Session) -> Prompt {
        match self.from {
            None => Prompt::new("Pick the arrow's tail", Accept::POINT),
            Some(f) => Prompt::new("Pick the arrow's tip", Accept::POINT).base(f),
        }
    }
    fn input(&mut self, s: &mut Session, i: Input) -> Result<Step> {
        match (self.from, i) {
            (_, Input::Cancel) => Ok(Step::Cancel),
            (None, Input::Enter) => Ok(Step::Done),
            (None, Input::Point(q)) => {
                self.from = Some(q);
                Ok(Step::Continue)
            }
            (Some(f), Input::Point(q)) => {
                let r = run_sipka(s, &json!({ "from": [f.x, f.y], "to": [q.x, q.y] }));
                say(s, r);
                self.from = None;
                Ok(Step::Continue)
            }
            _ => Ok(Step::Continue),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn square(s: &mut Session, w: f64) -> Handle {
        let space = s.space();
        let pl = LwPolyline {
            vertices: [(0.0, 0.0), (w, 0.0), (w, w), (0.0, w)].iter().map(|(x, y)| PolyVertex::new(Vec2::new(*x, *y))).collect(),
            closed: true,
            const_width: 0.0,
            elevation: 0.0,
            plinegen: false,
        };
        add(s, &space, "0", EntityKind::LwPolyline(pl)).unwrap()
    }

    fn texts(s: &Session, layer: &str) -> Vec<cadcraft_doc::Text> {
        s.doc()
            .unwrap()
            .model
            .iter()
            .filter(|e| e.common.layer == layer)
            .filter_map(|e| if let EntityKind::Text(t) = &e.kind { Some(t.clone()) } else { None })
            .collect()
    }

    #[test]
    fn parcel_and_room_labels_sit_inside() {
        let mut s = Session::new();
        let b = square(&mut s, 10.0);
        s.execute("geoparcela", &json!({ "boundary": b.hex(), "number": "123/4", "building": true })).unwrap();
        let t = &texts(&s, LAYER_PARCELS)[0];
        assert_eq!(t.value, "123/4");
        assert!((t.align_pt.unwrap().x - 5.0).abs() < 1e-9);
        assert_eq!(s.doc().unwrap().model.iter().filter(|e| e.common.layer == LAYER_PARCELS).count(), 2);
        let r = s.execute("geomistnost", &json!({ "boundary": b.hex(), "name": "1.01 Kuchyně", "height": 2.65 })).unwrap();
        assert!((r["area"].as_f64().unwrap() - 100.0).abs() < 1e-9);
        let rows: Vec<String> = texts(&s, LAYER_ROOMS).into_iter().map(|t| t.value).collect();
        assert_eq!(rows, ["1.01 Kuchyně", "100,00 m²", "SV = 2,65"]);
        // An L-shaped room: the label still lands inside.
        let space = s.space();
        let l = LwPolyline {
            vertices: [(0.0, 0.0), (10.0, 0.0), (10.0, 1.0), (1.0, 1.0), (1.0, 10.0), (0.0, 10.0)]
                .iter()
                .map(|(x, y)| PolyVertex::new(Vec2::new(*x, *y)))
                .collect(),
            closed: true,
            const_width: 0.0,
            elevation: 0.0,
            plinegen: false,
        };
        let poly: Vec<Vec2> = l.vertices.iter().map(|v| v.p).collect();
        add(&mut s, &space, "0", EntityKind::LwPolyline(l)).unwrap();
        assert!(inside(inner_point(&poly), &poly));
    }

    #[test]
    fn utility_labels_along_the_line() {
        let mut s = Session::new();
        let space = s.space();
        let l = add(&mut s, &space, "0", EntityKind::Line(Line { a: Vec3::new(10.0, 0.0, 0.0), b: Vec3::new(0.0, 0.0, 0.0) })).unwrap();
        let r = s.execute("geopopissite", &json!({ "line": l.hex(), "text": "V DN150", "every": 4 })).unwrap();
        assert_eq!(r["labels"], 2);
        // Drawn right to left, read left to right, above the line.
        let t = &texts(&s, LAYER_UTILITY)[0];
        assert!(t.rotation.abs() < 1e-9 && t.align_pt.unwrap().y > 0.0);
        s.execute("geopopis", &json!({ "from": [0, 0], "to": [5, 5], "text": "plot" })).unwrap();
        assert_eq!(texts(&s, LAYER_NOTES)[0].value, "plot");
    }

    #[test]
    fn stairs_and_arrows() {
        let mut s = Session::new();
        let r = s.execute("geoschody", &json!({ "from": [0, 0], "to": [2.8, 0], "width": 1.0, "steps": 10, "rise": 1.75 })).unwrap();
        assert!((r["going"].as_f64().unwrap() - 0.28).abs() < 1e-9);
        assert_eq!(texts(&s, LAYER_STAIRS)[0].value, "10 × 175/280");
        let treads = s.doc().unwrap().model.iter().filter(|e| e.common.layer == LAYER_STAIRS && matches!(e.kind, EntityKind::Line(_))).count();
        assert_eq!(treads, 9 + 1);
        s.execute("geosipka", &json!({ "from": [0, 0], "to": [0, 5] })).unwrap();
        assert_eq!(s.doc().unwrap().model.iter().filter(|e| e.common.layer == LAYER_ARROWS).count(), 2);
    }

    #[test]
    fn hostile_input_is_refused_not_panicking() {
        let mut s = Session::new();
        let b = square(&mut s, 1.0);
        let space = s.space();
        let l = add(&mut s, &space, "0", EntityKind::Line(Line { a: Vec3::new(0.0, 0.0, 0.0), b: Vec3::new(0.0, 0.0, 0.0) })).unwrap();
        for (c, p) in [
            ("geoparcela", json!({ "boundary": b.hex() })),
            ("geoparcela", json!({ "boundary": l.hex(), "number": "1" })),
            ("geopopissite", json!({ "line": l.hex(), "text": "x" })),
            ("geopopissite", json!({ "line": b.hex(), "text": "x", "every": 1e-12 })),
            ("geopopis", json!({ "from": [0, 0], "to": [1, 1] })),
            ("geomistnost", json!({ "boundary": b.hex(), "height": "abc" })),
            ("geoschody", json!({ "from": [0, 0], "to": [0, 0], "width": 1, "steps": 3 })),
            ("geoschody", json!({ "from": [0, 0], "to": [1, 0], "width": 1, "steps": 100000 })),
            ("geosipka", json!({ "from": [0, 0], "to": [0, 0] })),
        ] {
            assert!(s.execute(c, &p).is_err(), "{c} {p}");
        }
    }
}
