//! Survey add-on: the rest of the building survey and drawing set-up.
//!
//! - GEONIKA draws a wall niche in plan: the recess behind the wall face, with its depth (and
//!   height) written in it.
//! - GEOKROV draws a timber element of a roof truss: along its axis as a member of the given
//!   section, or, in section, its cross-section with both diagonals.
//! - GEOAXO draws a 3D view of the survey: points, 3D lines and the terrain model projected
//!   axonometrically (from a direction and an angle above the horizon) beside the drawing, so a
//!   3D look is a drawing that can be printed.
//! - GEOPASPORT lists the rooms labelled with GEOMISTNOST: names, areas, floors and clear
//!   heights, with the total, as a table in the drawing and, if asked, a CSV file.
//! - GEOHLADINY saves the drawing's layers (colour, linetype, lineweight, on, frozen, locked,
//!   plot) to a template file, or makes the layers of a template in this drawing.
//! - GEONOVY starts a new drawing set up for survey work: optionally a settings template
//!   (GEONASTAVENI) and a layer template (GEOHLADINY).

use cadcraft_doc::{EntityKind, HAlign, Layer, Line, LwPolyline, Point, VAlign};
use cadcraft_geom::{PolyVertex, Vec2, Vec3};
use serde_json::{Value, json};

use super::geo::LAYER_POINTS;
use super::geo_calc::{add, bbox, ensure_layer, say, text, typed_number};
use super::geo_nastaveni as nast;
use super::geo_popis::LAYER_ROOMS;
use super::{CommandSpec, bad, point_value, str_param, targets};
use crate::{Accept, Input, Interactive, Prompt, Result, Session, Step};

pub const LAYER_NICHES: &str = "GEO_niky";
pub const LAYER_TRUSS: &str = "GEO_krov";
pub const LAYER_3D: &str = "GEO_3D";
pub const LAYER_PASSPORT: &str = "GEO_pasport";
/// Objects drawn in one 3D view, at most.
const MAX_AXO: usize = 500_000;
/// Largest layer template read.
const MAX_TEMPLATE: u64 = 4 << 20;

pub(super) fn specs() -> Vec<CommandSpec> {
    vec![
        CommandSpec::new("geonika", "Wall Niche...", run_nika)
            .menu(&["Geo", "Building", "Wall Niche..."])
            .alias(&["geoniche"])
            .params("{from: [x, y], to: [x, y] the niche's ends on the wall face, depth: m, toward: [x, y] a point inside the wall, height?: m} → {message}")
            .interactive(|_| Ok(Box::new(StepsM::new("GEONIKA", "geonika", &[("from", "Pick one end of the niche on the wall face", 'p'), ("to", "Pick the other end", 'p'), ("toward", "Pick a point inside the wall", 'p'), ("depth", "Niche depth (m)", 'n'), ("height", "Niche height (m, Enter: none)", 'o')])))),
        CommandSpec::new("geokrov", "Roof Truss Element...", run_krov)
            .menu(&["Geo", "Building", "Roof Truss Element..."])
            .alias(&["geotimber", "geotruss"])
            .params("{from: [x, y], to: [x, y] the element's axis, width: m, height: m — or, in section: at: [x, y], width, height, angle?: degrees} → {message}")
            .interactive(|_| Ok(Box::new(StepsM::new("GEOKROV", "geokrov", &[("from", "Pick the start of the element's axis", 'p'), ("to", "Pick its end", 'p'), ("width", "Element width (m)", 'n'), ("height", "Element height (m)", 'n')])))),
        CommandSpec::new("geoaxo", "3D View of the Survey...", run_axo)
            .menu(&["Geo", "Terrain", "3D View of the Survey..."])
            .alias(&["geo3d", "geo3dpohled"])
            .params("{handles?: objects (default every survey point, 3D line and terrain triangle), azimuth?: direction looked from, degrees counter-clockwise from east (default 225, south-west), elevation?: degrees above the horizon (default 30), zscale?: height exaggeration (default 1), at?: [x, y] where the view goes (default right of the drawing)} → {objects, message}")
            .interactive(|_| Ok(Box::new(StepsM::new("GEOAXO", "geoaxo", &[("elevation", "Angle above the horizon (degrees)", 'd'), ("zscale", "Height exaggeration", 'd')])))),
        CommandSpec::new("geopasport", "Room Schedule...", run_pasport)
            .menu(&["Geo", "Building", "Room Schedule..."])
            .alias(&["geopassport", "georooms"])
            .params("{at?: [x, y] where the table goes, csv?: file path} → {rooms: [{name, area, floor, height}], total, message}")
            .interactive(|_| Ok(Box::new(StepsM::new("GEOPASPORT", "geopasport", &[("at", "Pick where the table goes", 'p')])))),
        CommandSpec::new("geohladiny", "Layer Template...", run_hladiny)
            .menu(&["Geo", "Tools", "Layer Template..."])
            .alias(&["geolayers"])
            .params("{save?: file path | load?: file path, only?: name prefix (default all layers)} → {layers, message}")
            .interactive(|_| Ok(Box::new(PathsM { name: "GEOHLADINY", answers: Vec::new() }))),
        CommandSpec::new("geonovy", "New Survey Drawing...", run_novy)
            .menu(&["Geo", "New Survey Drawing..."])
            .alias(&["geonew"])
            .params("{settings?: GEONASTAVENI template path, layers?: GEOHLADINY template path, scale?: 1:N} → {message}")
            .interactive(|_| Ok(Box::new(PathsM { name: "GEONOVY", answers: Vec::new() }))),
    ]
}

fn pt(p: &Value, key: &str, cmd: &str) -> Result<Vec2> {
    p.get(key).and_then(point_value).filter(|v| v.x.abs() < 1e9 && v.y.abs() < 1e9).ok_or_else(|| bad(cmd, format!("`{key}`: [x, y] is required")))
}

fn num(p: &Value, key: &str, cmd: &str, range: std::ops::RangeInclusive<f64>) -> Result<Option<f64>> {
    match p.get(key) {
        None | Some(Value::Null) => Ok(None),
        Some(Value::String(t)) if t.trim().is_empty() => Ok(None),
        Some(v) => v
            .as_f64()
            .or_else(|| v.as_str().and_then(typed_number))
            .filter(|x| x.is_finite() && range.contains(x))
            .map(Some)
            .ok_or_else(|| bad(cmd, format!("`{key}` must be a number from {} to {}", range.start(), range.end()))),
    }
}

fn need(p: &Value, key: &str, cmd: &str, range: std::ops::RangeInclusive<f64>) -> Result<f64> {
    num(p, key, cmd, range)?.ok_or_else(|| bad(cmd, format!("`{key}` is required")))
}

fn notes_h(s: &Session) -> f64 {
    let n = nast::load(s);
    n.height(&n.notes)
}

fn pl(v: &[Vec2], closed: bool) -> EntityKind {
    EntityKind::LwPolyline(LwPolyline {
        vertices: v.iter().copied().map(PolyVertex::new).collect(),
        closed,
        const_width: 0.0,
        elevation: 0.0,
        plinegen: false,
    })
}

fn mm(v: f64) -> String {
    format!("{:.0}", v * 1000.0)
}

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

fn run_nika(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "geonika";
    let (a, b, q) = (pt(p, "from", CMD)?, pt(p, "to", CMD)?, pt(p, "toward", CMD)?);
    let w = (b.x - a.x).hypot(b.y - a.y);
    if w < 1e-6 {
        return Err(bad(CMD, "the niche has no width"));
    }
    let depth = need(p, "depth", CMD, 0.001..=10.0)?;
    let height = num(p, "height", CMD, 0.001..=100.0)?;
    let u = Vec2::new((b.x - a.x) / w, (b.y - a.y) / w);
    let mut n = Vec2::new(-u.y, u.x);
    if (q.x - a.x) * n.x + (q.y - a.y) * n.y < 0.0 {
        n = Vec2::new(-n.x, -n.y);
    }
    let back = |p: Vec2| Vec2::new(p.x + n.x * depth, p.y + n.y * depth);
    ensure_layer(s, LAYER_NICHES, 7)?;
    let space = s.space();
    add(s, &space, LAYER_NICHES, pl(&[a, back(a), back(b), b], false))?;
    let label = match height {
        Some(h) => format!("nika {}/{}", mm(depth), mm(h)),
        None => format!("nika {}", mm(depth)),
    };
    let th = 0.8 * notes_h(s);
    let mid = Vec2::new((a.x + b.x) / 2.0 + n.x * depth / 2.0, (a.y + b.y) / 2.0 + n.y * depth / 2.0);
    text(s, &space, LAYER_NICHES, mid, th.min(0.8 * depth.max(1e-3)), label.clone(), readable(u.y.atan2(u.x)), (HAlign::Center, VAlign::Middle))?;
    Ok(json!({ "message": format!("Niche {label}") }))
}

fn run_krov(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "geokrov";
    let b = need(p, "width", CMD, 0.001..=10.0)?;
    let h = need(p, "height", CMD, 0.001..=10.0)?;
    let label = format!("{}/{}", mm(b), mm(h));
    ensure_layer(s, LAYER_TRUSS, 30)?;
    let space = s.space();
    let th = 0.8 * notes_h(s);
    if p.get("at").is_some() {
        // A cross-section: the rectangle with both diagonals.
        let c = pt(p, "at", CMD)?;
        let ang = num(p, "angle", CMD, -360.0..=360.0)?.unwrap_or(0.0).to_radians();
        let (cs, sn) = (ang.cos(), ang.sin());
        let at = |x: f64, y: f64| Vec2::new(c.x + x * cs - y * sn, c.y + x * sn + y * cs);
        let k = [at(-b / 2.0, -h / 2.0), at(b / 2.0, -h / 2.0), at(b / 2.0, h / 2.0), at(-b / 2.0, h / 2.0)];
        add(s, &space, LAYER_TRUSS, pl(&k, true))?;
        for (x, y) in [(k[0], k[2]), (k[1], k[3])] {
            add(s, &space, LAYER_TRUSS, EntityKind::Line(Line { a: Vec3::new(x.x, x.y, 0.0), b: Vec3::new(y.x, y.y, 0.0) }))?;
        }
        let r = readable(ang);
        let off = h / 2.0 + 0.5 * th;
        text(s, &space, LAYER_TRUSS, Vec2::new(c.x - r.sin() * off, c.y + r.cos() * off), th, label.clone(), r, (HAlign::Center, VAlign::Bottom))?;
        return Ok(json!({ "message": format!("Timber section {label}") }));
    }
    let (a, e) = (pt(p, "from", CMD)?, pt(p, "to", CMD)?);
    let l = (e.x - a.x).hypot(e.y - a.y);
    if l < 1e-6 {
        return Err(bad(CMD, "the element has no length"));
    }
    let n = Vec2::new(-(e.y - a.y) / l * h / 2.0, (e.x - a.x) / l * h / 2.0);
    let k = [Vec2::new(a.x + n.x, a.y + n.y), Vec2::new(e.x + n.x, e.y + n.y), Vec2::new(e.x - n.x, e.y - n.y), Vec2::new(a.x - n.x, a.y - n.y)];
    add(s, &space, LAYER_TRUSS, pl(&k, true))?;
    let r = readable((e.y - a.y).atan2(e.x - a.x));
    let off = h / 2.0 + 0.5 * th;
    let m = Vec2::new((a.x + e.x) / 2.0, (a.y + e.y) / 2.0);
    text(s, &space, LAYER_TRUSS, Vec2::new(m.x - r.sin() * off, m.y + r.cos() * off), th, label.clone(), r, (HAlign::Center, VAlign::Bottom))?;
    Ok(json!({ "message": format!("Timber element {label}, {l:.2} m") }))
}

fn run_axo(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "geoaxo";
    let az = num(p, "azimuth", CMD, -360.0..=360.0)?.unwrap_or(225.0).to_radians();
    let el = num(p, "elevation", CMD, 0.0..=90.0)?.unwrap_or(30.0).to_radians();
    let zs = num(p, "zscale", CMD, 0.0..=1000.0)?.unwrap_or(1.0);
    let space = s.space();
    let chosen = targets(s, p)?;
    // Each object as its 3D polylines (closed or not) and points.
    let mut lines: Vec<(Vec<Vec3>, bool)> = Vec::new();
    let mut points: Vec<Vec3> = Vec::new();
    {
        let d = s.doc()?;
        let st = d.space(&space).ok_or_else(|| bad(CMD, "no drawing space"))?;
        let pick: Vec<&cadcraft_doc::Entity> = if chosen.is_empty() {
            st.iter().map(|e| e.as_ref()).collect()
        } else {
            chosen.iter().filter_map(|h| st.get(*h)).map(|e| e.as_ref()).collect()
        };
        for e in pick {
            if lines.len() + points.len() > MAX_AXO {
                return Err(bad(CMD, format!("more than {MAX_AXO} objects")));
            }
            match &e.kind {
                EntityKind::Point(q) if !chosen.is_empty() || e.common.layer.eq_ignore_ascii_case(LAYER_POINTS) => points.push(q.p),
                EntityKind::Polyline3d(pl) => lines.push((pl.points.clone(), pl.closed)),
                EntityKind::Face3d(f) => lines.push((f.corners.to_vec(), true)),
                EntityKind::Line(l) if !chosen.is_empty() || l.a.z != 0.0 || l.b.z != 0.0 => lines.push((vec![l.a, l.b], false)),
                EntityKind::LwPolyline(pl) if !chosen.is_empty() || pl.elevation != 0.0 => {
                    lines.push((pl.vertices.iter().map(|v| Vec3::new(v.p.x, v.p.y, pl.elevation)).collect(), pl.closed))
                }
                _ => {}
            }
        }
    }
    let all: Vec<Vec3> = points
        .iter()
        .copied()
        .chain(lines.iter().flat_map(|l| l.0.iter().copied()))
        .filter(|q| q.x.is_finite() && q.y.is_finite() && q.z.is_finite())
        .collect();
    if all.is_empty() {
        return Err(bad(CMD, "nothing in 3D to show: no survey points, 3D lines or terrain model"));
    }
    let (lo, hi) = bbox(&all.iter().map(|q| q.xy()).collect::<Vec<_>>());
    let c = Vec2::new((lo.x + hi.x) / 2.0, (lo.y + hi.y) / 2.0);
    let z0 = all.iter().map(|q| q.z).fold(f64::INFINITY, f64::min);
    // Looking from `az`: the view's "into the screen" direction is the opposite, so the plan is
    // turned until the viewer stands below it (south of it, looking north).
    let rot = -(az + std::f64::consts::FRAC_PI_2);
    let (cs, sn) = (rot.cos(), rot.sin());
    let proj = |q: Vec3| {
        let (x, y) = (q.x - c.x, q.y - c.y);
        let (xr, yr) = (x * cs - y * sn, x * sn + y * cs);
        Vec2::new(xr, yr * el.sin() + (q.z - z0) * zs * el.cos())
    };
    let shown: Vec<Vec2> = all.iter().map(|q| proj(*q)).collect();
    let (plo, phi) = bbox(&shown);
    let at = match p.get("at") {
        Some(_) => pt(p, "at", CMD)?,
        None => Vec2::new(hi.x + 0.2 * (hi.x - lo.x).max(1.0) + (phi.x - plo.x) / 2.0, (lo.y + hi.y) / 2.0),
    };
    let shift = Vec2::new(at.x - (plo.x + phi.x) / 2.0, at.y - (plo.y + phi.y) / 2.0);
    let place = |q: Vec3| {
        let v = proj(q);
        Vec2::new(v.x + shift.x, v.y + shift.y)
    };
    ensure_layer(s, LAYER_3D, 4)?;
    let mut n = 0;
    for (v, closed) in &lines {
        let v: Vec<Vec2> = v.iter().filter(|q| q.x.is_finite() && q.y.is_finite() && q.z.is_finite()).map(|q| place(*q)).collect();
        if v.len() >= 2 {
            add(s, &space, LAYER_3D, pl(&v, *closed))?;
            n += 1;
        }
    }
    for q in points.iter().filter(|q| q.x.is_finite() && q.y.is_finite() && q.z.is_finite()) {
        let v = place(*q);
        add(s, &space, LAYER_3D, EntityKind::Point(Point { p: Vec3::new(v.x, v.y, 0.0), angle: 0.0 }))?;
        n += 1;
    }
    Ok(
        json!({ "objects": n, "message": format!("3D view of {n} objects (from {:.0}°, {:.0}° above the horizon, heights ×{zs})", az.to_degrees(), el.to_degrees()) }),
    )
}

/// "12,45 m²" → 12.45.
fn area_of(t: &str) -> Option<f64> {
    typed_number(t.trim().strip_suffix("m²")?.trim())
}

fn run_pasport(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "geopasport";
    let space = s.space();
    let labels: Vec<(Vec2, f64, String)> = {
        let d = s.doc()?;
        let st = d.space(&space).ok_or_else(|| bad(CMD, "no drawing space"))?;
        st.iter()
            .filter(|e| e.common.layer.eq_ignore_ascii_case(LAYER_ROOMS))
            .filter_map(|e| {
                if let EntityKind::Text(t) = &e.kind {
                    Some((t.align_pt.unwrap_or(t.insert).xy(), t.height, t.value.trim().to_string()))
                } else {
                    None
                }
            })
            .filter(|l| l.0.x.is_finite() && l.0.y.is_finite() && l.1.is_finite() && l.1 > 0.0)
            .collect()
    };
    // A room is its area row; the row just above it is its name, the rows just below its floor
    // and clear height (GEOMISTNOST writes them centred, one under another).
    let row = |at: Vec2, h: f64, below: bool| {
        labels
            .iter()
            .filter(|l| {
                (l.0.x - at.x).abs() < 1e-6 * at.x.abs().max(1.0) && {
                    let dy = if below { at.y - l.0.y } else { l.0.y - at.y };
                    dy > 0.1 * h && dy < 2.2 * h.max(l.1)
                }
            })
            .min_by(|a, b| (a.0.y - at.y).abs().total_cmp(&(b.0.y - at.y).abs()))
    };
    let mut rooms: Vec<(String, f64, String, String)> = Vec::new();
    for (at, h, v) in &labels {
        let Some(area) = area_of(v) else { continue };
        let name = row(*at, *h, false).map(|l| l.2.clone()).unwrap_or_default();
        let mut floor = String::new();
        let mut sv = String::new();
        if let Some(b) = row(*at, *h, true) {
            if let Some(x) = b.2.strip_prefix("SV = ") {
                sv = x.to_string();
            } else {
                floor = b.2.clone();
                if let Some(c) = row(b.0, b.1, true)
                    && let Some(x) = c.2.strip_prefix("SV = ")
                {
                    sv = x.to_string();
                }
            }
        }
        rooms.push((name, area, floor, sv));
    }
    if rooms.is_empty() {
        return Err(bad(CMD, "no room labels (GEOMISTNOST) in the drawing"));
    }
    rooms.sort_by(|a, b| a.0.cmp(&b.0));
    let total: f64 = rooms.iter().map(|r| r.1).sum();
    let cz = |v: f64| format!("{v:.2}").replace('.', ",");
    let mut csv = String::from("místnost;plocha [m2];podlaha;světlá výška [m]\n");
    for (n, a, f, h) in &rooms {
        csv.push_str(&format!("{n};{};{f};{h}\n", cz(*a)));
    }
    csv.push_str(&format!("celkem;{};;\n", cz(total)));
    if let Some(path) = str_param(p, "csv") {
        write_file(path, &csv, CMD)?;
    }
    if p.get("at").is_some() {
        let at = pt(p, "at", CMD)?;
        let h = notes_h(s);
        ensure_layer(s, LAYER_PASSPORT, 7)?;
        for (i, line) in csv.lines().enumerate() {
            let y = at.y - 1.6 * h * i as f64;
            for (k, cell) in line.split(';').enumerate() {
                let x = at.x + [0.0, 16.0, 26.0, 40.0].get(k).copied().unwrap_or(0.0) * h;
                if !cell.is_empty() {
                    text(s, &space, LAYER_PASSPORT, Vec2::new(x, y), h, cell.to_string(), 0.0, (HAlign::Left, VAlign::Baseline))?;
                }
            }
        }
    }
    let list: Vec<Value> = rooms.iter().map(|(n, a, f, h)| json!({ "name": n, "area": a, "floor": f, "height": h })).collect();
    Ok(json!({ "rooms": list, "total": total, "message": format!("{} rooms, {} m² in all", rooms.len(), cz(total)) }))
}

#[cfg(not(target_arch = "wasm32"))]
fn write_file(path: &str, text: &str, cmd: &str) -> Result<()> {
    let mut bytes = b"\xEF\xBB\xBF".to_vec();
    bytes.extend_from_slice(text.as_bytes());
    std::fs::write(path, bytes).map_err(|e| bad(cmd, format!("{path}: {e}")))
}

#[cfg(target_arch = "wasm32")]
fn write_file(_path: &str, _text: &str, cmd: &str) -> Result<()> {
    Err(bad(cmd, "files are not available on the web"))
}

#[cfg(not(target_arch = "wasm32"))]
fn write_plain(path: &str, text: &str, cmd: &str) -> Result<()> {
    std::fs::write(path, text).map_err(|e| bad(cmd, format!("{path}: {e}")))
}

#[cfg(target_arch = "wasm32")]
fn write_plain(_path: &str, _text: &str, cmd: &str) -> Result<()> {
    Err(bad(cmd, "templates are not available on the web"))
}

#[cfg(not(target_arch = "wasm32"))]
fn read_layers(path: &str) -> Result<Vec<Layer>> {
    let len = std::fs::metadata(path).map_err(|e| bad("geohladiny", format!("{path}: {e}")))?.len();
    if len > MAX_TEMPLATE {
        return Err(bad("geohladiny", format!("{path} is not a layer template")));
    }
    let t = std::fs::read_to_string(path).map_err(|e| bad("geohladiny", format!("{path}: {e}")))?;
    serde_json::from_str::<Vec<Layer>>(&t).map_err(|e| bad("geohladiny", format!("{path} is not a layer template: {e}")))
}

#[cfg(target_arch = "wasm32")]
fn read_layers(_path: &str) -> Result<Vec<Layer>> {
    Err(bad("geohladiny", "templates are not available on the web"))
}

/// Make the template's layers in the drawing (a layer that exists takes the template's look).
fn apply_layers(s: &mut Session, layers: Vec<Layer>) -> Result<usize> {
    let d = s.doc_mut()?;
    let mut n = 0;
    for mut l in layers.into_iter().take(10_000) {
        let name: String = l.name.trim().chars().take(255).collect();
        if name.is_empty() {
            continue;
        }
        l.name = name.clone();
        if d.linetype(&l.linetype).is_none() {
            l.linetype = "Continuous".into();
        }
        l.transparency = l.transparency.min(90);
        d.ensure_layer(&name);
        if let Some(t) = d.layer_mut(&name) {
            let keep = t.name.clone();
            *t = l;
            t.name = keep;
            n += 1;
        }
    }
    Ok(n)
}

fn run_hladiny(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "geohladiny";
    if let Some(path) = str_param(p, "load") {
        let n = apply_layers(s, read_layers(path)?)?;
        return Ok(json!({ "layers": n, "message": format!("{n} layers made from {path}") }));
    }
    let path = str_param(p, "save").ok_or_else(|| bad(CMD, "`save` or `load` (a file path) is required"))?;
    let only = str_param(p, "only").unwrap_or("").to_ascii_lowercase();
    let layers: Vec<Layer> = s.doc()?.layers.iter().filter(|l| l.name.to_ascii_lowercase().starts_with(&only)).cloned().collect();
    let t = serde_json::to_string_pretty(&layers).map_err(|e| bad(CMD, e.to_string()))?;
    write_plain(path, &t, CMD)?;
    Ok(json!({ "layers": layers.len(), "message": format!("{} layers saved to {path}", layers.len()) }))
}

fn run_novy(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "geonovy";
    // Read the templates first, so a bad path leaves no half-made drawing.
    let layers = match str_param(p, "layers") {
        Some(path) => Some(read_layers(path)?),
        None => None,
    };
    let mut settings = json!({});
    if let Some(path) = str_param(p, "settings") {
        settings["load"] = json!(path);
    }
    if let Some(v) = p.get("scale") {
        settings["scale"] = v.clone();
    }
    let metric = true;
    s.new_drawing(metric);
    s.execute("geonastaveni", &settings).map_err(|e| bad(CMD, e.to_string()))?;
    let mut n = 0;
    if let Some(l) = layers {
        n = apply_layers(s, l)?;
    }
    // The survey layers, so the drawing is ready before the first points come in.
    for (name, aci) in [(LAYER_POINTS, 7), (super::geo::LAYER_NUMBERS, 3), (super::geo::LAYER_HEIGHTS, 1), (super::geo::LAYER_CODES, 5)] {
        ensure_layer(s, name, aci)?;
    }
    let scale = nast::scale(s);
    Ok(
        json!({ "message": format!("New survey drawing at 1:{scale}{}", if n > 0 { format!(", {n} layers from the template") } else { String::new() }) }),
    )
}

/// Asks its questions in turn ('p' point, 'n' number, 'o' optional number, 'd' number with a
/// default), then runs the command.
struct StepsM {
    name: &'static str,
    cmd: &'static str,
    qs: &'static [(&'static str, &'static str, char)],
    answers: serde_json::Map<String, Value>,
    last: Option<Vec2>,
}

impl StepsM {
    fn new(name: &'static str, cmd: &'static str, qs: &'static [(&'static str, &'static str, char)]) -> Self {
        StepsM { name, cmd, qs, answers: serde_json::Map::new(), last: None }
    }
}

impl Interactive for StepsM {
    fn name(&self) -> &'static str {
        self.name
    }
    fn prompt(&self, _s: &Session) -> Prompt {
        match self.qs.get(self.answers.len()) {
            Some((key, msg, 'p')) => {
                let _ = key;
                Prompt::new(*msg, Accept::POINT).base_opt(self.last)
            }
            Some((key, msg, 'd')) => Prompt::new(*msg, Accept::NUMBER).default(match *key {
                "elevation" => "30",
                _ => "1",
            }),
            Some((_, msg, _)) => Prompt::new(*msg, Accept::NUMBER),
            None => Prompt::new("", Accept::TEXT),
        }
    }
    fn input(&mut self, s: &mut Session, i: Input) -> Result<Step> {
        let Some((key, _, kind)) = self.qs.get(self.answers.len()).copied() else { return Ok(Step::Done) };
        let v = match (kind, i) {
            (_, Input::Cancel) => return Ok(Step::Cancel),
            ('p', Input::Point(q)) => {
                self.last = Some(q);
                json!([q.x, q.y])
            }
            ('n' | 'o' | 'd', Input::Text(t) | Input::Keyword(t)) => {
                if typed_number(&t).is_none() {
                    s.echo("Enter a number.");
                    return Ok(Step::Continue);
                }
                json!(t.trim())
            }
            ('o', Input::Enter) => Value::Null,
            ('d', Input::Enter) => json!(self.prompt(s).default.unwrap_or_default()),
            _ => return Ok(Step::Continue),
        };
        self.answers.insert(key.into(), v);
        if self.answers.len() < self.qs.len() {
            return Ok(Step::Continue);
        }
        let r = s.execute(self.cmd, &Value::Object(self.answers.clone()));
        say(s, r);
        Ok(Step::Done)
    }
}

/// GEOHLADINY (Save or Load, then the file) and GEONOVY (scale, settings template, layer
/// template; Enter leaves a template out).
struct PathsM {
    name: &'static str,
    answers: Vec<String>,
}

impl Interactive for PathsM {
    fn name(&self) -> &'static str {
        self.name
    }
    fn prompt(&self, s: &Session) -> Prompt {
        match (self.name, self.answers.len()) {
            ("GEOHLADINY", 0) => Prompt::new("Layer template", Accept::TEXT).kw(&["Save", "Load"]).default("Save"),
            ("GEOHLADINY", _) => Prompt::new("Layer template file", Accept::TEXT),
            (_, 0) => Prompt::new("Drawing scale 1:", Accept::NUMBER).default(format!("{}", nast::scale(s))),
            (_, 1) => Prompt::new("Settings template file (Enter: none)", Accept::TEXT),
            _ => Prompt::new("Layer template file (Enter: none)", Accept::TEXT),
        }
    }
    fn input(&mut self, s: &mut Session, i: Input) -> Result<Step> {
        let t = match i {
            Input::Cancel => return Ok(Step::Cancel),
            Input::Text(t) | Input::Keyword(t) => t.trim().trim_matches('"').to_string(),
            Input::Enter => self.prompt(s).default.unwrap_or_default(),
            _ => return Ok(Step::Continue),
        };
        if self.name == "GEOHLADINY" {
            if self.answers.is_empty() {
                let k = self.prompt(s).match_keyword(&t).unwrap_or_else(|| "Save".into());
                self.answers.push(k);
                return Ok(Step::Continue);
            }
            if t.is_empty() {
                return Ok(Step::Continue);
            }
            let key = if self.answers.first().map(String::as_str) == Some("Load") { "load" } else { "save" };
            let r = run_hladiny(s, &json!({ key: t }));
            say(s, r);
            return Ok(Step::Done);
        }
        self.answers.push(t);
        if self.answers.len() < 3 {
            return Ok(Step::Continue);
        }
        let get = |k: usize| self.answers.get(k).filter(|t| !t.is_empty()).cloned();
        let mut p = json!({});
        if let Some(v) = get(0) {
            p["scale"] = json!(v);
        }
        if let Some(v) = get(1) {
            p["settings"] = json!(v);
        }
        if let Some(v) = get(2) {
            p["layers"] = json!(v);
        }
        let r = run_novy(s, &p);
        say(s, r);
        Ok(Step::Done)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn on(s: &Session, layer: &str) -> Vec<EntityKind> {
        s.doc().unwrap().model.iter().filter(|e| e.common.layer == layer).map(|e| e.kind.clone()).collect()
    }

    #[test]
    fn niche_goes_into_the_wall() {
        let mut s = Session::new();
        s.execute("geonika", &json!({ "from": [0, 0], "to": [1, 0], "toward": [0.5, -1], "depth": 0.15, "height": 0.9 })).unwrap();
        let k = on(&s, LAYER_NICHES);
        let EntityKind::LwPolyline(p) = &k[0] else { panic!() };
        assert!(p.vertices.iter().all(|v| v.p.y <= 1e-12) && p.vertices.iter().any(|v| (v.p.y + 0.15).abs() < 1e-12));
        assert!(k.iter().any(|e| matches!(e, EntityKind::Text(t) if t.value == "nika 150/900")));
    }

    #[test]
    fn truss_elements_along_and_in_section() {
        let mut s = Session::new();
        s.execute("geokrov", &json!({ "from": [0, 0], "to": [4, 3], "width": 0.12, "height": 0.16 })).unwrap();
        s.execute("geokrov", &json!({ "at": [10, 0], "width": 0.12, "height": 0.16 })).unwrap();
        let k = on(&s, LAYER_TRUSS);
        assert_eq!(k.iter().filter(|e| matches!(e, EntityKind::LwPolyline(_))).count(), 2);
        assert_eq!(k.iter().filter(|e| matches!(e, EntityKind::Line(_))).count(), 2);
        assert_eq!(k.iter().filter(|e| matches!(e, EntityKind::Text(t) if t.value == "120/160")).count(), 2);
    }

    #[test]
    fn axonometric_view_lifts_heights() {
        let mut s = Session::new();
        s.execute("geobody", &json!({ "text": "1 0 0 100\n2 10 0 100\n3 10 10 110\n", "sjtsk": false })).unwrap();
        let r = s.execute("geoaxo", &json!({ "azimuth": 270, "elevation": 0, "at": [100, 100] })).unwrap();
        assert_eq!(r["objects"], 3);
        // From the south at the horizon: across is x, up is height.
        let ys: Vec<f64> = on(&s, LAYER_3D).iter().filter_map(|e| if let EntityKind::Point(p) = e { Some(p.p.y) } else { None }).collect();
        let (lo, hi) = (ys.iter().copied().fold(f64::INFINITY, f64::min), ys.iter().copied().fold(f64::NEG_INFINITY, f64::max));
        assert!((hi - lo - 10.0).abs() < 1e-9, "{ys:?}");
        assert!(s.execute("geoaxo", &json!({ "elevation": 100 })).is_err());
    }

    #[test]
    fn room_schedule_from_labels() {
        let mut s = Session::new();
        let space = s.space();
        for (x, w, name, h) in [(0.0, 4.0, "1.02 Pokoj", Some(2.6)), (10.0, 3.0, "1.01 Kuchyně", None)] {
            let pl = LwPolyline {
                vertices: [(x, 0.0), (x + w, 0.0), (x + w, w), (x, w)].iter().map(|(a, b)| PolyVertex::new(Vec2::new(*a, *b))).collect(),
                closed: true,
                const_width: 0.0,
                elevation: 0.0,
                plinegen: false,
            };
            let b = add(&mut s, &space, "0", EntityKind::LwPolyline(pl)).unwrap();
            s.execute("geomistnost", &json!({ "boundary": b.hex(), "name": name, "floor": "dlažba", "height": h })).unwrap();
        }
        let dir = std::env::temp_dir().join(format!("geopasport-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let csv = dir.join("p.csv");
        let r = s.execute("geopasport", &json!({ "at": [0, -5], "csv": csv.to_string_lossy() })).unwrap();
        assert!((r["total"].as_f64().unwrap() - 25.0).abs() < 1e-9, "{r}");
        assert_eq!(r["rooms"][0]["name"], "1.01 Kuchyně");
        assert_eq!(r["rooms"][1]["height"], "2,60");
        assert_eq!(r["rooms"][1]["floor"], "dlažba");
        let t = std::fs::read_to_string(&csv).unwrap();
        assert!(t.contains("1.02 Pokoj;16,00;dlažba;2,60"), "{t}");
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn layer_template_round_trip_and_new_drawing() {
        let dir = std::env::temp_dir().join(format!("geohladiny-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("hladiny.json");
        let mut s = Session::new();
        s.execute("geobody", &json!({ "text": "1 0 0 1\n", "sjtsk": false })).unwrap();
        s.execute("geohladiny", &json!({ "save": path.to_string_lossy(), "only": "GEO_" })).unwrap();
        let settings = dir.join("nast.json");
        s.execute("geonastaveni", &json!({ "scale": 200, "save": settings.to_string_lossy() })).unwrap();
        let r = s.execute("geonovy", &json!({ "layers": path.to_string_lossy(), "settings": settings.to_string_lossy() })).unwrap();
        assert!(r["message"].as_str().unwrap().contains("1:200"), "{r}");
        let d = s.doc().unwrap();
        assert!(d.model.iter().next().is_none(), "a new, empty drawing");
        assert!(d.layer("GEO_cisla").is_some_and(|l| l.color == cadcraft_color::Color::Index(3)));
        assert!(s.execute("geonovy", &json!({ "layers": "/nonexistent/x.json" })).is_err());
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn hostile_input_is_refused_not_panicking() {
        let mut s = Session::new();
        for (c, p) in [
            ("geonika", json!({ "from": [0, 0], "to": [0, 0], "toward": [1, 1], "depth": 1 })),
            ("geonika", json!({ "from": [0, 0], "to": [1, 0], "toward": [1, 1], "depth": -1 })),
            ("geokrov", json!({ "from": [0, 0], "to": [0, 0], "width": 0.1, "height": 0.1 })),
            ("geokrov", json!({ "at": [0, 0], "width": "x", "height": 0.1 })),
            ("geoaxo", json!({})),
            ("geopasport", json!({})),
            ("geohladiny", json!({})),
            ("geohladiny", json!({ "load": "/nonexistent/l.json" })),
        ] {
            assert!(s.execute(c, &p).is_err(), "{c} {p}");
        }
    }
}
