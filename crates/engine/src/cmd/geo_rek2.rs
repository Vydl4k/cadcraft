//! Survey add-on: more of the measured building survey.
//!
//! - GEOKLENBA draws a vault in a section or elevation: the soffit arc between the springing
//!   points with its rise, and, with a thickness, the back of the vault closing the shape.
//! - GEOOBKLAD draws a cladding (lining) along a wall face: the face offset by the cladding's
//!   thickness to the room side, closed into one outline.
//! - GEOKOTAOTVORU labels an opening with its width/height in millimetres and its sill height.
//! - GEOPROSTUP draws a wall or floor penetration: a rectangle with both diagonals and its size.
//! - GEOREZSCHODY draws a straight flight in section: the steps, the slab under them and the
//!   step count with rise and going.
//! - GEOCISTENI cleans up linework: lines with no length go, lines lying on top of each other
//!   become one, and touching or overlapping lines along the same line on one layer are joined.

use cadcraft_doc::{EntityKind, Handle, Line, LwPolyline};
use cadcraft_geom::{PolyVertex, Vec2, Vec3};
use serde_json::{Map, Value, json};

use super::geo_calc::{add, ensure_layer, one_target, path_of, say, text, typed_number};
use super::geo_nastaveni as nast;
use super::geo_popis::LAYER_STAIRS;
use super::geo_rek::offset;
use super::{CommandSpec, bad, point_value, str_param, targets};
use crate::{Accept, Input, Interactive, Prompt, Result, Session, Step};
use cadcraft_doc::{HAlign, VAlign};

pub const LAYER_VAULTS: &str = "GEO_klenby";
pub const LAYER_CLADDING: &str = "GEO_obklad";
pub const LAYER_OPENING_LABELS: &str = "GEO_otvory_popis";
pub const LAYER_PENETRATIONS: &str = "GEO_prostupy";
/// Lines cleaned at once.
const MAX_LINES: usize = 200_000;

pub(super) fn specs() -> Vec<CommandSpec> {
    vec![
        CommandSpec::new("geoklenba", "Vault in Section...", run_klenba)
            .menu(&["Geo", "Building", "Vault in Section..."])
            .alias(&["geovault"])
            .params("{from: [x, y], to: [x, y] the springing points, rise: crown above them (m), thickness?: m} → {radius, message}")
            .interactive(|_| {
                Ok(Box::new(AskM::new("GEOKLENBA", "geoklenba", vec![
                    Q::point("from", "Pick the first springing point"),
                    Q::point("to", "Pick the second springing point"),
                    Q::number("rise", "Rise of the vault (m)", None),
                    Q::number("thickness", "Vault thickness (m, Enter: none)", Some("")),
                ])))
            }),
        CommandSpec::new("geoobklad", "Wall Cladding...", run_obklad)
            .menu(&["Geo", "Building", "Wall Cladding..."])
            .alias(&["geocladding"])
            .params("{path: handle of the wall face (line or polyline), thickness: m, toward: [x, y] a point on the cladding's side} → {message}")
            .interactive(|_| {
                Ok(Box::new(AskM::new("GEOOBKLAD", "geoobklad", vec![
                    Q::pick("path", "Select the wall face"),
                    Q::number("thickness", "Cladding thickness (m)", Some("0.02")),
                    Q::point("toward", "Pick a point on the cladding's side"),
                ])))
            }),
        CommandSpec::new("geokotaotvoru", "Opening Label...", run_kotaotvoru)
            .menu(&["Geo", "Building", "Opening Label..."])
            .alias(&["geoopeninglabel"])
            .params("{from: [x, y], to: [x, y] the jambs, height?: opening height (m), sill?: sill height (m), toward?: [x, y] the side the label goes} → {text, message}")
            .interactive(|_| {
                Ok(Box::new(AskM::new("GEOKOTAOTVORU", "geokotaotvoru", vec![
                    Q::point("from", "Pick one jamb"),
                    Q::point("to", "Pick the other jamb"),
                    Q::number("height", "Opening height (m, Enter: none)", Some("")),
                    Q::number("sill", "Sill height (m, Enter: none)", Some("")),
                    Q::point("toward", "Pick the side for the label"),
                ])))
            }),
        CommandSpec::new("geoprostup", "Penetration...", run_prostup)
            .menu(&["Geo", "Building", "Penetration..."])
            .alias(&["geopenetration"])
            .params("{at: [x, y] the centre, width: m, length: m, angle?: degrees, label?: text (default \"width×length\" in mm)} → {message}")
            .interactive(|_| {
                Ok(Box::new(AskM::new("GEOPROSTUP", "geoprostup", vec![
                    Q::point("at", "Pick the centre of the penetration"),
                    Q::number("width", "Width (m)", None),
                    Q::number("length", "Length (m)", None),
                ])))
            }),
        CommandSpec::new("georezschody", "Stairs in Section...", run_rezschody)
            .menu(&["Geo", "Building", "Stairs in Section..."])
            .alias(&["geostairsection"])
            .params("{from: [x, y] the foot (front of the first step at floor level), length: run (m, negative: to the left), rise: total height (m), steps: count, slab?: slab thickness (m, default 0.15)} → {riser, going, message}")
            .interactive(|_| {
                Ok(Box::new(AskM::new("GEOREZSCHODY", "georezschody", vec![
                    Q::point("from", "Pick the foot of the flight"),
                    Q::number("length", "Run length (m, negative: to the left)", None),
                    Q::number("rise", "Total rise (m)", None),
                    Q::number("steps", "Number of steps", None),
                    Q::number("slab", "Slab thickness (m)", Some("0.15")),
                ])))
            }),
        CommandSpec::new("geocisteni", "Clean Up Lines...", run_cisteni)
            .menu(&["Geo", "Tools", "Clean Up Lines..."])
            .alias(&["geocleanup"])
            .params("{handles?: lines (default the selection, else every line), tolerance?: m (default 0.001)} → {removed, joined, message}")
            .interactive(|_| Ok(Box::new(AskM::new("GEOCISTENI", "geocisteni", vec![Q::pick("handles", "Select the lines to clean up")])))),
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

fn poly(v: Vec<PolyVertex>, closed: bool) -> EntityKind {
    EntityKind::LwPolyline(LwPolyline { vertices: v, closed, const_width: 0.0, elevation: 0.0, plinegen: false })
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

fn run_klenba(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "geoklenba";
    let (a, b) = (pt(p, "from", CMD)?, pt(p, "to", CMD)?);
    let c = (b.x - a.x).hypot(b.y - a.y);
    if c < 1e-6 {
        return Err(bad(CMD, "the springing points are at the same place"));
    }
    let f = need(p, "rise", CMD, -1000.0..=1000.0)?;
    if f.abs() < 1e-6 || f.abs() > c {
        return Err(bad(CMD, "the rise must be more than 0 and at most the span"));
    }
    let t = num(p, "thickness", CMD, 0.0..=100.0)?.filter(|t| *t > 0.0);
    // The arc bulges to the left of a→b for a positive rise: bulge = 2f/c, negative for a
    // clockwise arc. Rise up from a left-to-right chord means clockwise.
    let bulge = -2.0 * f / c;
    let r = (c * c / 4.0 + f * f) / (2.0 * f.abs());
    ensure_layer(s, LAYER_VAULTS, 7)?;
    let space = s.space();
    match t {
        None => {
            add(s, &space, LAYER_VAULTS, poly(vec![PolyVertex::with_bulge(a, bulge), PolyVertex::new(b)], false))?;
        }
        Some(t) => {
            // The back of the vault: the concentric arc t further out, between the radial lines
            // through the springing points.
            let u = Vec2::new((b.x - a.x) / c, (b.y - a.y) / c);
            let up = Vec2::new(u.y, -u.x) * f.signum(); // perpendicular, toward the crown
            let up = Vec2::new(-up.x, -up.y);
            let m = Vec2::new((a.x + b.x) / 2.0, (a.y + b.y) / 2.0);
            let centre = Vec2::new(m.x - up.x * (r - f.abs()), m.y - up.y * (r - f.abs()));
            let out = |q: Vec2| {
                let (dx, dy) = (q.x - centre.x, q.y - centre.y);
                let l = dx.hypot(dy).max(1e-12);
                Vec2::new(centre.x + dx / l * (r + t), centre.y + dy / l * (r + t))
            };
            let (a2, b2) = (out(a), out(b));
            let c2 = (b2.x - a2.x).hypot(b2.y - a2.y);
            // More than a half circle when the rise is more than half the span.
            let root = ((r + t).powi(2) - c2 * c2 / 4.0).max(0.0).sqrt();
            let f2 = f.signum() * if f.abs() > c / 2.0 { (r + t) + root } else { (r + t) - root };
            let bulge2 = 2.0 * f2 / c2.max(1e-12);
            add(
                s,
                &space,
                LAYER_VAULTS,
                poly(vec![PolyVertex::with_bulge(a, bulge), PolyVertex::new(b), PolyVertex::with_bulge(b2, bulge2), PolyVertex::new(a2)], true),
            )?;
        }
    }
    Ok(json!({ "radius": r, "message": format!("Vault: span {c:.3} m, rise {:.3} m, radius {r:.3} m", f.abs()) }))
}

fn run_obklad(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "geoobklad";
    let h = one_target(s, p, "path", CMD)?;
    let space = s.space();
    let (v, closed) = path_of(s, h, &space).ok_or_else(|| bad(CMD, "the wall face must be a line or polyline"))?;
    let t = need(p, "thickness", CMD, 0.0001..=10.0)?;
    let toward = pt(p, "toward", CMD)?;
    let near = |o: &[Vec2]| o.iter().map(|q| (q.x - toward.x).hypot(q.y - toward.y)).fold(f64::INFINITY, f64::min);
    let l = offset(&v, closed, t).ok_or_else(|| bad(CMD, "the wall face has no length"))?;
    let r = offset(&v, closed, -t).ok_or_else(|| bad(CMD, "the wall face has no length"))?;
    let o = if near(&l) <= near(&r) { l } else { r };
    ensure_layer(s, LAYER_CLADDING, 4)?;
    if closed {
        add(s, &space, LAYER_CLADDING, poly(o.into_iter().map(PolyVertex::new).collect(), true))?;
    } else {
        // The face and the offset, joined at the ends.
        let mut ring: Vec<PolyVertex> = v.iter().copied().map(PolyVertex::new).collect();
        ring.extend(o.into_iter().rev().map(PolyVertex::new));
        add(s, &space, LAYER_CLADDING, poly(ring, true))?;
    }
    Ok(json!({ "message": format!("Cladding {:.0} mm", t * 1000.0) }))
}

fn mm(v: f64) -> String {
    format!("{:.0}", v * 1000.0)
}

fn run_kotaotvoru(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "geokotaotvoru";
    let (a, b) = (pt(p, "from", CMD)?, pt(p, "to", CMD)?);
    let w = (b.x - a.x).hypot(b.y - a.y);
    if w < 1e-6 {
        return Err(bad(CMD, "the jambs are at the same place"));
    }
    let height = num(p, "height", CMD, 0.0..=1000.0)?;
    let sill = num(p, "sill", CMD, -1000.0..=1000.0)?;
    let ang = readable((b.y - a.y).atan2(b.x - a.x));
    let m = Vec2::new((a.x + b.x) / 2.0, (a.y + b.y) / 2.0);
    let mut n = Vec2::new(-ang.sin(), ang.cos());
    if p.get("toward").is_some() {
        let q = pt(p, "toward", CMD)?;
        if (q.x - m.x) * n.x + (q.y - m.y) * n.y < 0.0 {
            n = Vec2::new(-n.x, -n.y);
        }
    }
    let h = notes_h(s);
    let value = match height {
        Some(z) => format!("{}/{}", mm(w), mm(z)),
        None => mm(w),
    };
    ensure_layer(s, LAYER_OPENING_LABELS, 7)?;
    let space = s.space();
    // Text above the line reads from its baseline; below it from its top.
    let above = n.x * -ang.sin() + n.y * ang.cos() > 0.0;
    let (va, vb) = if above { (VAlign::Bottom, VAlign::Top) } else { (VAlign::Top, VAlign::Bottom) };
    let first = Vec2::new(m.x + n.x * 0.5 * h, m.y + n.y * 0.5 * h);
    text(s, &space, LAYER_OPENING_LABELS, first, h, value.clone(), ang, (HAlign::Center, va))?;
    if let Some(z) = sill {
        let second = Vec2::new(m.x - n.x * 0.5 * h, m.y - n.y * 0.5 * h);
        text(s, &space, LAYER_OPENING_LABELS, second, 0.8 * h, format!("P {}", mm(z)), ang, (HAlign::Center, vb))?;
    }
    Ok(json!({ "text": value, "message": format!("Opening {value}") }))
}

fn run_prostup(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "geoprostup";
    let c = pt(p, "at", CMD)?;
    let w = need(p, "width", CMD, 0.001..=1000.0)?;
    let l = need(p, "length", CMD, 0.001..=1000.0)?;
    let ang = num(p, "angle", CMD, -360.0..=360.0)?.unwrap_or(0.0).to_radians();
    let (cs, sn) = (ang.cos(), ang.sin());
    let at = |x: f64, y: f64| Vec2::new(c.x + x * cs - y * sn, c.y + x * sn + y * cs);
    let k = [at(-w / 2.0, -l / 2.0), at(w / 2.0, -l / 2.0), at(w / 2.0, l / 2.0), at(-w / 2.0, l / 2.0)];
    ensure_layer(s, LAYER_PENETRATIONS, 1)?;
    let space = s.space();
    add(s, &space, LAYER_PENETRATIONS, poly(k.iter().copied().map(PolyVertex::new).collect(), true))?;
    let ln = |a: Vec2, b: Vec2| EntityKind::Line(Line { a: Vec3::new(a.x, a.y, 0.0), b: Vec3::new(b.x, b.y, 0.0) });
    add(s, &space, LAYER_PENETRATIONS, ln(k[0], k[2]))?;
    add(s, &space, LAYER_PENETRATIONS, ln(k[1], k[3]))?;
    let label = str_param(p, "label")
        .map(|t| t.trim().chars().take(100).collect::<String>())
        .filter(|t| !t.is_empty())
        .unwrap_or_else(|| format!("{}×{}", mm(w), mm(l)));
    let h = notes_h(s);
    let r = readable(ang);
    let off = l / 2.0 + 0.5 * h;
    let q = Vec2::new(c.x - r.sin() * off, c.y + r.cos() * off);
    text(s, &space, LAYER_PENETRATIONS, q, 0.8 * h, label.clone(), r, (HAlign::Center, VAlign::Bottom))?;
    Ok(json!({ "message": format!("Penetration {label}") }))
}

fn run_rezschody(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "georezschody";
    let f = pt(p, "from", CMD)?;
    let len = need(p, "length", CMD, -1000.0..=1000.0)?;
    let rise = need(p, "rise", CMD, 0.001..=1000.0)?;
    let steps = need(p, "steps", CMD, 1.0..=500.0)?.round() as usize;
    let slab = num(p, "slab", CMD, 0.0..=10.0)?.unwrap_or(0.15);
    if len.abs() < 1e-6 {
        return Err(bad(CMD, "the run has no length"));
    }
    let (g, h) = (len / steps as f64, rise / steps as f64);
    let mut v = vec![f];
    for i in 0..steps {
        let (x, y) = (f.x + g * i as f64, f.y + h * (i + 1) as f64);
        v.push(Vec2::new(x, y));
        v.push(Vec2::new(x + g, y));
    }
    let top = Vec2::new(f.x + len, f.y + rise);
    // The slab's underside: parallel to the pitch line through the nosings, `slab` under it.
    let cos = len.abs() / len.hypot(rise);
    let drop = h + slab / cos.max(1e-9);
    let under_top = Vec2::new(top.x, top.y - drop);
    let k = rise / len; // dy/dx of the pitch line
    // Where the underside meets the floor in front of the foot.
    let x_floor = top.x - (under_top.y - f.y) / k;
    let floor = Vec2::new(x_floor, f.y);
    let toward = (x_floor - f.x) * len.signum() > 0.0;
    v.push(under_top);
    if toward {
        v.push(floor);
    }
    ensure_layer(s, LAYER_STAIRS, 7)?;
    let space = s.space();
    add(s, &space, LAYER_STAIRS, poly(v.into_iter().map(PolyVertex::new).collect(), true))?;
    let label = format!("{steps} × {}/{}", mm(h), mm(g.abs()));
    let th = notes_h(s);
    let ang = readable(rise.atan2(len));
    let mid = Vec2::new(f.x + len / 2.0, f.y + rise / 2.0);
    let n = Vec2::new(-ang.sin(), ang.cos());
    text(
        s,
        &space,
        LAYER_STAIRS,
        Vec2::new(mid.x + n.x * (h + 0.5 * th), mid.y + n.y * (h + 0.5 * th)),
        0.8 * th,
        label.clone(),
        ang,
        (HAlign::Center, VAlign::Bottom),
    )?;
    Ok(json!({ "riser": h, "going": g.abs(), "message": format!("Stairs in section {label} (mm)") }))
}

fn run_cisteni(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "geocisteni";
    let tol = num(p, "tolerance", CMD, 0.0..=10.0)?.unwrap_or(0.001).max(1e-9);
    let space = s.space();
    let chosen = targets(s, p)?;
    let lines: Vec<(Handle, String, Vec3, Vec3)> = {
        let d = s.doc()?;
        let st = d.space(&space).ok_or_else(|| bad(CMD, "no drawing space"))?;
        let pick = |e: &cadcraft_doc::Entity| match &e.kind {
            EntityKind::Line(l) if [l.a, l.b].iter().all(|q| q.x.is_finite() && q.y.is_finite() && q.z.is_finite()) => {
                Some((e.handle, e.common.layer.clone(), l.a, l.b))
            }
            _ => None,
        };
        if chosen.is_empty() {
            st.iter().filter_map(|e| pick(e)).take(MAX_LINES + 1).collect()
        } else {
            chosen.iter().filter_map(|h| st.get(*h)).filter_map(|e| pick(e)).take(MAX_LINES + 1).collect()
        }
    };
    if lines.len() > MAX_LINES {
        return Err(bad(CMD, format!("more than {MAX_LINES} lines; select fewer")));
    }
    let mut removed = 0;
    let mut gone: std::collections::BTreeSet<Handle> = Default::default();
    // Lines with no length.
    for (h, _, a, b) in &lines {
        if (b.x - a.x).hypot(b.y - a.y) <= tol && (b.z - a.z).abs() <= tol {
            gone.insert(*h);
            removed += 1;
        }
    }
    // Collinear lines on one layer that touch or overlap become one. Group by layer and by the
    // line they lie on (direction and distance from the origin, rounded to the tolerance).
    let mut groups: std::collections::BTreeMap<(String, i64, i64), Vec<usize>> = Default::default();
    for (i, (h, layer, a, b)) in lines.iter().enumerate() {
        if gone.contains(h) || (a.z - b.z).abs() > tol || (a.z).abs() > 1e6 {
            continue;
        }
        // The line's direction, turned to point into the upper half plane.
        let (mut dx, mut dy) = (b.x - a.x, b.y - a.y);
        if dy < 0.0 || (dy == 0.0 && dx < 0.0) {
            (dx, dy) = (-dx, -dy);
        }
        let l = dx.hypot(dy);
        let ang = dy.atan2(dx);
        let dist = (a.x * dy - a.y * dx) / l;
        let key = (format!("{layer}\u{0}{:.6}", a.z), (ang * 1e6).round() as i64, (dist / tol).round() as i64);
        groups.entry(key).or_default().push(i);
    }
    let mut joined = 0;
    let mut new_lines: Vec<(String, Vec3, Vec3)> = Vec::new();
    for idx in groups.values().filter(|g| g.len() > 1) {
        let Some(&first) = idx.first() else { continue };
        let Some((_, layer, a0, b0)) = lines.get(first) else { continue };
        let (dx, dy) = (b0.x - a0.x, b0.y - a0.y);
        let l = dx.hypot(dy);
        let u = Vec2::new(dx / l, dy / l);
        let o = Vec2::new(a0.x, a0.y);
        let tpar = |q: &Vec3| (q.x - o.x) * u.x + (q.y - o.y) * u.y;
        let mut spans: Vec<(f64, f64, usize)> =
            idx.iter().filter_map(|&i| lines.get(i).map(|(_, _, a, b)| (tpar(a).min(tpar(b)), tpar(a).max(tpar(b)), i))).collect();
        spans.sort_by(|x, y| x.0.total_cmp(&y.0));
        let mut cur: Option<(f64, f64, Vec<usize>)> = None;
        let flush = |c: Option<(f64, f64, Vec<usize>)>,
                     new_lines: &mut Vec<(String, Vec3, Vec3)>,
                     gone: &mut std::collections::BTreeSet<Handle>,
                     joined: &mut usize| {
            if let Some((s0, s1, members)) = c
                && members.len() > 1
            {
                for m in &members {
                    if let Some((h, ..)) = lines.get(*m) {
                        gone.insert(*h);
                    }
                }
                *joined += members.len();
                let at = |t: f64| Vec3::new(o.x + u.x * t, o.y + u.y * t, a0.z);
                new_lines.push((layer.clone(), at(s0), at(s1)));
            }
        };
        for (s0, s1, i) in spans {
            cur = match cur {
                Some((c0, c1, mut m)) if s0 <= c1 + tol => {
                    m.push(i);
                    Some((c0, c1.max(s1), m))
                }
                other => {
                    flush(other, &mut new_lines, &mut gone, &mut joined);
                    Some((s0, s1, vec![i]))
                }
            };
        }
        flush(cur, &mut new_lines, &mut gone, &mut joined);
    }
    let made = new_lines.len();
    {
        let d = s.doc_mut()?;
        for h in &gone {
            d.remove_entity(*h);
        }
    }
    for (layer, a, b) in new_lines {
        add(s, &space, &layer, EntityKind::Line(Line { a, b }))?;
    }
    Ok(json!({
        "removed": removed,
        "joined": joined,
        "message": format!("{removed} lines without length removed; {joined} overlapping or touching lines joined into {made}"),
    }))
}

/// One question of a step-by-step command.
struct Q {
    key: &'static str,
    msg: &'static str,
    kind: QKind,
    default: Option<&'static str>,
}

#[derive(Clone, Copy, PartialEq)]
enum QKind {
    Point,
    Number,
    Pick,
}

impl Q {
    fn point(key: &'static str, msg: &'static str) -> Q {
        Q { key, msg, kind: QKind::Point, default: None }
    }
    fn number(key: &'static str, msg: &'static str, default: Option<&'static str>) -> Q {
        Q { key, msg, kind: QKind::Number, default }
    }
    fn pick(key: &'static str, msg: &'static str) -> Q {
        Q { key, msg, kind: QKind::Pick, default: None }
    }
}

/// Asks its questions in turn, then runs the command with the answers.
struct AskM {
    name: &'static str,
    cmd: &'static str,
    qs: Vec<Q>,
    answers: Map<String, Value>,
    last: Option<Vec2>,
}

impl AskM {
    fn new(name: &'static str, cmd: &'static str, qs: Vec<Q>) -> Self {
        AskM { name, cmd, qs, answers: Map::new(), last: None }
    }
}

impl Interactive for AskM {
    fn name(&self) -> &'static str {
        self.name
    }
    fn prompt(&self, _s: &Session) -> Prompt {
        let Some(q) = self.qs.get(self.answers.len()) else { return Prompt::new("", Accept::TEXT) };
        let p = match q.kind {
            QKind::Point => Prompt::new(q.msg, Accept::POINT).base_opt(self.last),
            QKind::Number => Prompt::new(q.msg, Accept::NUMBER),
            QKind::Pick => Prompt::new(q.msg, Accept::SELECT),
        };
        match q.default {
            Some(d) if !d.is_empty() => p.default(d),
            _ => p,
        }
    }
    fn input(&mut self, s: &mut Session, i: Input) -> Result<Step> {
        let Some(q) = self.qs.get(self.answers.len()) else { return Ok(Step::Done) };
        let v = match (q.kind, i) {
            (_, Input::Cancel) => return Ok(Step::Cancel),
            (QKind::Point, Input::Point(p)) => {
                self.last = Some(p);
                json!([p.x, p.y])
            }
            (QKind::Pick, Input::Pick(hs)) => {
                let hs: Vec<String> = hs.iter().map(|h| h.hex()).collect();
                if q.key == "handles" { json!(hs) } else { json!(hs.first()) }
            }
            (QKind::Number, Input::Text(t) | Input::Keyword(t)) => {
                if typed_number(&t).is_none() {
                    s.echo("Enter a number.");
                    return Ok(Step::Continue);
                }
                json!(t.trim())
            }
            (_, Input::Enter) => match q.default {
                Some(d) => json!(d),
                None => return Ok(Step::Continue),
            },
            _ => return Ok(Step::Continue),
        };
        self.answers.insert(q.key.into(), v);
        if self.answers.len() < self.qs.len() {
            return Ok(Step::Continue);
        }
        let r = s.execute(self.cmd, &Value::Object(self.answers.clone()));
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
    fn vault_with_and_without_thickness() {
        let mut s = Session::new();
        let r = s.execute("geoklenba", &json!({ "from": [0, 0], "to": [4, 0], "rise": 1 })).unwrap();
        assert!((r["radius"].as_f64().unwrap() - 2.5).abs() < 1e-9);
        let EntityKind::LwPolyline(pl) = &on(&s, LAYER_VAULTS)[0] else { panic!() };
        // The arc rises above the chord: its middle at y = 1.
        let (arc, _) = cadcraft_geom::bulge_to_arc(pl.vertices[0].p, pl.vertices[1].p, pl.vertices[0].bulge).unwrap();
        assert!((arc.center.y + 1.5).abs() < 1e-9, "{arc:?}");
        s.execute("geoklenba", &json!({ "from": [0, 0], "to": [4, 0], "rise": 1, "thickness": 0.3 })).unwrap();
        let EntityKind::LwPolyline(pl) = &on(&s, LAYER_VAULTS)[1] else { panic!() };
        assert!(pl.closed && pl.vertices.len() == 4);
        // The back's ends are 0.3 out from the springing points, along the radius.
        let (a2, a) = (pl.vertices[3].p, pl.vertices[0].p);
        assert!(((a2.x - a.x).hypot(a2.y - a.y) - 0.3).abs() < 1e-9);
        let (arc2, _) = cadcraft_geom::bulge_to_arc(pl.vertices[2].p, pl.vertices[3].p, pl.vertices[2].bulge).unwrap();
        assert!((arc2.radius - 2.8).abs() < 1e-9 && (arc2.center.y + 1.5).abs() < 1e-9, "{arc2:?}");
    }

    #[test]
    fn cladding_on_the_chosen_side() {
        let mut s = Session::new();
        let space = s.space();
        let l = add(&mut s, &space, "0", EntityKind::Line(Line { a: Vec3::new(0.0, 0.0, 0.0), b: Vec3::new(5.0, 0.0, 0.0) })).unwrap();
        s.execute("geoobklad", &json!({ "path": l.hex(), "thickness": 0.02, "toward": [2, -1] })).unwrap();
        let EntityKind::LwPolyline(pl) = &on(&s, LAYER_CLADDING)[0] else { panic!() };
        assert!(pl.closed && pl.vertices.iter().all(|v| v.p.y <= 1e-12) && pl.vertices.iter().any(|v| (v.p.y + 0.02).abs() < 1e-12));
    }

    #[test]
    fn opening_label_and_penetration() {
        let mut s = Session::new();
        let r = s.execute("geokotaotvoru", &json!({ "from": [0, 0], "to": [1.2, 0], "height": 1.5, "sill": 0.9 })).unwrap();
        assert_eq!(r["text"], "1200/1500");
        let labels: Vec<String> =
            on(&s, LAYER_OPENING_LABELS).into_iter().filter_map(|k| if let EntityKind::Text(t) = k { Some(t.value) } else { None }).collect();
        assert_eq!(labels, ["1200/1500", "P 900"]);
        s.execute("geoprostup", &json!({ "at": [0, 0], "width": 0.2, "length": 0.3 })).unwrap();
        let k = on(&s, LAYER_PENETRATIONS);
        assert_eq!(k.len(), 4);
        assert!(k.iter().any(|e| matches!(e, EntityKind::Text(t) if t.value == "200×300")));
    }

    #[test]
    fn stairs_in_section() {
        let mut s = Session::new();
        let r = s.execute("georezschody", &json!({ "from": [0, 0], "length": 2.8, "rise": 1.75, "steps": 10 })).unwrap();
        assert!((r["riser"].as_f64().unwrap() - 0.175).abs() < 1e-12);
        let k = on(&s, LAYER_STAIRS);
        let EntityKind::LwPolyline(pl) = &k[0] else { panic!() };
        // Foot, 10 risers and treads, the slab under the top, back to the floor.
        assert_eq!(pl.vertices.len(), 1 + 20 + 2);
        assert!(pl.vertices.iter().all(|v| v.p.y >= -1e-9));
        // To the left too.
        s.execute("georezschody", &json!({ "from": [0, 0], "length": -2.8, "rise": 1.75, "steps": 10 })).unwrap();
    }

    #[test]
    fn cleanup_joins_and_removes() {
        let mut s = Session::new();
        let space = s.space();
        let mk = |s: &mut Session, a: (f64, f64), b: (f64, f64)| {
            add(s, &space, "plot", EntityKind::Line(Line { a: Vec3::new(a.0, a.1, 0.0), b: Vec3::new(b.0, b.1, 0.0) })).unwrap()
        };
        mk(&mut s, (0.0, 0.0), (5.0, 0.0));
        mk(&mut s, (5.0, 0.0), (3.0, 0.0)); // overlapping, reversed
        mk(&mut s, (5.0, 0.0), (8.0, 0.0)); // touching
        mk(&mut s, (0.0, 0.0), (5.0, 0.0)); // duplicate
        mk(&mut s, (1.0, 1.0), (1.0, 1.0)); // no length
        mk(&mut s, (0.0, 2.0), (5.0, 2.0)); // parallel, apart
        let r = s.execute("geocisteni", &json!({})).unwrap();
        assert_eq!(r["removed"], 1);
        assert_eq!(r["joined"], 4);
        let mut lens: Vec<f64> = on(&s, "plot")
            .into_iter()
            .filter_map(|k| if let EntityKind::Line(l) = k { Some((l.b.x - l.a.x).hypot(l.b.y - l.a.y)) } else { None })
            .collect();
        lens.sort_by(f64::total_cmp);
        assert_eq!(lens, [5.0, 8.0]);
    }

    #[test]
    fn step_by_step_prompts() {
        let mut s = Session::new();
        s.start("geoprostup").unwrap();
        s.input(Input::Point(Vec2::new(1.0, 1.0))).unwrap();
        s.input(Input::Text("x".into())).unwrap();
        s.input(Input::Text("0.4".into())).unwrap();
        s.input(Input::Text("0.4".into())).unwrap();
        assert_eq!(on(&s, LAYER_PENETRATIONS).len(), 4);
    }

    #[test]
    fn hostile_input_is_refused_not_panicking() {
        let mut s = Session::new();
        for (c, p) in [
            ("geoklenba", json!({ "from": [0, 0], "to": [0, 0], "rise": 1 })),
            ("geoklenba", json!({ "from": [0, 0], "to": [1, 0], "rise": 5 })),
            ("geoklenba", json!({ "from": [0, 0], "to": [1, 0], "rise": "NaN" })),
            ("geoobklad", json!({ "path": "FFFF", "thickness": 1, "toward": [0, 0] })),
            ("geokotaotvoru", json!({ "from": [0, 0], "to": [0, 0] })),
            ("geoprostup", json!({ "at": [0, 0], "width": 0, "length": 1 })),
            ("georezschody", json!({ "from": [0, 0], "length": 0, "rise": 1, "steps": 3 })),
            ("georezschody", json!({ "from": [0, 0], "length": 1, "rise": 1, "steps": 1e9 })),
            ("geocisteni", json!({ "tolerance": -1 })),
        ] {
            assert!(s.execute(c, &p).is_err(), "{c} {p}");
        }
    }
}
