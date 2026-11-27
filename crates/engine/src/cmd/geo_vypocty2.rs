//! Survey add-on: more coordinate calculations.
//!
//! - GEOPROTINANI (intersection): a new point from two known points and either the two measured
//!   distances to it or the two angles measured at them.
//! - GEOORTO (orthogonal method): points from stations along a measuring line A→B and
//!   perpendicular offsets from it (+ to the right of the line, − to the left), with the usual
//!   scaling when the measured length of the line is given.
//! - GEOOBLOUK (setting out a curve): the circular arc of a given radius between two tangents,
//!   with optional transition curves (clothoids) of a given length at both ends: its elements
//!   (deflection angle, tangent length, arc length, external distance), main points and detail
//!   points at a station spacing, drawn and listed.
//! - GEOMISTOPIS (tie sketch): the distances from a point to nearby reference points, drawn as
//!   tie lines with their lengths, and the point's number and coordinates beside it.
//!
//! Everything is computed in the drawing's plane. "Left" and "right" are those of the map as
//! drawn (north up), which S-JTSK drawings (x = −Y, y = −X) are.

use cadcraft_doc::{Circle, EntityKind, HAlign, Line, LwPolyline, VAlign};
use cadcraft_geom::{PolyVertex, Vec2, Vec3};
use serde_json::{Value, json};

use super::geo_calc::{add, ensure_layer, say, text, typed_number};
use super::geo_pohled::numbered_points;
use super::geo_rezy::station_text;
use super::geo_vypocty::{Frame, number, point_ref, unit};
use super::{CommandSpec, bad, f64_or, point_value, str_param};
use crate::{Accept, Input, Interactive, Prompt, Result, Session, Step};

pub const LAYER_CURVE: &str = "GEO_oblouk";
pub const LAYER_TIES: &str = "GEO_mistopis";

/// Measurement rows or detail points handled at once.
const MAX_ROWS: usize = 100_000;

pub(super) fn specs() -> Vec<CommandSpec> {
    vec![
        CommandSpec::new("geoprotinani", "Intersection...", run_geoprotinani)
            .menu(&["Geo", "Calculations", "Intersection..."])
            .alias(&["geointersection"])
            .params("{a, b: point numbers or [x, y]; method?: \"delky\" (distances, default) | \"uhly\" (angles); da, db: distances from A and B — or alfa, beta: the angles at A and B between the other point and the new one; side?: \"left\" | \"right\" of A→B (default left), units?: \"gon\" | \"deg\", number?: the new point's number (default \"P\"), add?: bool, draw it like GEOBODY (default true), scale?: (default 500)} → {point: {number, y, x}, at: [x, y], angle (at the new point, gon), message}")
            .interactive(|_| Ok(Box::new(ProtM::default()))),
        CommandSpec::new("geoorto", "Orthogonal Method...", run_geoorto)
            .menu(&["Geo", "Calculations", "Orthogonal Method..."])
            .alias(&["geoorthogonal"])
            .params("{a, b: the measuring line's ends (point numbers or [x, y]), measurements: text, one line per point \"number station offset\" (offset + right of A→B, − left), measured?: the line's measured length (stations and offsets are scaled by computed/measured), add?: bool (default true), scale?: (default 500)} → {points: [{number, y, x}], length, difference?, text, message}")
            .interactive(|_| Ok(Box::new(OrtoM::default()))),
        CommandSpec::new("geooblouk", "Curve Setting Out...", run_geooblouk)
            .menu(&["Geo", "Calculations", "Curve Setting Out..."])
            .alias(&["geocurve", "geoprechodnice"])
            .params("{from: [x, y] a point on the incoming tangent, vertex: [x, y] where the tangents meet, to: [x, y] a point on the outgoing tangent, radius, transition?: length of the clothoid at each end (default 0 = plain circular arc), step?: detail point spacing (default 10; 0 = none), start?: station of the curve's beginning (default 0), scale?: 1:N for text (default 500)} → {alpha (gon), tangent, arc, length, external, shift?, points: [{name, station, x, y}], message}")
            .interactive(|_| Ok(Box::new(ObloukM::default()))),
        CommandSpec::new("geomistopis", "Tie Sketch...", run_geomistopis)
            .menu(&["Geo", "Map", "Tie Sketch..."])
            .alias(&["geoties"])
            .params("{point: point number or [x, y], refs: 1 to 8 reference points (numbers or [x, y]), scale?: 1:N for text (default 500)} → {distances: [..], message}")
            .interactive(|_| Ok(Box::new(MistoM::default()))),
    ]
}

/// Draw computed points like GEOBODY does, from their drawing positions.
fn add_points(s: &mut Session, pts: &[(String, Vec2)], frame: Frame, p: &Value) -> Result<String> {
    let mut list = String::new();
    for (num, q) in pts {
        let (y, x) = frame.to_yx(*q);
        list.push_str(&format!("{} {y:.3} {x:.3}\n", num.split_whitespace().collect::<Vec<_>>().join("_")));
    }
    if p.get("add").and_then(Value::as_bool).unwrap_or(true) {
        let scale = p.get("scale").cloned().unwrap_or(json!(500));
        s.execute("geobody", &json!({ "text": list, "columns": "PYXZ", "sjtsk": frame.sjtsk(), "scale": scale }))?;
    }
    Ok(list)
}

// --- GEOPROTINANI --------------------------------------------------------------------------------

/// The point `da` from `a` and `db` from `b`, on the left (`side` > 0) or right of a→b.
pub fn from_distances(a: Vec2, b: Vec2, da: f64, db: f64, side: f64) -> Option<Vec2> {
    let c = (b.x - a.x).hypot(b.y - a.y);
    if !(c.is_finite() && c > 1e-9 && da > 0.0 && db > 0.0) {
        return None;
    }
    let x = (da * da - db * db + c * c) / (2.0 * c);
    let h2 = da * da - x * x;
    // The circles must meet; a hair's miss (rounded measurements) counts as touching.
    if h2 < -1e-6 * da.max(db).powi(2) {
        return None;
    }
    let h = h2.max(0.0).sqrt();
    let u = Vec2::new((b.x - a.x) / c, (b.y - a.y) / c);
    let n = Vec2::new(-u.y * side.signum(), u.x * side.signum());
    Some(Vec2::new(a.x + u.x * x + n.x * h, a.y + u.y * x + n.y * h))
}

/// The point seen under `alfa` at `a` (from b) and `beta` at `b` (from a), in radians.
pub fn from_angles(a: Vec2, b: Vec2, alfa: f64, beta: f64, side: f64) -> Option<Vec2> {
    let c = (b.x - a.x).hypot(b.y - a.y);
    let sum = alfa + beta;
    if !(c.is_finite() && c > 1e-9 && alfa > 0.0 && beta > 0.0 && sum < std::f64::consts::PI - 1e-9) {
        return None;
    }
    let ap = c * beta.sin() / sum.sin();
    let turn = alfa * side.signum();
    let u = Vec2::new((b.x - a.x) / c, (b.y - a.y) / c);
    let d = Vec2::new(u.x * turn.cos() - u.y * turn.sin(), u.x * turn.sin() + u.y * turn.cos());
    Some(Vec2::new(a.x + d.x * ap, a.y + d.y * ap))
}

fn side_of(p: &Value, cmd: &str) -> Result<f64> {
    match str_param(p, "side").unwrap_or("left") {
        "left" | "vlevo" | "l" => Ok(1.0),
        "right" | "vpravo" | "r" | "p" => Ok(-1.0),
        o => Err(bad(cmd, format!("`side` must be left or right, not \"{o}\""))),
    }
}

fn positive(p: &Value, key: &str, cmd: &str) -> Result<f64> {
    p.get(key)
        .and_then(Value::as_f64)
        .filter(|v| v.is_finite() && *v > 0.0 && *v < 1e7)
        .ok_or_else(|| bad(cmd, format!("`{key}` must be a positive number")))
}

fn run_geoprotinani(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "geoprotinani";
    let space = s.space();
    let a = point_ref(s, &space, p.get("a"), "a", CMD)?.xy();
    let b = point_ref(s, &space, p.get("b"), "b", CMD)?.xy();
    let side = side_of(p, CMD)?;
    let at = match str_param(p, "method").unwrap_or("delky") {
        "delky" | "délky" | "distances" => {
            let (da, db) = (positive(p, "da", CMD)?, positive(p, "db", CMD)?);
            from_distances(a, b, da, db, side).ok_or_else(|| {
                bad(CMD, "the two distances don't meet: together they are shorter than A–B, or one is longer than the other plus A–B")
            })?
        }
        "uhly" | "úhly" | "angles" => {
            let k = unit(p)?;
            let (alfa, beta) = (positive(p, "alfa", CMD)? * k, positive(p, "beta", CMD)? * k);
            from_angles(a, b, alfa, beta, side)
                .ok_or_else(|| bad(CMD, "the two angles must add up to less than a half turn, and A and B must differ"))?
        }
        o => return Err(bad(CMD, format!("unknown method \"{o}\": use delky or uhly"))),
    };
    if !(at.x.is_finite() && at.y.is_finite()) {
        return Err(bad(CMD, "the intersection could not be computed"));
    }
    // The angle the two rays make at the new point: flat or sharp intersections are weak.
    let (va, vb) = (Vec2::new(a.x - at.x, a.y - at.y), Vec2::new(b.x - at.x, b.y - at.y));
    let angle = (va.x * vb.y - va.y * vb.x).atan2(va.x * vb.x + va.y * vb.y).abs() * 200.0 / std::f64::consts::PI;
    let num = match p.get("number") {
        Some(Value::String(t)) if !t.trim().is_empty() && t.chars().count() <= 32 => t.trim().to_string(),
        Some(Value::Number(n)) => n.to_string(),
        None | Some(Value::Null) => "P".to_string(),
        Some(_) => return Err(bad(CMD, "`number` must be a point number")),
    };
    let frame = Frame::of(&[a, b]);
    add_points(s, &[(num.clone(), at)], frame, p)?;
    let (y, x) = frame.to_yx(at);
    let weak = if !(30.0..=170.0).contains(&angle) { " — a weak intersection, check it with another measurement" } else { "" };
    Ok(json!({
        "point": { "number": num, "y": y, "x": x }, "at": [at.x, at.y], "angle": angle,
        "message": format!("Point {num}: Y {y:.3}, X {x:.3}; the rays meet at {angle:.1} gon{weak}"),
    }))
}

// --- GEOORTO -------------------------------------------------------------------------------------

/// The point at `station` along a→b and `offset` to its right (negative: left).
pub fn orthogonal(a: Vec2, b: Vec2, station: f64, offset: f64, ratio: f64) -> Option<Vec2> {
    let c = (b.x - a.x).hypot(b.y - a.y);
    if !(c.is_finite() && c > 1e-9) {
        return None;
    }
    let u = Vec2::new((b.x - a.x) / c, (b.y - a.y) / c);
    let r = Vec2::new(u.y, -u.x);
    Some(Vec2::new(a.x + (u.x * station + r.x * offset) * ratio, a.y + (u.y * station + r.y * offset) * ratio))
}

fn run_geoorto(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "geoorto";
    let space = s.space();
    let a = point_ref(s, &space, p.get("a"), "a", CMD)?.xy();
    let b = point_ref(s, &space, p.get("b"), "b", CMD)?.xy();
    let length = (b.x - a.x).hypot(b.y - a.y);
    if !(length.is_finite() && length > 1e-9) {
        return Err(bad(CMD, "A and B are the same point"));
    }
    let measured = match p.get("measured") {
        None | Some(Value::Null) => None,
        Some(_) => Some(positive(p, "measured", CMD)?),
    };
    let ratio = measured.map_or(1.0, |m| length / m);
    let rows = str_param(p, "measurements").ok_or_else(|| bad(CMD, "`measurements` is required: one line per point, \"number station offset\""))?;
    let mut pts: Vec<(String, Vec2)> = Vec::new();
    for line in rows.lines().filter(|l| !l.trim().is_empty()) {
        if pts.len() >= MAX_ROWS {
            return Err(bad(CMD, format!("at most {MAX_ROWS} points at once")));
        }
        let f: Vec<&str> = line.split([' ', '\t', ';']).filter(|t| !t.is_empty()).collect();
        let (Some(num), Some(st), Some(off)) = (f.first(), f.get(1).and_then(|t| number(t)), f.get(2).and_then(|t| number(t))) else {
            return Err(bad(CMD, format!("can't read \"{}\": expected \"number station offset\"", line.trim())));
        };
        if st.abs() > 1e6 || off.abs() > 1e6 {
            return Err(bad(CMD, format!("\"{}\": station or offset out of range", line.trim())));
        }
        let q = orthogonal(a, b, st, off, ratio).ok_or_else(|| bad(CMD, "A and B are the same point"))?;
        pts.push(((*num).to_string(), q));
    }
    if pts.is_empty() {
        return Err(bad(CMD, "no measurements"));
    }
    let frame = Frame::of(&[a, b]);
    let list = add_points(s, &pts, frame, p)?;
    let out: Vec<Value> = pts
        .iter()
        .map(|(n, q)| {
            let (y, x) = frame.to_yx(*q);
            json!({ "number": n, "y": y, "x": x })
        })
        .collect();
    let difference = measured.map(|m| m - length);
    Ok(json!({
        "points": out, "length": length, "difference": difference, "text": list,
        "message": format!(
            "{} points by the orthogonal method on a line of {length:.3} m{}",
            pts.len(),
            difference.map_or_else(String::new, |d| format!(" (measured {d:+.3} m off)"))
        ),
    }))
}

// --- GEOOBLOUK -----------------------------------------------------------------------------------

/// A curve between two tangents: a circular arc of `radius` with a clothoid of `transition`
/// length at each end (0 for none).
#[derive(Debug, Clone)]
pub struct Curve {
    /// Start of the curve (TP, or TK without transitions) and the incoming direction.
    pub start: Vec2,
    u1: Vec2,
    /// End of the curve and the outgoing direction.
    pub end: Vec2,
    u2: Vec2,
    /// +1 turning left, −1 right.
    turn: f64,
    centre: Vec2,
    pub radius: f64,
    pub transition: f64,
    /// Deflection angle between the tangents, radians (positive).
    pub alpha: f64,
    /// Tangent length from the vertex to the start (and to the end).
    pub tangent: f64,
    /// Length of the circular part.
    pub arc: f64,
    /// How far the circle is moved in from the tangents by the transitions.
    pub shift: f64,
}

/// (x, y) along and across the tangent at arc length `l` of a clothoid reaching `radius` after
/// `total`: ∫cos θ, ∫sin θ with θ = t² / (2·R·L), by Simpson's rule.
fn clothoid(l: f64, radius: f64, total: f64) -> (f64, f64) {
    if l <= 0.0 || total <= 0.0 {
        return (l.max(0.0), 0.0);
    }
    let k = 1.0 / (2.0 * radius * total);
    let n = 64;
    let h = l / n as f64;
    let (mut x, mut y) = (0.0, 0.0);
    for i in 0..=n {
        let t = i as f64 * h;
        let w = if i == 0 || i == n {
            1.0
        } else if i % 2 == 1 {
            4.0
        } else {
            2.0
        };
        let th = k * t * t;
        x += w * th.cos();
        y += w * th.sin();
    }
    (x * h / 3.0, y * h / 3.0)
}

impl Curve {
    pub fn new(from: Vec2, vertex: Vec2, to: Vec2, radius: f64, transition: f64) -> std::result::Result<Curve, String> {
        let dir = |a: Vec2, b: Vec2| {
            let l = (b.x - a.x).hypot(b.y - a.y);
            (l.is_finite() && l > 1e-9).then(|| Vec2::new((b.x - a.x) / l, (b.y - a.y) / l))
        };
        let (Some(u1), Some(u2)) = (dir(from, vertex), dir(vertex, to)) else {
            return Err("the three points must differ".into());
        };
        if !(radius.is_finite() && radius > 0.0 && radius < 1e7 && (0.0..1e6).contains(&transition)) {
            return Err("the radius must be positive and the transition length 0 or more".into());
        }
        let signed = (u1.x * u2.y - u1.y * u2.x).atan2(u1.x * u2.x + u1.y * u2.y);
        let alpha = signed.abs();
        if !(1e-6..=std::f64::consts::PI - 1e-3).contains(&alpha) {
            return Err("the tangents are in line or turn straight back: there is no curve to fit".into());
        }
        let turn = signed.signum();
        let tau = transition / (2.0 * radius);
        if 2.0 * tau >= alpha {
            return Err(format!("the transitions are too long for this curve: at most {:.2} m each", alpha * radius * 0.999));
        }
        let (xk, yk) = clothoid(transition, radius, transition);
        let shift = yk - radius * (1.0 - tau.cos());
        let xs = xk - radius * tau.sin();
        let tangent = (radius + shift) * (alpha / 2.0).tan() + xs;
        let n1 = Vec2::new(-u1.y * turn, u1.x * turn);
        let start = Vec2::new(vertex.x - u1.x * tangent, vertex.y - u1.y * tangent);
        let end = Vec2::new(vertex.x + u2.x * tangent, vertex.y + u2.y * tangent);
        let centre = Vec2::new(start.x + u1.x * xs + n1.x * (radius + shift), start.y + u1.y * xs + n1.y * (radius + shift));
        let c = Curve { start, u1, end, u2, turn, centre, radius, transition, alpha, tangent, arc: radius * (alpha - 2.0 * tau), shift };
        if !(c.tangent.is_finite() && c.arc.is_finite() && c.centre.x.is_finite() && c.centre.y.is_finite()) {
            return Err("the curve could not be computed".into());
        }
        Ok(c)
    }

    pub fn length(&self) -> f64 {
        2.0 * self.transition + self.arc
    }

    /// The point `l` along the curve from its start.
    pub fn at(&self, l: f64) -> Vec2 {
        let l = l.clamp(0.0, self.length());
        let n1 = Vec2::new(-self.u1.y * self.turn, self.u1.x * self.turn);
        if l <= self.transition {
            let (x, y) = clothoid(l, self.radius, self.transition);
            return Vec2::new(self.start.x + self.u1.x * x + n1.x * y, self.start.y + self.u1.y * x + n1.y * y);
        }
        if l <= self.transition + self.arc {
            let th = self.transition / (2.0 * self.radius) + (l - self.transition) / self.radius;
            let (sn, cs) = th.sin_cos();
            return Vec2::new(self.centre.x + self.radius * (self.u1.x * sn - n1.x * cs), self.centre.y + self.radius * (self.u1.y * sn - n1.y * cs));
        }
        let n2 = Vec2::new(-self.u2.y * self.turn, self.u2.x * self.turn);
        let (x, y) = clothoid(self.length() - l, self.radius, self.transition);
        Vec2::new(self.end.x - self.u2.x * x + n2.x * y, self.end.y - self.u2.y * x + n2.y * y)
    }

    /// Main points: (name, distance from the start).
    pub fn main_points(&self) -> Vec<(&'static str, f64)> {
        let (l, total) = (self.transition, self.length());
        if l > 0.0 {
            vec![("TP", 0.0), ("PK", l), ("KK", total / 2.0), ("KP", total - l), ("PT", total)]
        } else {
            vec![("TK", 0.0), ("KK", total / 2.0), ("KT", total)]
        }
    }
}

fn run_geooblouk(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "geooblouk";
    let pt = |key: &str| -> Result<Vec2> {
        p.get(key)
            .and_then(point_value)
            .filter(|a| a.x.is_finite() && a.y.is_finite() && a.x.abs() < 1e9 && a.y.abs() < 1e9)
            .ok_or_else(|| bad(CMD, format!("`{key}` must be a point [x, y]")))
    };
    let (from, vertex, to) = (pt("from")?, pt("vertex")?, pt("to")?);
    let radius = positive(p, "radius", CMD)?;
    let transition = f64_or(p, "transition", 0.0);
    let step = f64_or(p, "step", 10.0);
    let start = f64_or(p, "start", 0.0);
    let scale = f64_or(p, "scale", super::geo_nastaveni::scale(s));
    if !(step.is_finite() && step >= 0.0 && start.is_finite() && start.abs() < 1e7 && scale.is_finite() && (1.0..=1e6).contains(&scale)) {
        return Err(bad(CMD, "`step` must be 0 or more, `start` a station and `scale` a map scale"));
    }
    let c = Curve::new(from, vertex, to, radius, transition).map_err(|e| bad(CMD, e))?;
    let total = c.length();
    if step > 0.0 && total / step > MAX_ROWS as f64 {
        return Err(bad(CMD, "too many detail points: use a larger `step`"));
    }
    let space = s.space();
    ensure_layer(s, LAYER_CURVE, 1)?;
    // The curve: clothoids as short chords, the circular part as a true arc.
    let mut verts: Vec<PolyVertex> = Vec::new();
    let samples = 16;
    if c.transition > 0.0 {
        for i in 0..samples {
            verts.push(PolyVertex::new(c.at(c.transition * i as f64 / samples as f64)));
        }
    }
    let pk = c.at(c.transition);
    verts.push(PolyVertex { bulge: (c.arc / c.radius / 4.0).tan() * c.turn, ..PolyVertex::new(pk) });
    verts.push(PolyVertex::new(c.at(c.transition + c.arc)));
    if c.transition > 0.0 {
        for i in 1..=samples {
            verts.push(PolyVertex::new(c.at(c.transition + c.arc + c.transition * i as f64 / samples as f64)));
        }
    }
    add(
        s,
        &space,
        LAYER_CURVE,
        EntityKind::LwPolyline(LwPolyline { vertices: verts, closed: false, const_width: 0.0, elevation: 0.0, plinegen: false }),
    )?;
    // The tangents to the vertex, thin.
    for (a, b) in [(c.start, vertex), (vertex, c.end)] {
        add(s, &space, LAYER_CURVE, EntityKind::Line(Line { a: Vec3::new(a.x, a.y, 0.0), b: Vec3::new(b.x, b.y, 0.0) }))?;
    }
    let th = 0.002 * scale;
    let mut points = Vec::new();
    // Labels go on the outside of the curve.
    let outward = |q: Vec2| {
        let d = Vec2::new(q.x - c.centre.x, q.y - c.centre.y);
        let l = d.x.hypot(d.y).max(1e-12);
        Vec2::new(d.x / l, d.y / l)
    };
    let mut mark = |s: &mut Session, name: &str, l: f64, size: f64| -> Result<()> {
        let q = c.at(l);
        let o = outward(q);
        add(
            s,
            &space,
            LAYER_CURVE,
            EntityKind::Line(Line { a: Vec3::new(q.x - o.x * size, q.y - o.y * size, 0.0), b: Vec3::new(q.x + o.x * size, q.y + o.y * size, 0.0) }),
        )?;
        let label = if name.is_empty() { station_text(start + l) } else { format!("{name} {}", station_text(start + l)) };
        let ang = o.y.atan2(o.x);
        // Read from the right-hand side; labels standing nearly upright all read upwards.
        let (ang, align) = if o.x < -0.1 { (ang + std::f64::consts::PI, HAlign::Right) } else { (ang, HAlign::Left) };
        text(s, &space, LAYER_CURVE, Vec2::new(q.x + o.x * 1.6 * size, q.y + o.y * 1.6 * size), th * 0.8, label, ang, (align, VAlign::Middle))?;
        points.push(json!({ "name": name, "station": start + l, "x": q.x, "y": q.y }));
        Ok(())
    };
    let mains = c.main_points();
    for (name, l) in &mains {
        mark(s, name, *l, th)?;
    }
    if step > 0.0 {
        // Detail points at whole multiples of `step` in the line's own stationing.
        let mut d = ((start / step).floor() + 1.0) * step - start;
        while d < total - 1e-6 {
            if !mains.iter().any(|(_, l)| (l - d).abs() < 1e-6) {
                mark(s, "", d, th * 0.5)?;
            }
            d += step;
        }
    }
    let alpha_gon = c.alpha * 200.0 / std::f64::consts::PI;
    let mid = c.at(total / 2.0);
    let external = (mid.x - vertex.x).hypot(mid.y - vertex.y);
    // The elements, written beside the vertex on the outside of the curve.
    let o = outward(vertex);
    let anchor = Vec2::new(vertex.x + o.x * 3.0 * th, vertex.y + o.y * 3.0 * th);
    let comma = |v: f64, d: usize| format!("{v:.d$}").replace('.', ",");
    let mut lines = vec![
        format!("R = {} m", comma(radius, 2)),
        format!("α = {} gon", comma(alpha_gon, 4)),
        format!("T = {} m", comma(c.tangent, 3)),
        format!("O = {} m", comma(total, 3)),
        format!("z = {} m", comma(external, 3)),
    ];
    if c.transition > 0.0 {
        lines.insert(1, format!("L = {} m", comma(c.transition, 2)));
    }
    // The list grows away from the curve: up when the vertex is above it, else down.
    let rows = lines.len();
    for (i, l) in lines.into_iter().enumerate() {
        let row = if o.y >= 0.0 { (rows - 1 - i) as f64 } else { -(i as f64) };
        text(s, &space, LAYER_CURVE, Vec2::new(anchor.x, anchor.y + row * 1.6 * th), th, l, 0.0, (HAlign::Left, VAlign::Middle))?;
    }
    Ok(json!({
        "alpha": alpha_gon, "tangent": c.tangent, "arc": c.arc, "length": total, "external": external,
        "shift": (c.transition > 0.0).then_some(c.shift), "points": points,
        "message": format!("Curve R {radius} m: α {alpha_gon:.4} gon, tangent {:.3} m, length {total:.3} m, external {external:.3} m", c.tangent),
    }))
}

// --- GEOMISTOPIS ---------------------------------------------------------------------------------

fn run_geomistopis(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "geomistopis";
    let space = s.space();
    let at = point_ref(s, &space, p.get("point"), "point", CMD)?;
    let refs = p
        .get("refs")
        .and_then(Value::as_array)
        .filter(|a| (1..=8).contains(&a.len()))
        .ok_or_else(|| bad(CMD, "`refs` must list 1 to 8 reference points"))?;
    let refs: Vec<Vec2> = refs.iter().map(|v| point_ref(s, &space, Some(v), "refs", CMD).map(|q| q.xy())).collect::<Result<_>>()?;
    let scale = f64_or(p, "scale", super::geo_nastaveni::scale(s));
    if !(scale.is_finite() && (1.0..=1e6).contains(&scale)) {
        return Err(bad(CMD, "`scale` must be a map scale like 500"));
    }
    // The point's number and height when it is a surveyed point of the drawing.
    let known = numbered_points(s, &space, None)?.into_iter().find(|(_, q)| (q.x - at.x).abs() < 1e-3 && (q.y - at.y).abs() < 1e-3);
    let distances: Vec<f64> = refs.iter().map(|r| (r.x - at.x).hypot(r.y - at.y)).collect();
    if distances.iter().any(|d| !(d.is_finite() && *d > 1e-6)) {
        return Err(bad(CMD, "a reference point lies on the point itself"));
    }
    let th = 0.002 * scale;
    ensure_layer(s, LAYER_TIES, 6)?;
    add(s, &space, LAYER_TIES, EntityKind::Circle(Circle { center: Vec3::new(at.x, at.y, 0.0), radius: 0.6 * th }))?;
    for (r, d) in refs.iter().zip(&distances) {
        add(s, &space, LAYER_TIES, EntityKind::Line(Line { a: Vec3::new(at.x, at.y, 0.0), b: Vec3::new(r.x, r.y, 0.0) }))?;
        let mid = Vec2::new((at.x + r.x) / 2.0, (at.y + r.y) / 2.0);
        let ang = (r.y - at.y).atan2(r.x - at.x);
        let ang = if ang.abs() > std::f64::consts::FRAC_PI_2 { ang + std::f64::consts::PI } else { ang };
        let up = Vec2::new(-ang.sin(), ang.cos());
        text(
            s,
            &space,
            LAYER_TIES,
            Vec2::new(mid.x + up.x * 0.4 * th, mid.y + up.y * 0.4 * th),
            th,
            format!("{d:.2}").replace('.', ","),
            ang,
            (HAlign::Center, VAlign::Bottom),
        )?;
    }
    // The point's card, on the side away from the reference points.
    let k = refs.len() as f64;
    let mean = refs.iter().fold(Vec2::ZERO, |a, r| Vec2::new(a.x + (r.x - at.x) / k, a.y + (r.y - at.y) / k));
    let side = if mean.x > 0.0 { -1.0 } else { 1.0 };
    let frame = Frame::of(&[at.xy()]);
    let (y, x) = frame.to_yx(at.xy());
    let mut card = Vec::new();
    if let Some((num, _)) = &known {
        card.push(format!("Bod {num}"));
    }
    card.push(format!("Y = {y:.2}"));
    card.push(format!("X = {x:.2}"));
    if let Some((_, q)) = &known {
        card.push(format!("Z = {:.2}", q.z));
    }
    let align = if side > 0.0 { HAlign::Left } else { HAlign::Right };
    for (i, l) in card.into_iter().enumerate() {
        text(
            s,
            &space,
            LAYER_TIES,
            Vec2::new(at.x + side * 2.0 * th, at.y - (i as f64 + 1.0) * 1.6 * th),
            th * 0.9,
            l.replace('.', ","),
            0.0,
            (align, VAlign::Middle),
        )?;
    }
    Ok(json!({ "distances": distances, "message": format!("Tie sketch with {} distances", distances.len()) }))
}

// --- prompts -------------------------------------------------------------------------------------

#[derive(Default)]
struct ProtM {
    a: Option<Vec2>,
    b: Option<Vec2>,
    da: Option<f64>,
    db: Option<f64>,
}

impl Interactive for ProtM {
    fn name(&self) -> &'static str {
        "GEOPROTINANI"
    }
    fn prompt(&self, _s: &Session) -> Prompt {
        match (self.a, self.b, self.da, self.db) {
            (None, ..) => Prompt::new("First known point (A)", Accept::POINT),
            (_, None, ..) => Prompt::new("Second known point (B)", Accept::POINT),
            (_, _, None, _) => Prompt::new("Distance from A", Accept::NUMBER),
            (_, _, _, None) => Prompt::new("Distance from B", Accept::NUMBER),
            _ => Prompt::new("Side of A→B the new point lies on [Left/Right]", Accept::TEXT).default("Left"),
        }
    }
    fn input(&mut self, s: &mut Session, i: Input) -> Result<Step> {
        if matches!(i, Input::Cancel) {
            return Ok(Step::Cancel);
        }
        let typed = match &i {
            Input::Text(t) | Input::Keyword(t) => Some(t.trim().to_lowercase()),
            _ => None,
        };
        match (self.a, self.b, self.da, self.db) {
            (None, ..) => {
                if let Input::Point(q) = i {
                    self.a = Some(q);
                }
            }
            (_, None, ..) => {
                if let Input::Point(q) = i {
                    self.b = Some(q);
                }
            }
            (_, _, None, _) | (_, _, _, None) => match typed.as_deref().and_then(typed_number).filter(|v| *v > 0.0) {
                Some(v) if self.da.is_none() => self.da = Some(v),
                Some(v) => self.db = Some(v),
                None => s.echo("Enter the measured distance in metres."),
            },
            (Some(a), Some(b), Some(da), Some(db)) => {
                let side = match typed.as_deref() {
                    None if matches!(i, Input::Enter) => "left",
                    Some(t) if t.starts_with('l') || t.starts_with('v') && t.contains("lev") => "left",
                    Some(t) if t.starts_with('r') || t.starts_with('p') || t.contains("prav") => "right",
                    _ => {
                        s.echo("Enter Left or Right.");
                        return Ok(Step::Continue);
                    }
                };
                let r = run_geoprotinani(s, &json!({ "a": [a.x, a.y], "b": [b.x, b.y], "da": da, "db": db, "side": side }));
                say(s, r);
                return Ok(Step::Done);
            }
        }
        Ok(Step::Continue)
    }
}

#[derive(Default)]
struct OrtoM {
    a: Option<Vec2>,
    b: Option<Vec2>,
}

impl Interactive for OrtoM {
    fn name(&self) -> &'static str {
        "GEOORTO"
    }
    fn prompt(&self, _s: &Session) -> Prompt {
        match (self.a, self.b) {
            (None, _) => Prompt::new("Start of the measuring line (A)", Accept::POINT),
            (_, None) => Prompt::new("End of the measuring line (B)", Accept::POINT),
            _ => Prompt::new("Point: number station offset (+ right, − left), Enter to finish", Accept::TEXT),
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
            (Some(a), Some(b), Input::Text(t) | Input::Keyword(t)) if !t.trim().is_empty() => {
                let r = run_geoorto(s, &json!({ "a": [a.x, a.y], "b": [b.x, b.y], "measurements": t }));
                say(s, r);
                Ok(Step::Continue)
            }
            (Some(_), Some(_), Input::Enter | Input::Text(_) | Input::Keyword(_)) => Ok(Step::Done),
            _ => Ok(Step::Continue),
        }
    }
}

#[derive(Default)]
struct ObloukM {
    pts: Vec<Vec2>,
    radius: Option<f64>,
}

impl Interactive for ObloukM {
    fn name(&self) -> &'static str {
        "GEOOBLOUK"
    }
    fn prompt(&self, _s: &Session) -> Prompt {
        match (self.pts.len(), self.radius) {
            (0, _) => Prompt::new("Point on the incoming tangent", Accept::POINT),
            (1, _) => Prompt::new("Vertex (where the tangents meet)", Accept::POINT),
            (2, _) => Prompt::new("Point on the outgoing tangent", Accept::POINT),
            (_, None) => Prompt::new("Radius", Accept::NUMBER),
            _ => Prompt::new("Transition curve length (0 for a plain arc)", Accept::NUMBER).default("0"),
        }
    }
    fn input(&mut self, s: &mut Session, i: Input) -> Result<Step> {
        if matches!(i, Input::Cancel) {
            return Ok(Step::Cancel);
        }
        if self.pts.len() < 3 {
            if let Input::Point(q) = i {
                self.pts.push(q);
            }
            return Ok(Step::Continue);
        }
        let typed = match &i {
            Input::Text(t) | Input::Keyword(t) => typed_number(t),
            _ => None,
        };
        let Some(radius) = self.radius else {
            match typed.filter(|v| *v > 0.0) {
                Some(v) => self.radius = Some(v),
                None => s.echo("Enter the radius in metres."),
            }
            return Ok(Step::Continue);
        };
        let transition = match (&i, typed) {
            (Input::Enter, _) => 0.0,
            (_, Some(v)) if v >= 0.0 => v,
            _ => {
                s.echo("Enter the transition length in metres, or 0.");
                return Ok(Step::Continue);
            }
        };
        let [a, v, b] = self.pts.as_slice() else { return Ok(Step::Done) };
        let r = run_geooblouk(s, &json!({ "from": [a.x, a.y], "vertex": [v.x, v.y], "to": [b.x, b.y], "radius": radius, "transition": transition }));
        say(s, r);
        Ok(Step::Done)
    }
}

#[derive(Default)]
struct MistoM {
    point: Option<Vec2>,
    refs: Vec<Vec2>,
}

impl Interactive for MistoM {
    fn name(&self) -> &'static str {
        "GEOMISTOPIS"
    }
    fn prompt(&self, _s: &Session) -> Prompt {
        match self.point {
            None => Prompt::new("The point to tie", Accept::POINT),
            Some(_) => Prompt::new("Reference point (Enter when done)", Accept::POINT),
        }
    }
    fn input(&mut self, s: &mut Session, i: Input) -> Result<Step> {
        match (self.point, i) {
            (_, Input::Cancel) => Ok(Step::Cancel),
            (None, Input::Point(q)) => {
                self.point = Some(q);
                Ok(Step::Continue)
            }
            (Some(_), Input::Point(q)) => {
                self.refs.push(q);
                if self.refs.len() < 8 { Ok(Step::Continue) } else { self.finish(s) }
            }
            (Some(_), Input::Enter) => self.finish(s),
            _ => Ok(Step::Continue),
        }
    }
}

impl MistoM {
    fn finish(&mut self, s: &mut Session) -> Result<Step> {
        if let (Some(q), false) = (self.point, self.refs.is_empty()) {
            let refs: Vec<Value> = self.refs.iter().map(|r| json!([r.x, r.y])).collect();
            let r = run_geomistopis(s, &json!({ "point": [q.x, q.y], "refs": refs }));
            say(s, r);
        }
        Ok(Step::Done)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn survey(s: &mut Session) {
        s.execute("geobody", &json!({ "text": "1 1000 2000 300\n2 1100 2000 301\n", "sjtsk": false })).unwrap();
    }

    #[test]
    fn intersection_from_distances_and_angles() {
        // A (0,0), B (100,0): the 3-4-5 point (36, 48) is 60 from A and 80 from B, on the left.
        let (a, b) = (Vec2::new(0.0, 0.0), Vec2::new(100.0, 0.0));
        let q = from_distances(a, b, 60.0, 80.0, 1.0).unwrap();
        assert!((q.x - 36.0).abs() < 1e-9 && (q.y - 48.0).abs() < 1e-9, "{q:?}");
        let q = from_distances(a, b, 60.0, 80.0, -1.0).unwrap();
        assert!((q.y + 48.0).abs() < 1e-9);
        assert!(from_distances(a, b, 10.0, 20.0, 1.0).is_none());
        // The same point by its angles: at A atan(48/36), at B atan(48/64).
        let q = from_angles(a, b, (48.0f64 / 36.0).atan(), (48.0f64 / 64.0).atan(), 1.0).unwrap();
        assert!((q.x - 36.0).abs() < 1e-9 && (q.y - 48.0).abs() < 1e-9, "{q:?}");
        assert!(from_angles(a, b, 2.0, 2.0, 1.0).is_none());

        let mut s = Session::new();
        survey(&mut s);
        let r = s.execute("geoprotinani", &json!({ "a": "1", "b": "2", "da": 60, "db": 80, "number": "50" })).unwrap();
        // Local frame: Y is the drawing's x.
        assert!((r["point"]["y"].as_f64().unwrap() - 1036.0).abs() < 1e-6 && (r["point"]["x"].as_f64().unwrap() - 2048.0).abs() < 1e-6, "{r}");
        assert!((r["angle"].as_f64().unwrap() - 100.0).abs() < 1e-6, "{r}");
        // The new point is in the drawing under its number.
        assert!(s.execute("geocisla", &json!({ "find": "50" })).is_ok());
        let r = s
            .execute("geoprotinani", &json!({ "a": "1", "b": "2", "method": "uhly", "alfa": 50, "beta": 50, "side": "right", "add": false }))
            .unwrap();
        assert!((r["at"][0].as_f64().unwrap() - 1050.0).abs() < 1e-6 && (r["at"][1].as_f64().unwrap() - 1950.0).abs() < 1e-6, "{r}");
    }

    #[test]
    fn orthogonal_points() {
        let mut s = Session::new();
        survey(&mut s);
        // Along A→B (east): 30 m on and 5 m to the right (south); 40 m on and 2 m to the left.
        let r = s.execute("geoorto", &json!({ "a": "1", "b": "2", "measurements": "10 30 5\n11 40,00 -2\n" })).unwrap();
        let pts = r["points"].as_array().unwrap();
        assert!((pts[0]["y"].as_f64().unwrap() - 1030.0).abs() < 1e-9 && (pts[0]["x"].as_f64().unwrap() - 1995.0).abs() < 1e-9, "{r}");
        assert!((pts[1]["x"].as_f64().unwrap() - 2002.0).abs() < 1e-9, "{r}");
        // The tape read 100.10 m on a 100 m line: everything shrinks by that ratio.
        let r = s.execute("geoorto", &json!({ "a": "1", "b": "2", "measured": 100.1, "measurements": "12 100.1 0", "add": false })).unwrap();
        assert!((r["points"][0]["y"].as_f64().unwrap() - 1100.0).abs() < 1e-9, "{r}");
        assert!((r["difference"].as_f64().unwrap() - 0.1).abs() < 1e-9);
    }

    #[test]
    fn a_plain_arc_has_the_textbook_elements() {
        // A right angle with R = 100: T = 100, arc = 50π, external = 100(√2 − 1).
        let c = Curve::new(Vec2::new(-200.0, 0.0), Vec2::new(0.0, 0.0), Vec2::new(0.0, 200.0), 100.0, 0.0).unwrap();
        assert!((c.tangent - 100.0).abs() < 1e-9 && (c.arc - 50.0 * std::f64::consts::PI).abs() < 1e-9, "{c:?}");
        let mid = c.at(c.length() / 2.0);
        assert!((mid.x.hypot(mid.y) - 100.0 * (2f64.sqrt() - 1.0)).abs() < 1e-9, "{mid:?}");
        assert!((c.at(0.0).x + 100.0).abs() < 1e-9 && (c.at(c.length()).y - 100.0).abs() < 1e-9);
        // Every point is R from the centre (-100, 100).
        for i in 0..=10 {
            let q = c.at(c.length() * i as f64 / 10.0);
            assert!(((q.x + 100.0).hypot(q.y - 100.0) - 100.0).abs() < 1e-9, "{q:?}");
        }
    }

    #[test]
    fn transitions_join_smoothly_and_mirror() {
        let c = Curve::new(Vec2::new(-500.0, 0.0), Vec2::new(0.0, 0.0), Vec2::new(300.0, -400.0), 200.0, 60.0).unwrap();
        let total = c.length();
        // No jumps anywhere along the curve, the three joins included.
        let n = 2000;
        let mut prev = c.at(0.0);
        for i in 1..=n {
            let q = c.at(total * i as f64 / n as f64);
            let d = (q.x - prev.x).hypot(q.y - prev.y);
            assert!((d - total / n as f64).abs() < 1e-4, "step {i}: {d}");
            prev = q;
        }
        // The shift is about L² / (24 R) = 0.75 m.
        assert!((c.shift - 0.75).abs() < 0.01, "{}", c.shift);
        // The middle of the curve lies on the bisector: as far from both tangent points.
        let mid = c.at(total / 2.0);
        let (da, db) = ((mid.x - c.start.x).hypot(mid.y - c.start.y), (mid.x - c.end.x).hypot(mid.y - c.end.y));
        assert!((da - db).abs() < 1e-6, "{da} {db}");
        // The circular part keeps its radius.
        let q = c.at(c.transition + c.arc / 3.0);
        assert!(((q.x - c.centre.x).hypot(q.y - c.centre.y) - 200.0).abs() < 1e-6);
        assert!(Curve::new(Vec2::new(-1.0, 0.0), Vec2::ZERO, Vec2::new(1.0, 0.01), 100.0, 50.0).is_err());
    }

    #[test]
    fn the_curve_command_and_tie_sketch() {
        let mut s = Session::new();
        let r =
            s.execute("geooblouk", &json!({ "from": [-200, 0], "vertex": [0, 0], "to": [0, 200], "radius": 100, "step": 50, "start": 120 })).unwrap();
        assert!((r["alpha"].as_f64().unwrap() - 100.0).abs() < 1e-9, "{r}");
        let names: Vec<&str> = r["points"].as_array().unwrap().iter().map(|p| p["name"].as_str().unwrap()).collect();
        // TK, KK, KT and detail points at 150, 200, 250 (the curve runs 120 to 277.08).
        assert_eq!(names, ["TK", "KK", "KT", "", "", ""], "{r}");
        let r = s
            .execute("geooblouk", &json!({ "from": [-200, 0], "vertex": [0, 0], "to": [0, 200], "radius": 100, "transition": 40, "step": 0 }))
            .unwrap();
        assert_eq!(r["points"].as_array().unwrap().len(), 5);
        assert!(r["shift"].as_f64().unwrap() > 0.6);

        survey(&mut s);
        let r = s.execute("geomistopis", &json!({ "point": "1", "refs": [[1003, 2004], "2"] })).unwrap();
        assert!((r["distances"][0].as_f64().unwrap() - 5.0).abs() < 1e-9 && (r["distances"][1].as_f64().unwrap() - 100.0).abs() < 1e-9, "{r}");
        let texts: Vec<String> = s
            .doc()
            .unwrap()
            .model
            .iter()
            .filter(|e| e.common.layer == LAYER_TIES)
            .filter_map(|e| if let EntityKind::Text(t) = &e.kind { Some(t.value.clone()) } else { None })
            .collect();
        assert!(
            texts.contains(&"5,00".to_string()) && texts.contains(&"Bod 1".to_string()) && texts.contains(&"Z = 300,00".to_string()),
            "{texts:?}"
        );
    }

    #[test]
    fn hostile_input_is_refused_not_panicking() {
        let mut s = Session::new();
        survey(&mut s);
        for (c, p) in [
            ("geoprotinani", json!({})),
            ("geoprotinani", json!({ "a": "1", "b": "1", "da": 5, "db": 5 })),
            ("geoprotinani", json!({ "a": "1", "b": "2", "da": 5, "db": 5 })),
            ("geoprotinani", json!({ "a": "1", "b": "2", "da": -5, "db": 5 })),
            ("geoprotinani", json!({ "a": "1", "b": "2", "da": 60, "db": 80, "side": "up" })),
            ("geoprotinani", json!({ "a": "1", "b": "2", "method": "uhly", "alfa": 150, "beta": 150 })),
            ("geoprotinani", json!({ "a": "1", "b": "2", "method": "x" })),
            ("geoorto", json!({ "a": "1", "b": "2" })),
            ("geoorto", json!({ "a": "1", "b": "1", "measurements": "5 1 1" })),
            ("geoorto", json!({ "a": "1", "b": "2", "measurements": "5 x 1" })),
            ("geoorto", json!({ "a": "1", "b": "2", "measurements": "5 1 1", "measured": 0 })),
            ("geooblouk", json!({})),
            ("geooblouk", json!({ "from": [0, 0], "vertex": [0, 0], "to": [1, 1], "radius": 10 })),
            ("geooblouk", json!({ "from": [0, 0], "vertex": [10, 0], "to": [20, 0], "radius": 10 })),
            ("geooblouk", json!({ "from": [0, 0], "vertex": [10, 0], "to": [10, 10], "radius": -1 })),
            ("geooblouk", json!({ "from": [0, 0], "vertex": [10, 0], "to": [10, 10], "radius": 10, "step": 1e-9 })),
            ("geomistopis", json!({ "point": "1" })),
            ("geomistopis", json!({ "point": "1", "refs": [] })),
            ("geomistopis", json!({ "point": "1", "refs": ["1"] })),
            ("geomistopis", json!({ "point": "1", "refs": ["9"] })),
        ] {
            assert!(s.execute(c, &p).is_err(), "{c} {p}");
        }
        let _ = clothoid(f64::NAN, 1.0, 1.0);
        let _ = Curve::new(Vec2::new(f64::NAN, 0.0), Vec2::ZERO, Vec2::new(1.0, 1.0), 1.0, 0.0);
        let _ = orthogonal(Vec2::ZERO, Vec2::ZERO, 1.0, 1.0, 1.0);
    }
}
