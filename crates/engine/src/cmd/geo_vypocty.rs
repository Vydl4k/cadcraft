//! Survey add-on: coordinate calculations.
//!
//! - GEOPOLARNI (polar method): new points from a station, an orientation and measured
//!   horizontal angles and distances (and height differences), added like GEOBODY's points.
//! - GEOVYTYC (setting out): the inverse, the angle and distance from a station to each point.
//! - GEOHELMERT: the similarity (Helmert) transformation that fits identical points of two
//!   systems by least squares, its scale, rotation and residuals, applied to objects if asked.
//! - GEOCISLA: find a point by its number, or renumber points by an offset.
//!
//! Angles are in gon by default (400 to a turn), measured clockwise like a theodolite's; bearings
//! are clockwise from +X, the geodetic way. Points are named by their GEO_cisla numbers or given as
//! coordinates; S-JTSK drawings (x = −Y, y = −X) are recognised by their coordinates.

use cadcraft_doc::{EntityKind, Handle, Space};
use cadcraft_geom::{Mat3, Vec2, Vec3};
use serde_json::{Value, json};

use super::geo::{LAYER_NUMBERS, parse};
use super::geo_calc::say;
use super::geo_pohled::numbered_points;
use super::{CommandSpec, bad, str_param};
use crate::{Accept, Input, Interactive, Prompt, Result, Session, Step};

/// Measurements or identical points handled at once.
const MAX_ROWS: usize = 100_000;

pub(super) fn specs() -> Vec<CommandSpec> {
    vec![
        CommandSpec::new("geopolarni", "Polar Method...", run_geopolarni)
            .menu(&["Geo", "Calculations", "Polar Method..."])
            .alias(&["geopolar"])
            .params("{station: point number or [x, y, z?], orientation: point number or [x, y], measurements: text, one line per point \"number angle distance [height difference]\", orientationAngle?: angle read on the orientation (default 0), units?: \"gon\" (default) | \"deg\", add?: bool, draw the points like GEOBODY (default true), scale?: (default 500)} → {points: [{number, y, x, z?}], text, message}")
            .interactive(|_| Ok(Box::new(TextM::new("GEOPOLARNI", "geopolarni")))),
        CommandSpec::new("geovytyc", "Setting Out...", run_geovytyc)
            .menu(&["Geo", "Calculations", "Setting Out..."])
            .alias(&["geostakeout"])
            .params("{station: point number or [x, y], orientation: point number or [x, y], points?: [numbers] (default: every numbered point but the two), units?: \"gon\" | \"deg\"} → {rows: [{number, angle, distance}], text, message}"),
        CommandSpec::new("geohelmert", "Helmert Transformation...", run_geohelmert)
            .menu(&["Geo", "Calculations", "Helmert Transformation..."])
            .alias(&["geotransform"])
            .params("{source: coordinate list (number Y X), target: coordinate list, matched by number — or pairs: [{from: [x, y], to: [x, y]}] in drawing coordinates; apply?: handles to transform (or true for the selection)} → {scale, rotation (gon), translation, residuals: [{number?, dx, dy}], meanError, transformed, message}"),
        CommandSpec::new("geocisla", "Find or Renumber Points...", run_geocisla)
            .menu(&["Geo", "Points", "Find or Renumber Points..."])
            .alias(&["georenumber", "geofind"])
            .params("{find?: point number — selects it and returns its coordinates; or offset: added to numeric point numbers, from?/to?: only numbers in this range} → {found?: {x, y, z}, renumbered?, message}")
            .interactive(|_| Ok(Box::new(TextM::new("GEOCISLA", "geocisla")))),
    ]
}

/// Geodetic Y, X of a drawing point, and back.
#[derive(Clone, Copy)]
struct Frame {
    sjtsk: bool,
}

impl Frame {
    fn of(pts: &[Vec2]) -> Frame {
        Frame { sjtsk: !pts.is_empty() && pts.iter().all(|q| q.x < -300_000.0 && q.y < -900_000.0) }
    }
    fn to_yx(self, p: Vec2) -> (f64, f64) {
        if self.sjtsk { (-p.x, -p.y) } else { (p.x, p.y) }
    }
}

/// Bearing from `a` to `b` in radians, clockwise from +X, in 0..2π.
pub fn bearing(a: (f64, f64), b: (f64, f64)) -> f64 {
    let t = (b.0 - a.0).atan2(b.1 - a.1);
    if t < 0.0 { t + std::f64::consts::TAU } else { t }
}

fn unit(p: &Value) -> Result<f64> {
    match str_param(p, "units").unwrap_or("gon") {
        u if u.eq_ignore_ascii_case("gon") || u.eq_ignore_ascii_case("g") => Ok(std::f64::consts::PI / 200.0),
        u if u.eq_ignore_ascii_case("deg") || u.eq_ignore_ascii_case("°") => Ok(std::f64::consts::PI / 180.0),
        u => Err(bad("geo", format!("unknown angle units \"{u}\": use gon or deg"))),
    }
}

/// A point given by its number or as coordinates, in the drawing.
fn point_ref(s: &Session, space: &Space, v: Option<&Value>, key: &str, cmd: &str) -> Result<Vec3> {
    let v = v.ok_or_else(|| bad(cmd, format!("`{key}` (a point number or [x, y]) is required")))?;
    if let Some(a) = v.as_array() {
        let n: Vec<f64> = a.iter().filter_map(Value::as_f64).filter(|x| x.is_finite() && x.abs() < 1e9).collect();
        return match n.as_slice() {
            [x, y] => Ok(Vec3::new(*x, *y, 0.0)),
            [x, y, z] => Ok(Vec3::new(*x, *y, *z)),
            _ => Err(bad(cmd, format!("`{key}` must be [x, y] or [x, y, z]"))),
        };
    }
    let num = match v {
        Value::String(t) => t.trim().to_string(),
        Value::Number(n) => n.to_string(),
        _ => return Err(bad(cmd, format!("`{key}` must be a point number or [x, y]"))),
    };
    numbered_points(s, space, None)?
        .into_iter()
        .find(|(c, _)| *c == num)
        .map(|(_, q)| q)
        .ok_or_else(|| bad(cmd, format!("no point {num} in the drawing (GEO_body)")))
}

fn number(t: &str) -> Option<f64> {
    t.trim().replace(',', ".").parse::<f64>().ok().filter(|v| v.is_finite())
}

fn run_geopolarni(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "geopolarni";
    let space = s.space();
    let station = point_ref(s, &space, p.get("station"), "station", CMD)?;
    let orient = point_ref(s, &space, p.get("orientation"), "orientation", CMD)?;
    let k = unit(p)?;
    let o_read = p.get("orientationAngle").and_then(Value::as_f64).unwrap_or(0.0);
    if !o_read.is_finite() {
        return Err(bad(CMD, "`orientationAngle` must be an angle"));
    }
    let text = str_param(p, "measurements")
        .ok_or_else(|| bad(CMD, "`measurements` is required: one line per point, \"number angle distance [height difference]\""))?;
    let frame = Frame::of(&[station.xy(), orient.xy()]);
    let (sy, sx) = frame.to_yx(station.xy());
    let base = bearing((sy, sx), frame.to_yx(orient.xy()));
    if (orient.x - station.x).hypot(orient.y - station.y) < 1e-9 {
        return Err(bad(CMD, "the station and the orientation are the same point"));
    }
    let mut out = Vec::new();
    let mut list = String::new();
    for line in text.lines().filter(|l| !l.trim().is_empty()) {
        if out.len() >= MAX_ROWS {
            return Err(bad(CMD, format!("at most {MAX_ROWS} points at once")));
        }
        let f: Vec<&str> = line.split([' ', '\t', ';']).filter(|t| !t.is_empty()).collect();
        let (Some(num), Some(a), Some(d)) = (f.first(), f.get(1).and_then(|t| number(t)), f.get(2).and_then(|t| number(t))) else {
            return Err(bad(CMD, format!("can't read \"{}\": expected \"number angle distance [height difference]\"", line.trim())));
        };
        if !(0.0..=1e6).contains(&d) || a.abs() > 1e6 {
            return Err(bad(CMD, format!("\"{}\": distance or angle out of range", line.trim())));
        }
        let dh = f.get(3).and_then(|t| number(t));
        let sigma = base + (a - o_read) * k;
        let (y, x) = (sy + d * sigma.sin(), sx + d * sigma.cos());
        let z = dh.map(|h| station.z + h);
        match z {
            Some(z) => list.push_str(&format!("{num} {y:.3} {x:.3} {z:.3}\n")),
            None => list.push_str(&format!("{num} {y:.3} {x:.3}\n")),
        }
        out.push(json!({ "number": num, "y": y, "x": x, "z": z }));
    }
    if out.is_empty() {
        return Err(bad(CMD, "no measurements"));
    }
    if p.get("add").and_then(Value::as_bool).unwrap_or(true) {
        let scale = p.get("scale").cloned().unwrap_or(json!(500));
        s.execute("geobody", &json!({ "text": list, "columns": "PYXZ", "sjtsk": frame.sjtsk, "scale": scale }))?;
    }
    Ok(json!({ "points": out, "text": list, "message": format!("{} points by the polar method", out.len()) }))
}

fn run_geovytyc(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "geovytyc";
    let space = s.space();
    let station = point_ref(s, &space, p.get("station"), "station", CMD)?;
    let orient = point_ref(s, &space, p.get("orientation"), "orientation", CMD)?;
    let k = unit(p)?;
    let all = numbered_points(s, &space, None)?;
    let wanted: Option<Vec<String>> = match p.get("points") {
        None | Some(Value::Null) => None,
        Some(Value::Array(a)) => Some(a.iter().map(|v| v.as_str().map_or_else(|| v.to_string(), str::to_string)).collect()),
        Some(_) => return Err(bad(CMD, "`points` must be a list of point numbers")),
    };
    let frame = Frame::of(&[station.xy(), orient.xy()]);
    let s_yx = frame.to_yx(station.xy());
    let base = bearing(s_yx, frame.to_yx(orient.xy()));
    let turn = std::f64::consts::TAU;
    let mut rows = Vec::new();
    let mut list = String::new();
    for (num, q) in &all {
        let picked = match &wanted {
            Some(w) => w.iter().any(|n| n == num),
            None => q.xy() != station.xy() && q.xy() != orient.xy(),
        };
        if !picked {
            continue;
        }
        let t = frame.to_yx(q.xy());
        let d = (t.0 - s_yx.0).hypot(t.1 - s_yx.1);
        let a = (bearing(s_yx, t) - base).rem_euclid(turn) / k;
        list.push_str(&format!("{num} {a:.4} {d:.3}\n"));
        rows.push(json!({ "number": num, "angle": a, "distance": d }));
    }
    if rows.is_empty() {
        return Err(bad(CMD, "no points to set out"));
    }
    Ok(json!({ "rows": rows, "text": list, "message": format!("Setting out {} points (number, angle from the orientation, distance)", rows.len()) }))
}

/// The similarity transformation taking `from` to `to` by least squares: scale·rotation `a`, `b`
/// (x' = a·x − b·y + tx, y' = b·x + a·y + ty).
pub fn helmert(from: &[Vec2], to: &[Vec2]) -> Option<Mat3> {
    let n = from.len();
    if n < 2 || to.len() != n {
        return None;
    }
    let k = n as f64;
    let c = |v: &[Vec2]| v.iter().fold(Vec2::ZERO, |a, p| Vec2::new(a.x + p.x / k, a.y + p.y / k));
    let (cf, ct) = (c(from), c(to));
    let (mut num_a, mut num_b, mut den) = (0.0, 0.0, 0.0);
    for (f, t) in from.iter().zip(to) {
        let (fx, fy, tx, ty) = (f.x - cf.x, f.y - cf.y, t.x - ct.x, t.y - ct.y);
        num_a += fx * tx + fy * ty;
        num_b += fx * ty - fy * tx;
        den += fx * fx + fy * fy;
    }
    if !(den.is_finite() && den > 1e-18) {
        return None;
    }
    let (a, b) = (num_a / den, num_b / den);
    let m = Mat3 { a, b, c: -b, d: a, e: ct.x - (a * cf.x - b * cf.y), f: ct.y - (b * cf.x + a * cf.y) };
    [m.a, m.b, m.e, m.f].iter().all(|v| v.is_finite()).then_some(m)
}

fn list_points(text: &str, cmd: &str) -> Result<Vec<(String, Vec2)>> {
    let (bods, _) = parse(text, "PYX").map_err(|e| bad(cmd, e))?;
    if bods.len() > MAX_ROWS {
        return Err(bad(cmd, format!("at most {MAX_ROWS} points")));
    }
    let sj = super::geo::looks_sjtsk(&bods);
    Ok(bods.into_iter().map(|b| (b.cislo, if sj { Vec2::new(-b.y, -b.x) } else { Vec2::new(b.y, b.x) })).collect())
}

fn run_geohelmert(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "geohelmert";
    let mut names: Vec<Option<String>> = Vec::new();
    let (mut from, mut to) = (Vec::new(), Vec::new());
    if let Some(pairs) = p.get("pairs") {
        let a = pairs.as_array().ok_or_else(|| bad(CMD, "`pairs` must be a list of {from, to}"))?;
        for v in a.iter().take(MAX_ROWS + 1) {
            let (Some(f), Some(t)) = (v.get("from").and_then(super::point_value), v.get("to").and_then(super::point_value)) else {
                return Err(bad(CMD, "each pair needs `from` and `to` points"));
            };
            from.push(f);
            to.push(t);
            names.push(v.get("name").and_then(Value::as_str).map(str::to_string));
        }
    } else {
        let src = list_points(str_param(p, "source").ok_or_else(|| bad(CMD, "give `source` and `target` lists, or `pairs`"))?, CMD)?;
        let tgt = list_points(str_param(p, "target").ok_or_else(|| bad(CMD, "`target` is required"))?, CMD)?;
        for (n, f) in &src {
            if let Some((_, t)) = tgt.iter().find(|(m, _)| m == n) {
                from.push(*f);
                to.push(*t);
                names.push(Some(n.clone()));
            }
        }
    }
    if from.len() > MAX_ROWS {
        return Err(bad(CMD, format!("at most {MAX_ROWS} identical points")));
    }
    if from.len() < 2 || from.iter().chain(&to).any(|p| !(p.x.is_finite() && p.y.is_finite())) {
        return Err(bad(CMD, "at least 2 identical points (with the same number in both lists) are needed"));
    }
    let m = helmert(&from, &to).ok_or_else(|| bad(CMD, "the identical points coincide: no transformation fits them"))?;
    let mut sum = 0.0;
    let residuals: Vec<Value> = from
        .iter()
        .zip(&to)
        .zip(&names)
        .map(|((f, t), n)| {
            let q = m.apply(*f);
            let (dx, dy) = (t.x - q.x, t.y - q.y);
            sum += dx * dx + dy * dy;
            json!({ "number": n, "dx": dx, "dy": dy, "distance": dx.hypot(dy) })
        })
        .collect();
    let mean_error = if from.len() > 2 { Some((sum / (2.0 * from.len() as f64 - 4.0)).sqrt()) } else { None };
    let scale = m.a.hypot(m.b);
    let rotation = (m.b.atan2(m.a) * 200.0 / std::f64::consts::PI).rem_euclid(400.0);
    let targets: Vec<Handle> = match p.get("apply") {
        None | Some(Value::Null) | Some(Value::Bool(false)) => Vec::new(),
        Some(Value::Bool(true)) => s.selection(),
        Some(v) => super::targets(s, &json!({ "handles": v }))?,
    };
    let transformed = if targets.is_empty() { 0 } else { super::modify::transform_entities(s, &targets, &m, false)?.len() };
    Ok(json!({
        "scale": scale, "rotation": rotation, "translation": [m.e, m.f], "residuals": residuals, "meanError": mean_error, "transformed": transformed,
        "message": format!(
            "Helmert from {} points: scale {:.6} ({:+.1} ppm), rotation {rotation:.4} gon{}{}",
            from.len(), scale, (scale - 1.0) * 1e6,
            mean_error.map_or_else(String::new, |e| format!(", mean error {e:.3} m")),
            if transformed > 0 { format!("; {transformed} objects transformed") } else { String::new() }
        ),
    }))
}

fn run_geocisla(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "geocisla";
    let space = s.space();
    if let Some(v) = p.get("find") {
        let num = match v {
            Value::String(t) => t.trim().to_string(),
            Value::Number(n) => n.to_string(),
            _ => return Err(bad(CMD, "`find` must be a point number")),
        };
        // The point and its number label, selected.
        let q = point_ref(s, &space, Some(&json!(num)), "find", CMD)?;
        let d = s.doc()?;
        let st = d.space(&space).ok_or_else(|| bad(CMD, "no drawing space"))?;
        let hits: Vec<Handle> = st
            .iter()
            .filter(|e| match &e.kind {
                EntityKind::Point(pt) => pt.p.xy() == q.xy() && pt.p.z == q.z,
                EntityKind::Text(t) => e.common.layer.eq_ignore_ascii_case(LAYER_NUMBERS) && t.value.trim() == num,
                _ => false,
            })
            .map(|e| e.handle)
            .collect();
        s.set_selection(hits);
        return Ok(json!({ "found": { "x": q.x, "y": q.y, "z": q.z }, "message": format!("Point {num}: {:.3}, {:.3}, {:.3}", q.x, q.y, q.z) }));
    }
    let offset = p.get("offset").and_then(Value::as_i64).ok_or_else(|| bad(CMD, "give `find` (a point number) or `offset` (to renumber by)"))?;
    let from = p.get("from").and_then(Value::as_i64).unwrap_or(i64::MIN);
    let to = p.get("to").and_then(Value::as_i64).unwrap_or(i64::MAX);
    let labels: Vec<(Handle, i64)> = {
        let d = s.doc()?;
        let st = d.space(&space).ok_or_else(|| bad(CMD, "no drawing space"))?;
        st.iter()
            .filter(|e| e.common.layer.eq_ignore_ascii_case(LAYER_NUMBERS))
            .filter_map(|e| if let EntityKind::Text(t) = &e.kind { t.value.trim().parse::<i64>().ok().map(|n| (e.handle, n)) } else { None })
            .filter(|(_, n)| (from..=to).contains(n))
            .collect()
    };
    let mut changed = 0;
    for (h, n) in labels {
        let Some(new) = n.checked_add(offset) else { return Err(bad(CMD, format!("point {n} + {offset} is out of range"))) };
        let d = s.doc_mut()?;
        d.modify_entity(h, |e: &mut cadcraft_doc::Entity| {
            if let EntityKind::Text(t) = &mut e.kind {
                t.value = new.to_string();
            }
        })
        .map_err(|e| bad(CMD, e.to_string()))?;
        changed += 1;
    }
    Ok(json!({ "renumbered": changed, "message": format!("{changed} points renumbered by {offset:+}") }))
}

/// One typed line: the measurement list for GEOPOLARNI is too long for a prompt, so it asks for
/// a station, orientation and one point at a time; GEOCISLA asks for a number to find.
struct TextM {
    name: &'static str,
    cmd: &'static str,
    station: Option<String>,
    orientation: Option<String>,
}

impl TextM {
    fn new(name: &'static str, cmd: &'static str) -> TextM {
        TextM { name, cmd, station: None, orientation: None }
    }
}

impl Interactive for TextM {
    fn name(&self) -> &'static str {
        self.name
    }
    fn prompt(&self, _s: &Session) -> Prompt {
        if self.cmd == "geocisla" {
            return Prompt::new("Point number to find", Accept::TEXT);
        }
        match (&self.station, &self.orientation) {
            (None, _) => Prompt::new("Station point number", Accept::TEXT),
            (_, None) => Prompt::new("Orientation point number", Accept::TEXT),
            _ => Prompt::new("Point: number angle distance [height difference], Enter to finish", Accept::TEXT),
        }
    }
    fn input(&mut self, s: &mut Session, i: Input) -> Result<Step> {
        let t = match i {
            Input::Cancel => return Ok(Step::Cancel),
            Input::Enter => return Ok(Step::Done),
            Input::Text(t) | Input::Keyword(t) => t.trim().to_string(),
            _ => return Ok(Step::Continue),
        };
        if t.is_empty() {
            return Ok(Step::Done);
        }
        if self.cmd == "geocisla" {
            let r = run_geocisla(s, &json!({ "find": t }));
            say(s, r);
            return Ok(Step::Done);
        }
        if self.station.is_none() {
            self.station = Some(t);
            return Ok(Step::Continue);
        }
        if self.orientation.is_none() {
            self.orientation = Some(t);
            return Ok(Step::Continue);
        }
        let r = run_geopolarni(s, &json!({ "station": self.station, "orientation": self.orientation, "measurements": t }));
        say(s, r);
        Ok(Step::Continue)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn survey(s: &mut Session) {
        // Station 1 at Y 1000, X 2000 (z 300); orientation 2 due north (+X) of it.
        s.execute("geobody", &json!({ "text": "1 1000 2000 300\n2 1000 2100 301\n", "sjtsk": false })).unwrap();
    }

    #[test]
    fn polar_points_and_setting_out_agree() {
        let mut s = Session::new();
        survey(&mut s);
        // 100 gon (east) 50 m, 1 m up; 200 gon (south) 20 m.
        let r = s.execute("geopolarni", &json!({ "station": "1", "orientation": "2", "measurements": "10 100 50 1\n11 200,0000 20\n" })).unwrap();
        let pts = r["points"].as_array().unwrap();
        assert!((pts[0]["y"].as_f64().unwrap() - 1050.0).abs() < 1e-9 && (pts[0]["x"].as_f64().unwrap() - 2000.0).abs() < 1e-9, "{r}");
        assert!((pts[0]["z"].as_f64().unwrap() - 301.0).abs() < 1e-9);
        assert!((pts[1]["x"].as_f64().unwrap() - 1980.0).abs() < 1e-9, "{r}");
        // The new points are in the drawing with their numbers.
        let r = s.execute("geovytyc", &json!({ "station": "1", "orientation": "2", "points": ["10", "11"] })).unwrap();
        let rows = r["rows"].as_array().unwrap();
        assert!((rows[0]["angle"].as_f64().unwrap() - 100.0).abs() < 1e-9 && (rows[0]["distance"].as_f64().unwrap() - 50.0).abs() < 1e-9, "{r}");
        assert!((rows[1]["angle"].as_f64().unwrap() - 200.0).abs() < 1e-9, "{r}");
        // Degrees and an orientation reading of 50 gon.
        let r = s
            .execute(
                "geopolarni",
                &json!({ "station": [1000, 2000], "orientation": [1000, 2100], "measurements": "12 90 10", "units": "deg", "add": false }),
            )
            .unwrap();
        assert!((r["points"][0]["y"].as_f64().unwrap() - 1010.0).abs() < 1e-9);
        let r = s
            .execute("geopolarni", &json!({ "station": "1", "orientation": "2", "orientationAngle": 50, "measurements": "13 150 10", "add": false }))
            .unwrap();
        assert!((r["points"][0]["y"].as_f64().unwrap() - 1010.0).abs() < 1e-9, "{r}");
    }

    #[test]
    fn helmert_recovers_a_similarity() {
        // Rotate 30°, scale 1.0002, shift: the transformation is found exactly.
        let (c, sn) = ((30f64).to_radians().cos() * 1.0002, (30f64).to_radians().sin() * 1.0002);
        let from = [Vec2::new(0.0, 0.0), Vec2::new(100.0, 0.0), Vec2::new(0.0, 100.0), Vec2::new(70.0, 40.0)];
        let to: Vec<Vec2> = from.iter().map(|p| Vec2::new(c * p.x - sn * p.y + 500.0, sn * p.x + c * p.y - 20.0)).collect();
        let m = helmert(&from, &to).unwrap();
        assert!((m.a - c).abs() < 1e-12 && (m.b - sn).abs() < 1e-12 && (m.e - 500.0).abs() < 1e-9 && (m.f + 20.0).abs() < 1e-9);
        let mut s = Session::new();
        let list = |v: &[Vec2]| v.iter().enumerate().map(|(i, p)| format!("{} {} {}\n", i + 1, p.x, p.y)).collect::<String>();
        let r = s.execute("geohelmert", &json!({ "source": list(&from), "target": list(&to) })).unwrap();
        assert!((r["scale"].as_f64().unwrap() - 1.0002).abs() < 1e-9, "{r}");
        assert!(r["meanError"].as_f64().unwrap() < 1e-6, "{r}");
        // Applied to a line in the drawing.
        let l = s.execute("line", &json!({ "points": [[0, 0], [100, 0]] })).unwrap();
        let h = l["handle"].as_str().map(str::to_string).or_else(|| l["handles"][0].as_str().map(str::to_string));
        if let Some(h) = h {
            let r = s.execute("geohelmert", &json!({ "pairs": from.iter().zip(&to).map(|(f, t)| json!({ "from": [f.x, f.y], "to": [t.x, t.y] })).collect::<Vec<_>>(), "apply": [h] })).unwrap();
            assert_eq!(r["transformed"], 1, "{r}");
        }
        assert!(helmert(&[Vec2::ZERO, Vec2::ZERO], &[Vec2::ZERO, Vec2::ZERO]).is_none());
    }

    #[test]
    fn find_and_renumber() {
        let mut s = Session::new();
        survey(&mut s);
        let r = s.execute("geocisla", &json!({ "find": "2" })).unwrap();
        assert!((r["found"]["y"].as_f64().unwrap() - 2100.0).abs() < 1e-9, "{r}");
        assert_eq!(s.selection().len(), 2);
        let r = s.execute("geocisla", &json!({ "offset": 1000, "from": 2 })).unwrap();
        assert_eq!(r["renumbered"], 1);
        assert!(s.execute("geocisla", &json!({ "find": "1002" })).is_ok());
        assert!(s.execute("geocisla", &json!({ "find": "2" })).is_err());
    }

    #[test]
    fn hostile_input_is_refused_not_panicking() {
        let mut s = Session::new();
        survey(&mut s);
        for (c, p) in [
            ("geopolarni", json!({})),
            ("geopolarni", json!({ "station": "1", "orientation": "1", "measurements": "5 1 1" })),
            ("geopolarni", json!({ "station": "1", "orientation": "2", "measurements": "5 x 1" })),
            ("geopolarni", json!({ "station": "1", "orientation": "2", "measurements": "5 1 -3" })),
            ("geopolarni", json!({ "station": "9", "orientation": "2", "measurements": "5 1 1" })),
            ("geopolarni", json!({ "station": "1", "orientation": "2", "measurements": "5 1 1", "units": "rad" })),
            ("geovytyc", json!({ "station": [0], "orientation": "2" })),
            ("geohelmert", json!({})),
            ("geohelmert", json!({ "pairs": [{ "from": [0, 0], "to": [1, 1] }] })),
            ("geohelmert", json!({ "pairs": "x" })),
            ("geocisla", json!({})),
            ("geocisla", json!({ "offset": i64::MAX })),
        ] {
            assert!(s.execute(c, &p).is_err(), "{c} {p}");
        }
    }
}
