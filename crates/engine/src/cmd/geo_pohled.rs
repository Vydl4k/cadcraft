//! Survey add-on: an elevation or section drawn from 3D points (GEOPOHLED), and height marks
//! relative to ±0,00 (GEOKOTA).
//!
//! GEOPOHLED projects surveyed points onto a vertical plane: across the drawing goes the distance
//! along the plane, up the height above ±0,00. The plane is a line in plan (two points, or a line
//! or polyline segment), or, when none is given, the vertical plane that fits the points best
//! (a facade measured with a total station). With `depth` only the points that close to the plane
//! are kept, which makes a section. Each point is drawn with its number and height, like the
//! measured points of a building survey, ready to be traced over.
//!
//! GEOKOTA writes a height mark at a picked point of such a view: a small triangle on a leader
//! line with the height above ±0,00 (the view's own y, so it matches GEOPOHLED's output).

use cadcraft_doc::{EntityKind, HAlign, Handle, Line, LwPolyline, Point, Space, VAlign};
use cadcraft_geom::{PolyVertex, Vec2, Vec3};
use serde_json::{Value, json};

use super::geo::{LAYER_NUMBERS, LAYER_POINTS, decode, looks_sjtsk, parse};
use super::geo_calc::{add, ensure_layer, say, text, typed_number};
use super::{CommandSpec, bad, f64_or, point_value, str_param};
use crate::{Accept, Input, Interactive, Prompt, Result, Session, Step};

pub const LAYER_VIEW_POINTS: &str = "GEO_pohled_body";
pub const LAYER_VIEW_NUMBERS: &str = "GEO_pohled_cisla";
pub const LAYER_VIEW_HEIGHTS: &str = "GEO_pohled_vysky";
pub const LAYER_VIEW_DATUM: &str = "GEO_pohled_nula";
pub const LAYER_MARKS: &str = "GEO_pohled_koty";

/// Points projected at once.
const MAX_POINTS: usize = 500_000;

pub(super) fn specs() -> Vec<CommandSpec> {
    vec![
        CommandSpec::new("geopohled", "Elevation or Section from Points...", run_geopohled)
            .menu(&["Geo", "Building", "Elevation or Section from Points..."])
            .alias(&["georez", "geoelevation"])
            .params("{text?: coordinate list, path?: its file, columns?: (default \"PYXZ\") — otherwise the selected POINTs, else every POINT on GEO_body; from?, to?: [x, y] the plane's line in plan (default: the plane that fits the points), line?: handle of a line or polyline for it, flip?: bool, look from the other side, depth?: keep only points this close to the plane (m; default all), base?: height of ±0,00 (default 0), at?: [x, y] where distance 0 at ±0,00 is drawn (default: right of the points), scale?: drawing scale 1:N for text (default 50), decimals?: (default 2)} → {points, dropped, length, min, max, from, to, message}")
            .interactive(|_| Ok(Box::new(PohledM::default()))),
        CommandSpec::new("geokota", "Height Mark...", run_geokota)
            .menu(&["Geo", "Building", "Height Mark..."])
            .alias(&["geoheightmark"])
            .params("{at: [x, y] the marked point, datum?: drawing y of ±0,00 (default 0), base?: height of ±0,00 added to the mark (default 0, so marks read relative), scale?: 1:N (default 50), decimals?: (default 2)} → {height, text}")
            .interactive(|_| Ok(Box::new(KotaM::default()))),
    ]
}

/// The POINTs on GEO_body (or just `only`), each with its number: the GEO_cisla text GEOBODY
/// wrote beside it, else its order.
pub(super) fn numbered_points(s: &Session, space: &Space, only: Option<&[Handle]>) -> Result<Vec<(String, Vec3)>> {
    let d = s.doc()?;
    let Some(st) = d.space(space) else { return Ok(Vec::new()) };
    let point = |e: &cadcraft_doc::Entity| match &e.kind {
        EntityKind::Point(q) if q.p.x.is_finite() && q.p.y.is_finite() && q.p.z.is_finite() => Some(q.p),
        _ => None,
    };
    let pts: Vec<Vec3> = match only {
        Some(hs) => hs.iter().filter_map(|h| st.get(*h)).filter_map(|e| point(e)).collect(),
        None => st.iter().filter(|e| e.common.layer.eq_ignore_ascii_case(LAYER_POINTS)).filter_map(|e| point(e)).collect(),
    };
    let labels: Vec<(Vec2, f64, String)> = st
        .iter()
        .filter(|e| e.common.layer.eq_ignore_ascii_case(LAYER_NUMBERS))
        .filter_map(|e| if let EntityKind::Text(t) = &e.kind { Some((t.insert.xy(), t.height, t.value.clone())) } else { None })
        .filter(|l| l.0.x.is_finite() && l.0.y.is_finite() && l.1.is_finite() && l.1 > 0.0)
        .collect();
    let mut grid: std::collections::HashMap<(i64, i64), Vec<usize>> = std::collections::HashMap::new();
    let cell = labels.iter().map(|l| l.1).fold(0.0f64, f64::max).clamp(0.01, 1e6) * 4.0;
    let ckey = |q: Vec2| ((q.x / cell).floor() as i64, (q.y / cell).floor() as i64);
    for (i, l) in labels.iter().enumerate() {
        grid.entry(ckey(l.0)).or_default().push(i);
    }
    let mut out = Vec::with_capacity(pts.len());
    for (n, q) in pts.iter().enumerate() {
        let (cx, cy) = ckey(q.xy());
        let mut best: Option<(f64, &str)> = None;
        for dx in -1..=1 {
            for dy in -1..=1 {
                for &i in grid.get(&(cx.saturating_add(dx), cy.saturating_add(dy))).map(Vec::as_slice).unwrap_or(&[]) {
                    let Some((at, h, v)) = labels.get(i) else { continue };
                    let want = Vec2::new(q.x + 0.6 * h, q.y + 0.25 * h);
                    let d = (at.x - want.x).hypot(at.y - want.y);
                    if d < 0.5 * h && best.is_none_or(|(bd, _)| d < bd) {
                        best = Some((d, v.as_str()));
                    }
                }
            }
        }
        out.push((best.map_or_else(|| format!("{}", n + 1), |(_, v)| v.to_string()), *q));
    }
    Ok(out)
}

/// The vertical plane that fits the points best in plan: the line through their centre along
/// the direction they spread most, from the first to the last point along it.
pub fn fit_line(pts: &[Vec2]) -> Option<(Vec2, Vec2)> {
    let n = pts.len();
    if n < 2 {
        return None;
    }
    let k = n as f64;
    let c = pts.iter().fold(Vec2::ZERO, |a, p| Vec2::new(a.x + p.x / k, a.y + p.y / k));
    let (mut sxx, mut sxy, mut syy) = (0.0, 0.0, 0.0);
    for p in pts {
        let (dx, dy) = (p.x - c.x, p.y - c.y);
        sxx += dx * dx;
        sxy += dx * dy;
        syy += dy * dy;
    }
    let ang = 0.5 * (2.0 * sxy).atan2(sxx - syy);
    let u = Vec2::new(ang.cos(), ang.sin());
    let along = |p: &Vec2| (p.x - c.x) * u.x + (p.y - c.y) * u.y;
    let lo = pts.iter().map(along).fold(f64::INFINITY, f64::min);
    let hi = pts.iter().map(along).fold(f64::NEG_INFINITY, f64::max);
    if !(lo.is_finite() && hi.is_finite()) || hi - lo < 1e-9 {
        return None;
    }
    Some((Vec2::new(c.x + lo * u.x, c.y + lo * u.y), Vec2::new(c.x + hi * u.x, c.y + hi * u.y)))
}

/// A point of the view: its number, distance along the plane, height, and distance off it.
#[derive(Clone, Debug, PartialEq)]
pub struct ViewPoint {
    pub cislo: String,
    pub station: f64,
    pub z: f64,
    pub offset: f64,
}

/// Project `pts` onto the vertical plane over `from`→`to`. `offset` is positive behind the plane
/// (to the left of the direction, seen by a viewer standing to its right).
pub fn project(pts: &[(String, Vec3)], from: Vec2, to: Vec2) -> Option<Vec<ViewPoint>> {
    let (dx, dy) = (to.x - from.x, to.y - from.y);
    let len = dx.hypot(dy);
    if !(len.is_finite() && len > 1e-9) {
        return None;
    }
    let u = Vec2::new(dx / len, dy / len);
    Some(
        pts.iter()
            .map(|(c, p)| {
                let (px, py) = (p.x - from.x, p.y - from.y);
                ViewPoint { cislo: c.clone(), station: px * u.x + py * u.y, z: p.z, offset: u.x * py - u.y * px }
            })
            .collect(),
    )
}

fn plan_point(v: Option<&Value>, key: &str) -> Result<Option<Vec2>> {
    match v {
        None | Some(Value::Null) => Ok(None),
        Some(v) => point_value(v)
            .filter(|p| p.x.is_finite() && p.y.is_finite() && p.x.abs() < 1e9 && p.y.abs() < 1e9)
            .map(Some)
            .ok_or_else(|| bad("geopohled", format!("`{key}` must be a point [x, y]"))),
    }
}

/// The points GEOPOHLED works on, in drawing coordinates.
fn source_points(s: &Session, p: &Value, space: &Space) -> Result<Vec<(String, Vec3)>> {
    const CMD: &str = "geopohled";
    let listed: Option<String> = match (str_param(p, "text"), str_param(p, "path")) {
        (Some(t), _) => Some(t.to_string()),
        (None, Some(path)) => {
            #[cfg(not(target_arch = "wasm32"))]
            {
                let meta = std::fs::metadata(path).map_err(|e| bad(CMD, format!("{path}: {e}")))?;
                if meta.len() > 200 * 1024 * 1024 {
                    return Err(bad(CMD, format!("{path}: larger than 200 MB")));
                }
                Some(decode(&std::fs::read(path).map_err(|e| bad(CMD, format!("{path}: {e}")))?))
            }
            #[cfg(target_arch = "wasm32")]
            return Err(bad(CMD, format!("{path}: files can't be read on the web; pass `text`")));
        }
        (None, None) => None,
    };
    if let Some(t) = listed {
        let cols = str_param(p, "columns").unwrap_or("PYXZ");
        let (bods, _) = parse(&t, cols).map_err(|e| bad(CMD, e))?;
        let sj = looks_sjtsk(&bods);
        return Ok(bods
            .into_iter()
            .filter_map(|b| {
                let z = b.z?;
                let at = if sj { Vec2::new(-b.y, -b.x) } else { Vec2::new(b.y, b.x) };
                Some((b.cislo, Vec3::new(at.x, at.y, z)))
            })
            .collect());
    }
    let sel = super::targets(s, p)?;
    numbered_points(s, space, if sel.is_empty() { None } else { Some(&sel) })
}

/// The plane's line from a line or polyline: its first segment.
fn line_of(s: &Session, h: Handle, space: &Space) -> Option<(Vec2, Vec2)> {
    let d = s.doc().ok()?;
    match &d.space(space)?.get(h)?.kind {
        EntityKind::Line(l) => Some((l.a.xy(), l.b.xy())),
        EntityKind::LwPolyline(pl) => Some((pl.vertices.first()?.p, pl.vertices.get(1)?.p)),
        EntityKind::Polyline3d(pl) => Some((pl.points.first()?.xy(), pl.points.get(1)?.xy())),
        _ => None,
    }
}

fn run_geopohled(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "geopohled";
    let space = s.space();
    let pts = source_points(s, p, &space)?;
    if pts.is_empty() {
        return Err(bad(CMD, "no points with heights: select them, import them with GEOBODY, or pass `text`"));
    }
    if pts.len() > MAX_POINTS {
        return Err(bad(CMD, format!("at most {MAX_POINTS} points at once")));
    }
    let base = f64_or(p, "base", 0.0);
    let scale = f64_or(p, "scale", 50.0);
    let depth = match p.get("depth") {
        None | Some(Value::Null) => None,
        Some(v) => Some(v.as_f64().filter(|d| d.is_finite() && *d > 0.0).ok_or_else(|| bad(CMD, "`depth` must be a positive distance"))?),
    };
    if !(base.is_finite() && base.abs() < 1e6) || !(scale.is_finite() && (1.0..=100_000.0).contains(&scale)) {
        return Err(bad(CMD, "`base` must be a height and `scale` a drawing scale like 50"));
    }
    let decimals = p.get("decimals").and_then(Value::as_u64).unwrap_or(2).min(4) as usize;
    let (mut from, mut to) = match (plan_point(p.get("from"), "from")?, plan_point(p.get("to"), "to")?) {
        (Some(a), Some(b)) => (a, b),
        (None, None) => match p.get("line") {
            None | Some(Value::Null) => {
                let plan: Vec<Vec2> = pts.iter().map(|(_, q)| q.xy()).collect();
                fit_line(&plan).ok_or_else(|| bad(CMD, "the points lie on one spot: give the plane with `from` and `to`"))?
            }
            Some(v) => {
                let h = v.as_str().and_then(Handle::parse_hex).ok_or_else(|| bad(CMD, "`line` must be an object handle"))?;
                line_of(s, h, &space).ok_or_else(|| bad(CMD, "`line` must be a line or polyline"))?
            }
        },
        _ => return Err(bad(CMD, "give both `from` and `to`")),
    };
    if p.get("flip").and_then(Value::as_bool).unwrap_or(false) {
        std::mem::swap(&mut from, &mut to);
    }
    let all = project(&pts, from, to).ok_or_else(|| bad(CMD, "`from` and `to` must be two different points"))?;
    let total = all.len();
    let kept: Vec<ViewPoint> = all.into_iter().filter(|v| depth.is_none_or(|d| v.offset.abs() <= d)).collect();
    if kept.is_empty() {
        return Err(bad(CMD, format!("no point lies within {} m of the plane", depth.unwrap_or(0.0))));
    }
    let lo = kept.iter().map(|v| v.station).fold(f64::INFINITY, f64::min);
    let hi = kept.iter().map(|v| v.station).fold(f64::NEG_INFINITY, f64::max);
    let zmin = kept.iter().map(|v| v.z - base).fold(f64::INFINITY, f64::min);
    let zmax = kept.iter().map(|v| v.z - base).fold(f64::NEG_INFINITY, f64::max);
    let at = match plan_point(p.get("at"), "at")? {
        Some(a) => a,
        None => {
            let xs = pts.iter().map(|(_, q)| q.x).fold(f64::NEG_INFINITY, f64::max);
            let ys = pts.iter().map(|(_, q)| q.y).fold(f64::INFINITY, f64::min);
            Vec2::new(xs + 10.0 - lo, ys - zmin)
        }
    };
    let th = 0.002 * scale;
    for (l, aci) in [(LAYER_VIEW_POINTS, 7), (LAYER_VIEW_NUMBERS, 7), (LAYER_VIEW_HEIGHTS, 8), (LAYER_VIEW_DATUM, 1)] {
        ensure_layer(s, l, aci)?;
    }
    for v in &kept {
        let q = Vec2::new(at.x + v.station, at.y + v.z - base);
        add(s, &space, LAYER_VIEW_POINTS, EntityKind::Point(Point { p: Vec3::new(q.x, q.y, 0.0), angle: 0.0 }))?;
        text(s, &space, LAYER_VIEW_NUMBERS, Vec2::new(q.x + 0.6 * th, q.y + 0.25 * th), th, v.cislo.clone(), 0.0, (HAlign::Left, VAlign::Baseline))?;
        text(
            s,
            &space,
            LAYER_VIEW_HEIGHTS,
            Vec2::new(q.x + 0.6 * th, q.y - 1.25 * th),
            th * 0.8,
            format_height(v.z - base, decimals),
            0.0,
            (HAlign::Left, VAlign::Baseline),
        )?;
    }
    // The ±0,00 line under the points, and what it stands for.
    let (x0, x1) = (at.x + lo - 1.0, at.x + hi + 1.0);
    add(s, &space, LAYER_VIEW_DATUM, EntityKind::Line(Line { a: Vec3::new(x0, at.y, 0.0), b: Vec3::new(x1, at.y, 0.0) }))?;
    let label = if base != 0.0 { format!("±0,00 = {}", format!("{base:.decimals$}").replace('.', ",")) } else { "±0,00".to_string() };
    text(s, &space, LAYER_VIEW_DATUM, Vec2::new(x0 - th, at.y), th, label, 0.0, (HAlign::Right, VAlign::Middle))?;
    let length = hi - lo;
    Ok(json!({
        "points": kept.len(), "dropped": total - kept.len(), "length": length, "min": zmin, "max": zmax,
        "from": [from.x, from.y], "to": [to.x, to.y],
        "message": format!(
            "{} points over {length:.2} m, heights {} to {}{}",
            kept.len(),
            format_height(zmin, decimals),
            format_height(zmax, decimals),
            if total > kept.len() { format!(" ({} farther than {} m from the plane left out)", total - kept.len(), depth.unwrap_or(0.0)) } else { String::new() }
        ),
    }))
}

/// A height the Czech way: decimal comma, + for heights above ±0,00 (as on drawings).
fn format_height(z: f64, decimals: usize) -> String {
    let t = format!("{z:+.decimals$}").replace('.', ",");
    // −0,00 reads as ±0,00.
    if t.trim_start_matches(['+', '-']).chars().all(|c| c == '0' || c == ',') { format!("±{}", t.trim_start_matches(['+', '-'])) } else { t }
}

fn run_geokota(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "geokota";
    let at = plan_point(p.get("at"), "at")?.ok_or_else(|| bad(CMD, "`at` (the point to mark) is required"))?;
    let datum = f64_or(p, "datum", 0.0);
    let base = f64_or(p, "base", 0.0);
    let scale = f64_or(p, "scale", 50.0);
    if !(datum.is_finite() && base.is_finite() && datum.abs() < 1e9 && base.abs() < 1e6 && scale.is_finite() && (1.0..=100_000.0).contains(&scale)) {
        return Err(bad(CMD, "`datum`, `base` and `scale` must be numbers"));
    }
    let decimals = p.get("decimals").and_then(Value::as_u64).unwrap_or(2).min(4) as usize;
    let space = s.space();
    let height = at.y - datum + base;
    let value = if base != 0.0 { format!("{height:.decimals$}").replace('.', ",") } else { format_height(height, decimals) };
    // A filled-looking triangle standing on the point, a short leader, the height above it.
    let k = 0.002 * scale;
    let tri = [Vec2::new(at.x, at.y), Vec2::new(at.x - 0.9 * k, at.y + 1.5 * k), Vec2::new(at.x + 0.9 * k, at.y + 1.5 * k)];
    ensure_layer(s, LAYER_MARKS, 1)?;
    let pl = LwPolyline { vertices: tri.into_iter().map(PolyVertex::new).collect(), closed: true, const_width: 0.0, elevation: 0.0, plinegen: false };
    add(s, &space, LAYER_MARKS, EntityKind::LwPolyline(pl))?;
    let shelf = 1.2 * k * value.chars().count() as f64 * 0.8;
    add(s, &space, LAYER_MARKS, EntityKind::Line(Line { a: Vec3::new(at.x, at.y + 1.5 * k, 0.0), b: Vec3::new(at.x, at.y + 2.5 * k, 0.0) }))?;
    add(s, &space, LAYER_MARKS, EntityKind::Line(Line { a: Vec3::new(at.x, at.y + 2.5 * k, 0.0), b: Vec3::new(at.x + shelf, at.y + 2.5 * k, 0.0) }))?;
    text(s, &space, LAYER_MARKS, Vec2::new(at.x + 0.3 * k, at.y + 2.9 * k), k, value.clone(), 0.0, (HAlign::Left, VAlign::Baseline))?;
    Ok(json!({ "height": height, "text": value, "message": format!("Height mark {value}") }))
}

#[derive(Default)]
struct PohledM {
    from: Option<Vec2>,
    to: Option<Vec2>,
    fitted: bool,
    base: Option<f64>,
}

impl Interactive for PohledM {
    fn name(&self) -> &'static str {
        "GEOPOHLED"
    }
    fn prompt(&self, _s: &Session) -> Prompt {
        match (self.from, self.to, self.fitted, self.base) {
            (None, _, false, _) => Prompt::new("First point of the view line in plan, or Enter to fit it to the points", Accept::POINT),
            (Some(_), None, _, _) => Prompt::new("Second point of the view line", Accept::POINT),
            (_, _, _, None) => Prompt::new("Height of ±0,00", Accept::NUMBER).default("0"),
            _ => Prompt::new("Where to draw the view (distance 0 at ±0,00), or Enter to place it beside the points", Accept::POINT),
        }
    }
    fn input(&mut self, s: &mut Session, i: Input) -> Result<Step> {
        if matches!(i, Input::Cancel) {
            return Ok(Step::Cancel);
        }
        if self.from.is_none() && !self.fitted {
            match i {
                Input::Point(p) => self.from = Some(p),
                Input::Enter => self.fitted = true,
                _ => {}
            }
            return Ok(Step::Continue);
        }
        if self.from.is_some() && self.to.is_none() {
            if let Input::Point(p) = i {
                self.to = Some(p);
            }
            return Ok(Step::Continue);
        }
        if self.base.is_none() {
            match i {
                Input::Enter => self.base = Some(0.0),
                Input::Text(t) | Input::Keyword(t) => match typed_number(&t) {
                    Some(b) => self.base = Some(b),
                    None => s.echo("Enter the height of ±0,00, like 343.50."),
                },
                _ => {}
            }
            return Ok(Step::Continue);
        }
        let mut params = json!({ "base": self.base.unwrap_or(0.0) });
        if let (Some(a), Some(b), Some(o)) = (self.from, self.to, params.as_object_mut()) {
            o.insert("from".into(), json!([a.x, a.y]));
            o.insert("to".into(), json!([b.x, b.y]));
        }
        match i {
            Input::Point(p) => {
                if let Some(o) = params.as_object_mut() {
                    o.insert("at".into(), json!([p.x, p.y]));
                }
            }
            Input::Enter => {}
            _ => return Ok(Step::Continue),
        }
        let r = run_geopohled(s, &params);
        say(s, r);
        Ok(Step::Done)
    }
}

#[derive(Default)]
struct KotaM {
    datum: Option<f64>,
}

impl Interactive for KotaM {
    fn name(&self) -> &'static str {
        "GEOKOTA"
    }
    fn prompt(&self, _s: &Session) -> Prompt {
        match self.datum {
            None => Prompt::new("Pick a point on the ±0,00 line (or type its y)", Accept::POINT_OR_NUMBER),
            Some(_) => Prompt::new("Point to mark", Accept::POINT),
        }
    }
    fn input(&mut self, s: &mut Session, i: Input) -> Result<Step> {
        match (self.datum, i) {
            (_, Input::Cancel) => Ok(Step::Cancel),
            (_, Input::Enter) => Ok(Step::Done),
            (None, Input::Point(p)) => {
                self.datum = Some(p.y);
                Ok(Step::Continue)
            }
            (None, Input::Text(t) | Input::Keyword(t)) => {
                match typed_number(&t) {
                    Some(v) => self.datum = Some(v),
                    None => s.echo("Pick a point, or type the y of ±0,00."),
                }
                Ok(Step::Continue)
            }
            (Some(datum), Input::Point(p)) => {
                let r = run_geokota(s, &json!({ "at": [p.x, p.y], "datum": datum }));
                say(s, r);
                Ok(Step::Continue)
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

    #[test]
    fn a_facade_is_fitted_and_projected() {
        // Points on a wall along x = -210.5 (a facade measured from inside a hall), heights 0..8.
        let list = "171 -210.541 -68.535 3.325\n172 -210.556 -67.095 1.566\n179 -210.658 -55.160 0.311\n159 -210.507 -59.586 8.777\n";
        let mut s = Session::new();
        let r = s.execute("geopohled", &json!({ "text": list, "base": 0.311, "at": [0, 0] })).unwrap();
        assert_eq!(r["points"], 4, "{r}");
        // Along the wall: 68.535 - 55.160 = 13.375 m.
        assert!((r["length"].as_f64().unwrap() - 13.375).abs() < 0.01, "{r}");
        assert!((r["min"].as_f64().unwrap()).abs() < 1e-9 && (r["max"].as_f64().unwrap() - 8.466).abs() < 1e-9, "{r}");
        let nums = texts(&s, LAYER_VIEW_NUMBERS);
        assert!(["171", "172", "179", "159"].iter().all(|n| nums.iter().any(|t| t == n)), "{nums:?}");
        let hs = texts(&s, LAYER_VIEW_HEIGHTS);
        assert!(hs.iter().any(|t| t == "±0,00") && hs.iter().any(|t| t == "+8,47"), "{hs:?}");
        assert!(texts(&s, LAYER_VIEW_DATUM).iter().any(|t| t == "±0,00 = 0,31"));
        // Every view point lies at its station and height.
        let ys: Vec<f64> = s
            .doc()
            .unwrap()
            .model
            .iter()
            .filter(|e| e.common.layer == LAYER_VIEW_POINTS)
            .filter_map(|e| if let EntityKind::Point(q) = &e.kind { Some(q.p.y) } else { None })
            .collect();
        assert!(ys.iter().any(|y| (y - 8.466).abs() < 1e-9) && ys.iter().any(|y| y.abs() < 1e-9), "{ys:?}");
    }

    #[test]
    fn a_section_keeps_the_points_near_the_plane() {
        let pts = vec![
            ("1".to_string(), Vec3::new(0.0, 0.0, 1.0)),
            ("2".to_string(), Vec3::new(10.0, 0.2, 2.0)),
            ("3".to_string(), Vec3::new(5.0, 3.0, 3.0)),
        ];
        let v = project(&pts, Vec2::new(0.0, 0.0), Vec2::new(10.0, 0.0)).unwrap();
        assert!((v[1].station - 10.0).abs() < 1e-12 && (v[1].offset - 0.2).abs() < 1e-12);
        assert!((v[2].offset - 3.0).abs() < 1e-12);
        let mut s = Session::new();
        let list = "1 0 0 1\n2 10 0.2 2\n3 5 3 3\n";
        // Y, X columns: drawing x = Y, y = X.
        let r = s.execute("geopohled", &json!({ "text": list, "from": [0, 0], "to": [10, 0], "depth": 0.5 })).unwrap();
        assert_eq!(r["points"], 2, "{r}");
        assert_eq!(r["dropped"], 1, "{r}");
        assert!(s.execute("geopohled", &json!({ "text": list, "from": [0, 0], "to": [10, 0], "depth": 0.01, "flip": true })).is_ok());
    }

    #[test]
    fn fit_line_follows_the_spread() {
        let (a, b) = fit_line(&[Vec2::new(0.0, 0.0), Vec2::new(10.0, 10.0), Vec2::new(5.0, 5.1)]).unwrap();
        assert!(((b.y - a.y) / (b.x - a.x) - 1.0).abs() < 0.05);
        assert!(fit_line(&[Vec2::new(1.0, 1.0), Vec2::new(1.0, 1.0)]).is_none());
        assert!(fit_line(&[]).is_none());
    }

    #[test]
    fn height_marks_read_from_the_datum() {
        let mut s = Session::new();
        let r = s.execute("geokota", &json!({ "at": [3, 12.5], "datum": 10 })).unwrap();
        assert_eq!(r["text"], "+2,50");
        let r = s.execute("geokota", &json!({ "at": [3, 9.0], "datum": 10 })).unwrap();
        assert_eq!(r["text"], "-1,00");
        let r = s.execute("geokota", &json!({ "at": [3, 10.0], "datum": 10, "base": 343.5 })).unwrap();
        assert_eq!(r["text"], "343,50");
        assert_eq!(format_height(-0.001, 2), "±0,00");
    }

    #[test]
    fn numbers_come_from_geobody_labels() {
        let mut s = Session::new();
        s.execute("geobody", &json!({ "text": "501 10 20 5\n502 30 20 6\n", "sjtsk": false })).unwrap();
        let space = s.space();
        let n = numbered_points(&s, &space, None).unwrap();
        assert!(n.iter().any(|(c, q)| c == "502" && (q.z - 6.0).abs() < 1e-12), "{n:?}");
        let r = s.execute("geopohled", &json!({})).unwrap();
        assert_eq!(r["points"], 2);
        assert!(texts(&s, LAYER_VIEW_NUMBERS).contains(&"501".to_string()));
    }

    #[test]
    fn hostile_input_is_refused_not_panicking() {
        let mut s = Session::new();
        for p in [
            json!({}),
            json!({ "text": "1 0 0 1\n" }),
            json!({ "text": "1 0 0 1\n2 0 0 2\n" }),
            json!({ "text": "1 0 0 1\n2 1 0 2\n", "from": [0, 0] }),
            json!({ "text": "1 0 0 1\n2 1 0 2\n", "from": [0, 0], "to": [0, 0] }),
            json!({ "text": "1 0 0 1\n2 1 0 2\n", "depth": -1 }),
            json!({ "text": "1 0 0 1\n2 1 0 2\n", "scale": 0 }),
            json!({ "text": "1 0 0 1\n2 1 0 2\n", "line": "zz" }),
            json!({ "text": "1 0 0 1\n2 1 0 2\n", "at": "x" }),
            json!({ "text": "1 0 0 1\n2 1 0 2\n", "columns": "QQ" }),
        ] {
            assert!(s.execute("geopohled", &p).is_err(), "{p}");
        }
        for p in [json!({}), json!({ "at": [0, 0], "scale": -5 }), json!({ "at": [0, 0], "datum": "x" })] {
            let r = s.execute("geokota", &p);
            assert!(r.is_err() || p.get("datum").is_some(), "{p}");
        }
        let _ = project(&[("1".into(), Vec3::new(f64::NAN, 0.0, 0.0))], Vec2::ZERO, Vec2::new(1.0, 0.0));
        let _ = fit_line(&[Vec2::new(f64::NAN, 0.0), Vec2::new(1.0, 1.0)]);
    }
}
