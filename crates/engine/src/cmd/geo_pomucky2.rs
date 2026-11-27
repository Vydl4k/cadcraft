//! Survey add-on: checks and land take.
//!
//! - GEOKRIZENI finds where selected lines cross (utility lines, say): each crossing gets a
//!   mark, and where both lines have heights (3D polylines, lines with Z), the height of each
//!   there and the clearance between them.
//! - GEOZABORY works out a land take: for each parcel (found round its number from the parcel
//!   boundaries, as the VFK import draws them) the area inside the take boundary, listed, written
//!   as a table into the drawing and, if asked, saved as a CSV file for a spreadsheet.
//! - GEOCHRANICKA draws a casing pipe along a utility line between two places: its two walls at
//!   the given diameter and its ends.

use cadcraft_doc::{EntityKind, HAlign, Handle, Line, VAlign};
use cadcraft_geom::{Vec2, Vec3, region::enclosing_loop};
use serde_json::{Value, json};

use super::geo_calc::{add, bbox, clip_to_tri, ear_clip, ensure_layer, one_target, overlaps, path_of, say, signed_area, text, typed_number};
use super::geo_nastaveni as nast;
use super::geo_rek::offset;
use super::geo_vfk::{LAYER_BOUNDARIES, LAYER_NUMBERS as KN_NUMBERS};
use super::{CommandSpec, bad, point_value, str_param, targets};
use crate::{Accept, Input, Interactive, Prompt, Result, Session, Step};

pub const LAYER_CROSSINGS: &str = "GEO_krizeni";
pub const LAYER_TAKE: &str = "GEO_zabory";
pub const LAYER_CASING: &str = "GEO_chranicky";
/// Segments compared at once.
const MAX_SEGMENTS: usize = 50_000;
/// Crossings reported at once.
const MAX_CROSSINGS: usize = 10_000;
/// Parcels in one land take.
const MAX_PARCELS: usize = 5_000;

pub(super) fn specs() -> Vec<CommandSpec> {
    vec![
        CommandSpec::new("geokrizeni", "Check Crossings...", run_krizeni)
            .menu(&["Geo", "Tools", "Check Crossings..."])
            .alias(&["geocrossings"])
            .params("{handles?: lines, polylines and 3D polylines (default the selection), mark?: bool draw marks (default true)} → {crossings: [{at, z1, z2, clearance}], message}")
            .interactive(|_| Ok(Box::new(SelM { name: "GEOKRIZENI", msg: "Select the lines to check", cmd: "geokrizeni" }))),
        CommandSpec::new("geozabory", "Land Take...", run_zabory)
            .menu(&["Geo", "Tools", "Land Take..."])
            .alias(&["geolandtake"])
            .params("{take: handle of the closed take boundary, layers?: boundary layers (default [\"KN_hranice_parcel\", \"GEO_parcely\"]), numbers?: number layers (default [\"KN_parcelni_cisla\", \"GEO_parcely\"]), at?: [x, y] where the table goes, csv?: file path} → {parcels: [{parcel, area, take}], total, message}")
            .interactive(|_| Ok(Box::new(ZabM::default()))),
        CommandSpec::new("geochranicka", "Casing Pipe...", run_chranicka)
            .menu(&["Geo", "Labels", "Casing Pipe..."])
            .alias(&["geocasing"])
            .params("{line: handle of the utility line, from: [x, y], to: [x, y] the casing's ends (near the line), diameter: m} → {length, message}")
            .interactive(|_| Ok(Box::new(ChrM::default()))),
    ]
}

/// The pieces of an object as 3D segments (heights where it has them).
fn segments_of(kind: &EntityKind) -> Vec<(Vec3, Vec3, bool)> {
    let pairs = |v: Vec<Vec3>, closed: bool, has_z: bool| {
        let n = v.len();
        let mut out: Vec<(Vec3, Vec3, bool)> = v.windows(2).filter_map(|w| Some((*w.first()?, *w.get(1)?, has_z))).collect();
        if closed
            && n > 2
            && let (Some(a), Some(b)) = (v.last(), v.first())
        {
            out.push((*a, *b, has_z));
        }
        out
    };
    match kind {
        EntityKind::Line(l) => vec![(l.a, l.b, l.a.z != 0.0 || l.b.z != 0.0)],
        EntityKind::LwPolyline(pl) => {
            pairs(pl.vertices.iter().map(|v| Vec3::new(v.p.x, v.p.y, pl.elevation)).collect(), pl.closed, pl.elevation != 0.0)
        }
        EntityKind::Polyline3d(pl) => pairs(pl.points.clone(), pl.closed, true),
        _ => Vec::new(),
    }
}

/// Where two segments cross in plan, as the fraction along each.
fn cross(a: Vec3, b: Vec3, c: Vec3, d: Vec3) -> Option<(f64, f64)> {
    let (rx, ry, sx, sy) = (b.x - a.x, b.y - a.y, d.x - c.x, d.y - c.y);
    let den = rx * sy - ry * sx;
    if den.abs() < 1e-12 {
        return None;
    }
    let t = ((c.x - a.x) * sy - (c.y - a.y) * sx) / den;
    let u = ((c.x - a.x) * ry - (c.y - a.y) * rx) / den;
    ((-1e-12..=1.0 + 1e-12).contains(&t) && (-1e-12..=1.0 + 1e-12).contains(&u)).then_some((t, u))
}

fn run_krizeni(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "geokrizeni";
    let hs = targets(s, p)?;
    if hs.len() < 2 {
        return Err(bad(CMD, "select at least two lines"));
    }
    let space = s.space();
    let mut objs: Vec<(Handle, Vec<(Vec3, Vec3, bool)>, (Vec2, Vec2))> = Vec::new();
    let mut total = 0;
    {
        let d = s.doc()?;
        let st = d.space(&space).ok_or_else(|| bad(CMD, "no drawing space"))?;
        for h in &hs {
            let Some(e) = st.get(*h) else { continue };
            let segs: Vec<_> = segments_of(&e.kind)
                .into_iter()
                .filter(|(a, b, _)| [a, b].iter().all(|q| q.x.is_finite() && q.y.is_finite() && q.z.is_finite()))
                .collect();
            if segs.is_empty() {
                continue;
            }
            total += segs.len();
            if total > MAX_SEGMENTS {
                return Err(bad(CMD, format!("more than {MAX_SEGMENTS} segments; select fewer lines")));
            }
            let pts: Vec<Vec2> = segs.iter().flat_map(|(a, b, _)| [a.xy(), b.xy()]).collect();
            objs.push((*h, segs, bbox(&pts)));
        }
    }
    let mut found: Vec<(Vec2, Option<f64>, Option<f64>)> = Vec::new();
    'outer: for i in 0..objs.len() {
        for j in i + 1..objs.len() {
            let (Some((_, s1, b1)), Some((_, s2, b2))) = (objs.get(i), objs.get(j)) else { continue };
            if !overlaps(*b1, *b2) {
                continue;
            }
            for (a, b, z1) in s1 {
                for (c, d, z2) in s2 {
                    if let Some((t, u)) = cross(*a, *b, *c, *d) {
                        let at = Vec2::new(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y));
                        // A crossing at a shared vertex is counted once.
                        if found.iter().any(|f| (f.0.x - at.x).hypot(f.0.y - at.y) < 1e-6) {
                            continue;
                        }
                        let za = z1.then(|| a.z + t * (b.z - a.z));
                        let zb = z2.then(|| c.z + u * (d.z - c.z));
                        found.push((at, za, zb));
                        if found.len() >= MAX_CROSSINGS {
                            break 'outer;
                        }
                    }
                }
            }
        }
    }
    let n = nast::load(s);
    let h = n.height(&n.notes);
    if p.get("mark").and_then(Value::as_bool).unwrap_or(true) && !found.is_empty() {
        ensure_layer(s, LAYER_CROSSINGS, 1)?;
        for (at, za, zb) in &found {
            add(s, &space, LAYER_CROSSINGS, EntityKind::Circle(cadcraft_doc::Circle { center: Vec3::new(at.x, at.y, 0.0), radius: 0.6 * h }))?;
            if let (Some(a), Some(b)) = (za, zb) {
                text(
                    s,
                    &space,
                    LAYER_CROSSINGS,
                    Vec2::new(at.x + 0.8 * h, at.y + 0.8 * h),
                    0.8 * h,
                    format!("Δh {:.2}", (a - b).abs()),
                    0.0,
                    (HAlign::Left, VAlign::Bottom),
                )?;
            }
        }
    }
    let list: Vec<Value> = found
        .iter()
        .map(|(at, za, zb)| json!({ "at": [at.x, at.y], "z1": za, "z2": zb, "clearance": za.zip(*zb).map(|(a, b)| (a - b).abs()) }))
        .collect();
    let closest = found.iter().filter_map(|(_, a, b)| a.zip(*b).map(|(a, b)| (a - b).abs())).min_by(f64::total_cmp);
    Ok(json!({
        "crossings": list,
        "message": format!(
            "{} crossings{}",
            found.len(),
            closest.map(|c| format!("; the smallest clearance {c:.3} m")).unwrap_or_default()
        ),
    }))
}

/// Counter-clockwise triangle.
fn ccw(t: [Vec2; 3]) -> [Vec2; 3] {
    let [a, b, c] = t;
    if (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x) < 0.0 { [a, c, b] } else { t }
}

/// The area two simple polygons share.
fn shared_area(p: &[Vec2], q: &[Vec2]) -> f64 {
    if !overlaps(bbox(p), bbox(q)) {
        return 0.0;
    }
    let (tp, tq) = (ear_clip(p), ear_clip(q));
    let mut area = 0.0;
    for a in &tp {
        let ba = bbox(a);
        for b in &tq {
            if overlaps(ba, bbox(b)) {
                let c = clip_to_tri(a, &ccw(*b));
                if c.len() >= 3 {
                    area += signed_area(&c).abs();
                }
            }
        }
    }
    area
}

fn layer_list(p: &Value, key: &str, default: &[&str]) -> Vec<String> {
    match p.get(key).and_then(Value::as_array) {
        Some(a) => a.iter().filter_map(Value::as_str).map(str::to_string).take(64).collect(),
        None => default.iter().map(|s| s.to_string()).collect(),
    }
}

fn cz(v: f64) -> String {
    format!("{v:.2}").replace('.', ",")
}

fn run_zabory(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "geozabory";
    let take_h = one_target(s, p, "take", CMD)?;
    let space = s.space();
    let (mut take, closed) = path_of(s, take_h, &space).ok_or_else(|| bad(CMD, "the take boundary must be a polyline"))?;
    let joined = take.first().zip(take.last()).is_some_and(|(a, b)| (a.x - b.x).hypot(a.y - b.y) < 1e-9);
    if !closed && !joined {
        return Err(bad(CMD, "the take boundary must be a closed polyline"));
    }
    if joined && take.len() > 3 {
        take.pop();
    }
    if take.len() < 3 || signed_area(&take).abs() < 1e-12 {
        return Err(bad(CMD, "the take boundary has no area"));
    }
    let bl = layer_list(p, "layers", &[LAYER_BOUNDARIES, super::geo_popis::LAYER_PARCELS]);
    let nl = layer_list(p, "numbers", &[KN_NUMBERS, super::geo_popis::LAYER_PARCELS]);
    let tb = bbox(&take);
    let (polys, numbers) = {
        let d = s.doc()?;
        let st = d.space(&space).ok_or_else(|| bad(CMD, "no drawing space"))?;
        let on = |layer: &str, list: &[String]| list.iter().any(|l| l.eq_ignore_ascii_case(layer));
        let mut polys: Vec<Vec<Vec2>> = Vec::new();
        let mut numbers: Vec<(String, Vec2)> = Vec::new();
        for e in st.iter() {
            if on(&e.common.layer, &bl) && e.handle != take_h {
                let v: Vec<Vec2> = match &e.kind {
                    EntityKind::Line(l) => vec![l.a.xy(), l.b.xy()],
                    EntityKind::LwPolyline(pl) => {
                        let mut v: Vec<Vec2> = pl.vertices.iter().map(|v| v.p).collect();
                        if pl.closed
                            && let Some(f) = v.first().copied()
                        {
                            v.push(f);
                        }
                        v
                    }
                    EntityKind::Polyline3d(pl) => {
                        let mut v: Vec<Vec2> = pl.points.iter().map(|v| v.xy()).collect();
                        if pl.closed
                            && let Some(f) = v.first().copied()
                        {
                            v.push(f);
                        }
                        v
                    }
                    _ => Vec::new(),
                };
                if v.len() >= 2 && v.iter().all(|q| q.x.is_finite() && q.y.is_finite()) {
                    polys.push(v);
                }
            }
            if on(&e.common.layer, &nl)
                && let EntityKind::Text(t) = &e.kind
            {
                let at = t.align_pt.filter(|_| t.halign != HAlign::Left || t.valign != VAlign::Baseline).unwrap_or(t.insert).xy();
                let v = t.value.trim();
                if !v.is_empty() && at.x.is_finite() && at.y.is_finite() {
                    numbers.push((v.to_string(), at));
                }
            }
        }
        (polys, numbers)
    };
    if polys.len() > 200_000 {
        return Err(bad(CMD, "too many boundary lines"));
    }
    // Boundaries near the take are enough to close the parcels it touches.
    let reach = ((tb.1.x - tb.0.x) + (tb.1.y - tb.0.y)).max(1.0);
    let near = (Vec2::new(tb.0.x - reach, tb.0.y - reach), Vec2::new(tb.1.x + reach, tb.1.y + reach));
    let polys: Vec<Vec<Vec2>> = polys.into_iter().filter(|v| overlaps(bbox(v), near)).collect();
    let mut rows: Vec<(String, f64, f64)> = Vec::new();
    for (num, at) in numbers.iter().filter(|(_, at)| overlaps((*at, *at), near)) {
        if rows.len() >= MAX_PARCELS {
            break;
        }
        let Some(parcel) = enclosing_loop(&polys, *at, 1e-4) else { continue };
        let take_area = shared_area(&parcel, &take);
        if take_area > 1e-6 {
            rows.push((num.clone(), signed_area(&parcel).abs(), take_area));
        }
    }
    rows.sort_by(|a, b| a.0.cmp(&b.0));
    rows.dedup_by(|a, b| a.0 == b.0);
    if rows.is_empty() {
        return Err(bad(CMD, "no parcel with a number on the boundary layers lies under the take"));
    }
    let total: f64 = rows.iter().map(|r| r.2).sum();
    let mut out = String::from("parcela;výměra parcely [m2];zábor [m2]\n");
    for (n, a, t) in &rows {
        out.push_str(&format!("{n};{};{}\n", cz(*a), cz(*t)));
    }
    out.push_str(&format!("celkem;;{}\n", cz(total)));
    if let Some(path) = str_param(p, "csv") {
        write_csv(path, &out)?;
    }
    if let Some(at) = p.get("at") {
        let at = point_value(at).ok_or_else(|| bad(CMD, "`at` must be [x, y]"))?;
        let n = nast::load(s);
        let h = n.height(&n.notes);
        ensure_layer(s, LAYER_TAKE, 2)?;
        for (i, line) in out.lines().enumerate() {
            let y = at.y - 1.6 * h * i as f64;
            for (k, cell) in line.split(';').enumerate() {
                let x = at.x + [0.0, 12.0, 26.0].get(k).copied().unwrap_or(0.0) * h;
                text(s, &space, LAYER_TAKE, Vec2::new(x, y), h, cell.to_string(), 0.0, (HAlign::Left, VAlign::Baseline))?;
            }
        }
    }
    let list: Vec<Value> = rows.iter().map(|(n, a, t)| json!({ "parcel": n, "area": a, "take": t })).collect();
    let mut msg = vec![format!("Land take {} m² on {} parcels", cz(total), rows.len())];
    msg.extend(rows.iter().map(|(n, a, t)| format!("  {n}: {} m² of {} m²", cz(*t), cz(*a))));
    Ok(json!({ "parcels": list, "total": total, "message": msg.join("\n") }))
}

#[cfg(not(target_arch = "wasm32"))]
fn write_csv(path: &str, text: &str) -> Result<()> {
    // With a byte order mark, so a spreadsheet reads the Czech letters right.
    let mut bytes = b"\xEF\xBB\xBF".to_vec();
    bytes.extend_from_slice(text.as_bytes());
    std::fs::write(path, bytes).map_err(|e| bad("geozabory", format!("{path}: {e}")))
}

#[cfg(target_arch = "wasm32")]
fn write_csv(_path: &str, _text: &str) -> Result<()> {
    Err(bad("geozabory", "files are not available on the web"))
}

/// The distance along `path` of the point on it nearest `q`.
fn station(path: &[Vec2], q: Vec2) -> Option<f64> {
    let mut run = 0.0;
    let mut best: Option<(f64, f64)> = None;
    for w in path.windows(2) {
        let (a, b) = (*w.first()?, *w.get(1)?);
        let (dx, dy) = (b.x - a.x, b.y - a.y);
        let l = dx.hypot(dy);
        if l < 1e-12 {
            continue;
        }
        let t = (((q.x - a.x) * dx + (q.y - a.y) * dy) / (l * l)).clamp(0.0, 1.0);
        let d = (a.x + t * dx - q.x).hypot(a.y + t * dy - q.y);
        if best.is_none_or(|(bd, _)| d < bd) {
            best = Some((d, run + t * l));
        }
        run += l;
    }
    best.map(|b| b.1)
}

/// The part of `path` from distance `s0` to `s1` along it.
fn sub_path(path: &[Vec2], s0: f64, s1: f64) -> Vec<Vec2> {
    let mut out = Vec::new();
    let mut run = 0.0;
    for w in path.windows(2) {
        let (Some(&a), Some(&b)) = (w.first(), w.get(1)) else { continue };
        let l = (b.x - a.x).hypot(b.y - a.y);
        let at = |d: f64| {
            let t = ((d - run) / l.max(1e-12)).clamp(0.0, 1.0);
            Vec2::new(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y))
        };
        if run + l >= s0 && run <= s1 {
            if out.is_empty() {
                out.push(at(s0));
            }
            out.push(at(s1.min(run + l)));
        }
        run += l;
    }
    out.dedup_by(|a, b| (a.x - b.x).hypot(a.y - b.y) < 1e-9);
    out
}

fn run_chranicka(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "geochranicka";
    let h = one_target(s, p, "line", CMD)?;
    let space = s.space();
    let (mut path, closed) = path_of(s, h, &space).ok_or_else(|| bad(CMD, "pick a line or polyline"))?;
    if closed && let Some(f) = path.first().copied() {
        path.push(f);
    }
    let get = |k: &str| p.get(k).and_then(point_value).ok_or_else(|| bad(CMD, format!("`{k}`: [x, y] is required")));
    let (a, b) = (get("from")?, get("to")?);
    let d = p
        .get("diameter")
        .and_then(|v| v.as_f64().or_else(|| v.as_str().and_then(typed_number)))
        .filter(|x| x.is_finite() && *x > 0.0 && *x < 100.0)
        .ok_or_else(|| bad(CMD, "`diameter` must be metres"))?;
    let (s0, s1) =
        (station(&path, a).ok_or_else(|| bad(CMD, "the line has no length"))?, station(&path, b).ok_or_else(|| bad(CMD, "the line has no length"))?);
    let (s0, s1) = (s0.min(s1), s0.max(s1));
    if s1 - s0 < 1e-6 {
        return Err(bad(CMD, "the casing has no length"));
    }
    let axis = sub_path(&path, s0, s1);
    let (l, r) = (
        offset(&axis, false, d / 2.0).ok_or_else(|| bad(CMD, "the casing has no length"))?,
        offset(&axis, false, -d / 2.0).ok_or_else(|| bad(CMD, "the casing has no length"))?,
    );
    ensure_layer(s, LAYER_CASING, 6)?;
    let pl = |v: &[Vec2]| {
        EntityKind::LwPolyline(cadcraft_doc::LwPolyline {
            vertices: v.iter().copied().map(cadcraft_geom::PolyVertex::new).collect(),
            closed: false,
            const_width: 0.0,
            elevation: 0.0,
            plinegen: false,
        })
    };
    add(s, &space, LAYER_CASING, pl(&l))?;
    add(s, &space, LAYER_CASING, pl(&r))?;
    for (x, y) in [(l.first(), r.first()), (l.last(), r.last())] {
        if let (Some(x), Some(y)) = (x, y) {
            add(s, &space, LAYER_CASING, EntityKind::Line(Line { a: Vec3::new(x.x, x.y, 0.0), b: Vec3::new(y.x, y.y, 0.0) }))?;
        }
    }
    Ok(json!({ "length": s1 - s0, "message": format!("Casing pipe ⌀{:.0} mm, {:.2} m", d * 1000.0, s1 - s0) }))
}

struct SelM {
    name: &'static str,
    msg: &'static str,
    cmd: &'static str,
}

impl Interactive for SelM {
    fn name(&self) -> &'static str {
        self.name
    }
    fn prompt(&self, _s: &Session) -> Prompt {
        Prompt::new(self.msg, Accept::SELECT)
    }
    fn input(&mut self, s: &mut Session, i: Input) -> Result<Step> {
        match i {
            Input::Cancel => Ok(Step::Cancel),
            Input::Pick(hs) => {
                let hs: Vec<String> = hs.iter().map(|h| h.hex()).collect();
                let r = s.execute(self.cmd, &json!({ "handles": hs }));
                say(s, r);
                Ok(Step::Done)
            }
            _ => Ok(Step::Continue),
        }
    }
}

/// GEOZABORY: the take, then where the table goes (Enter: no table).
#[derive(Default)]
struct ZabM {
    take: Option<Handle>,
}

impl Interactive for ZabM {
    fn name(&self) -> &'static str {
        "GEOZABORY"
    }
    fn prompt(&self, _s: &Session) -> Prompt {
        match self.take {
            None => Prompt::new("Select the closed take boundary", Accept::SELECT),
            Some(_) => Prompt::new("Pick where the table goes (Enter: no table)", Accept::POINT),
        }
    }
    fn input(&mut self, s: &mut Session, i: Input) -> Result<Step> {
        match (self.take, i) {
            (_, Input::Cancel) => Ok(Step::Cancel),
            (None, Input::Pick(hs)) => {
                self.take = hs.first().copied();
                Ok(Step::Continue)
            }
            (Some(t), Input::Point(q)) => {
                let r = run_zabory(s, &json!({ "take": t.hex(), "at": [q.x, q.y] }));
                say(s, r);
                Ok(Step::Done)
            }
            (Some(t), Input::Enter) => {
                let r = run_zabory(s, &json!({ "take": t.hex() }));
                say(s, r);
                Ok(Step::Done)
            }
            _ => Ok(Step::Continue),
        }
    }
}

/// GEOCHRANICKA: the line, both ends, the diameter.
#[derive(Default)]
struct ChrM {
    line: Option<Handle>,
    ends: Vec<Vec2>,
}

impl Interactive for ChrM {
    fn name(&self) -> &'static str {
        "GEOCHRANICKA"
    }
    fn prompt(&self, _s: &Session) -> Prompt {
        match (self.line, self.ends.len()) {
            (None, _) => Prompt::new("Select the utility line", Accept::SELECT),
            (Some(_), 0) => Prompt::new("Pick the start of the casing", Accept::POINT),
            (Some(_), 1) => Prompt::new("Pick the end of the casing", Accept::POINT),
            _ => Prompt::new("Casing diameter (m)", Accept::NUMBER).default("0.2"),
        }
    }
    fn input(&mut self, s: &mut Session, i: Input) -> Result<Step> {
        match (self.line, i) {
            (_, Input::Cancel) => Ok(Step::Cancel),
            (None, Input::Pick(hs)) => {
                self.line = hs.first().copied();
                Ok(Step::Continue)
            }
            (Some(_), Input::Point(q)) if self.ends.len() < 2 => {
                self.ends.push(q);
                Ok(Step::Continue)
            }
            (Some(l), i @ (Input::Text(_) | Input::Keyword(_) | Input::Enter)) if self.ends.len() == 2 => {
                let d = match i {
                    Input::Text(t) | Input::Keyword(t) => t,
                    _ => "0.2".into(),
                };
                let (a, b) = (self.ends.first().copied().unwrap_or(Vec2::ZERO), self.ends.get(1).copied().unwrap_or(Vec2::ZERO));
                let r = run_chranicka(s, &json!({ "line": l.hex(), "from": [a.x, a.y], "to": [b.x, b.y], "diameter": d.trim() }));
                say(s, r);
                Ok(Step::Done)
            }
            _ => Ok(Step::Continue),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cadcraft_doc::Polyline3d;

    fn line(s: &mut Session, layer: &str, a: (f64, f64, f64), b: (f64, f64, f64)) -> Handle {
        let space = s.space();
        add(s, &space, layer, EntityKind::Line(Line { a: Vec3::new(a.0, a.1, a.2), b: Vec3::new(b.0, b.1, b.2) })).unwrap()
    }

    #[test]
    fn crossings_with_clearance() {
        let mut s = Session::new();
        let space = s.space();
        let a = add(
            &mut s,
            &space,
            "V",
            EntityKind::Polyline3d(Polyline3d { points: vec![Vec3::new(0.0, 0.0, 300.0), Vec3::new(10.0, 0.0, 300.0)], closed: false }),
        )
        .unwrap();
        let b = add(
            &mut s,
            &space,
            "K",
            EntityKind::Polyline3d(Polyline3d { points: vec![Vec3::new(5.0, -5.0, 298.0), Vec3::new(5.0, 5.0, 299.0)], closed: false }),
        )
        .unwrap();
        let c = line(&mut s, "E", (2.0, -1.0, 0.0), (2.0, 1.0, 0.0));
        let r = s.execute("geokrizeni", &json!({ "handles": [a.hex(), b.hex(), c.hex()] })).unwrap();
        let x = r["crossings"].as_array().unwrap();
        assert_eq!(x.len(), 2, "{r}");
        let deep = x.iter().find(|c| c["clearance"].is_number()).unwrap();
        assert!((deep["clearance"].as_f64().unwrap() - 1.5).abs() < 1e-9);
    }

    #[test]
    fn land_take_per_parcel() {
        let mut s = Session::new();
        // Two parcels side by side, 10 × 10 each, as boundary lines with numbers.
        for (a, b) in [
            ((0.0, 0.0), (20.0, 0.0)),
            ((20.0, 0.0), (20.0, 10.0)),
            ((20.0, 10.0), (0.0, 10.0)),
            ((0.0, 10.0), (0.0, 0.0)),
            ((10.0, 0.0), (10.0, 10.0)),
        ] {
            line(&mut s, LAYER_BOUNDARIES, (a.0, a.1, 0.0), (b.0, b.1, 0.0));
        }
        let space = s.space();
        for (n, x) in [("101", 5.0), ("102/1", 15.0)] {
            text(&mut s, &space, KN_NUMBERS, Vec2::new(x, 5.0), 1.0, n.into(), 0.0, (HAlign::Center, VAlign::Middle)).unwrap();
        }
        // A take 4 m wide across both.
        let take = cadcraft_doc::LwPolyline {
            vertices: [(5.0, 2.0), (18.0, 2.0), (18.0, 6.0), (5.0, 6.0)]
                .iter()
                .map(|(x, y)| cadcraft_geom::PolyVertex::new(Vec2::new(*x, *y)))
                .collect(),
            closed: true,
            const_width: 0.0,
            elevation: 0.0,
            plinegen: false,
        };
        let t = add(&mut s, &space, "0", EntityKind::LwPolyline(take)).unwrap();
        let dir = std::env::temp_dir().join(format!("geozabory-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let csv = dir.join("zabory.csv");
        let r = s.execute("geozabory", &json!({ "take": t.hex(), "at": [0, -5], "csv": csv.to_string_lossy() })).unwrap();
        let rows = r["parcels"].as_array().unwrap();
        assert_eq!(rows.len(), 2, "{r}");
        assert!((rows[0]["take"].as_f64().unwrap() - 20.0).abs() < 1e-6);
        assert!((rows[1]["take"].as_f64().unwrap() - 32.0).abs() < 1e-6);
        assert!((rows[0]["area"].as_f64().unwrap() - 100.0).abs() < 1e-6);
        let written = std::fs::read_to_string(&csv).unwrap();
        assert!(written.contains("102/1;100,00;32,00") && written.contains("celkem;;52,00"), "{written}");
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn casing_pipe_along_a_line() {
        let mut s = Session::new();
        let l = line(&mut s, "V", (0.0, 0.0, 0.0), (20.0, 0.0, 0.0));
        let r = s.execute("geochranicka", &json!({ "line": l.hex(), "from": [5, 1], "to": [12, -1], "diameter": 0.3 })).unwrap();
        assert!((r["length"].as_f64().unwrap() - 7.0).abs() < 1e-9);
        let k: Vec<_> = s.doc().unwrap().model.iter().filter(|e| e.common.layer == LAYER_CASING).map(|e| e.kind.clone()).collect();
        assert_eq!(k.len(), 4);
        assert!(k.iter().any(|e| matches!(e, EntityKind::LwPolyline(p) if p.vertices.iter().all(|v| (v.p.y - 0.15).abs() < 1e-9))));
    }

    #[test]
    fn hostile_input_is_refused_not_panicking() {
        let mut s = Session::new();
        let l = line(&mut s, "V", (0.0, 0.0, 0.0), (20.0, 0.0, 0.0));
        for (c, p) in [
            ("geokrizeni", json!({ "handles": [l.hex()] })),
            ("geozabory", json!({ "take": l.hex() })),
            ("geozabory", json!({ "take": "FFFFF" })),
            ("geochranicka", json!({ "line": l.hex(), "from": [5, 0], "to": [5, 0], "diameter": 0.2 })),
            ("geochranicka", json!({ "line": l.hex(), "from": [1, 0], "to": [5, 0], "diameter": -1 })),
        ] {
            assert!(s.execute(c, &p).is_err(), "{c} {p}");
        }
    }
}
