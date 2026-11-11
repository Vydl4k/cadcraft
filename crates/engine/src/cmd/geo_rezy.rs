//! Survey add-on: the design grade line in a long section (GEONIVELETA) and cross sections along
//! an axis (GEOPRICNE).
//!
//! GEONIVELETA takes a polyline drawn over a GEOPROFIL long section and writes what it means:
//! the height at each vertex, the slope and length of each segment, and under the profile two
//! rows with the grade height and its difference from the terrain (+ cut, − fill) at every
//! vertex and station. The profile is found by its "S.R." datum label; its exaggeration by the
//! "převýšení" label GEOPROFIL writes beside it.
//!
//! GEOPRICNE cuts the terrain model across an axis every `step` metres, `width` to either side,
//! and draws the cross sections in a grid, each with its station, datum and axis. When there is a
//! design surface (GEO_DMT_navrh, see GEONAVRH), its line is drawn too, the cut and fill areas of
//! every section are written under it, and the volumes between sections are summed by average
//! end areas.

use cadcraft_doc::{EntityKind, HAlign, Line, LwPolyline, Space, VAlign};
use cadcraft_geom::{PolyVertex, Vec2, Vec3};
use serde_json::{Value, json};

use super::geo_calc::{LAYER_PROFILE, add, ensure_layer, one_target, path_of, say, section, text, tin, tin_on, typed_number};
use super::geo_dmt::LAYER_DESIGN;
use super::{CommandSpec, bad, f64_or, point_value};
use crate::{Accept, Input, Interactive, Prompt, Result, Session, Step};
use cadcraft_doc::Handle;

pub const LAYER_GRADE: &str = "GEO_niveleta";
pub const LAYER_CROSS: &str = "GEO_pricne_rezy";
pub const LAYER_CROSS_DESIGN: &str = "GEO_pricne_navrh";

/// Cross sections drawn at once.
const MAX_SECTIONS: usize = 2_000;
/// Vertices of a grade line.
const MAX_GRADE: usize = 10_000;

pub(super) fn specs() -> Vec<CommandSpec> {
    vec![
        CommandSpec::new("geoniveleta", "Grade Line in Long Section...", run_geoniveleta)
            .menu(&["Geo", "Grade Line in Long Section..."])
            .alias(&["geograde"])
            .params("{grade?: handle of the polyline drawn over a GEOPROFIL profile (default: the selected one), step?: station spacing for the rows under the profile (default 10), scale?: 1:N for text (default 500)} → {vertices: [{station, height, terrain?, difference?}], segments: [{length, slope}], message}")
            .interactive(|_| Ok(Box::new(NivM))),
        CommandSpec::new("geopricne", "Cross Sections...", run_geopricne)
            .menu(&["Geo", "Cross Sections..."])
            .alias(&["geocross"])
            .params("{path?: handle of the axis (line or polyline; default: the selected one), step?: spacing in metres (default 20), width?: to each side (default 10), at: [x, y] lower left of the first section, columns?: sections per row (default 4), exaggeration?: vertical (default 1), scale?: 1:N for text (default 200)} → {sections: [{station, cut?, fill?}], cutVolume?, fillVolume?, message}")
            .interactive(|_| Ok(Box::new(PricneM::default()))),
    ]
}

/// "0+012.50" for 12.5 m.
pub fn station_text(st: f64) -> String {
    let st = st.max(0.0);
    format!("{}+{:06.2}", (st / 1000.0).floor() as i64, st % 1000.0)
}

/// The height of a polyline (in profile space) at `x`, if it spans it.
fn y_at(lines: &[Vec<Vec2>], x: f64) -> Option<f64> {
    for l in lines {
        for w in l.windows(2) {
            let (Some(a), Some(b)) = (w.first(), w.get(1)) else { continue };
            let (lo, hi) = if a.x <= b.x { (a, b) } else { (b, a) };
            if x >= lo.x - 1e-9 && x <= hi.x + 1e-9 {
                if (hi.x - lo.x).abs() < 1e-12 {
                    return Some(hi.y.max(lo.y));
                }
                let t = (x - lo.x) / (hi.x - lo.x);
                return Some(lo.y + t * (hi.y - lo.y));
            }
        }
    }
    None
}

/// The profile under a grade line: where it starts (datum line's left end), its datum height and
/// exaggeration, and its terrain polylines.
struct Profile {
    at: Vec2,
    datum: f64,
    ex: f64,
    th: f64,
    terrain: Vec<Vec<Vec2>>,
}

fn find_profile(s: &Session, space: &Space, grade: &[Vec2], cmd: &str) -> Result<Profile> {
    let d = s.doc()?;
    let st = d.space(space).ok_or_else(|| bad(cmd, "no drawing space"))?;
    let on_profile = || st.iter().filter(|e| e.common.layer.eq_ignore_ascii_case(LAYER_PROFILE));
    let (gx, gy) = grade.iter().fold((f64::INFINITY, f64::INFINITY), |(x, y), p| (x.min(p.x), y.min(p.y)));
    // The datum label nearest under the grade line, starting left of it.
    let mut best: Option<(f64, Vec2, f64, f64)> = None;
    for e in on_profile() {
        let EntityKind::Text(t) = &e.kind else { continue };
        let Some(v) = t.value.strip_prefix("S.R. ").and_then(|v| v.trim().parse::<f64>().ok()).filter(|v| v.is_finite()) else { continue };
        let p = t.align_pt.unwrap_or(t.insert).xy();
        let at = Vec2::new(p.x + t.height, p.y);
        let below = gy - at.y;
        if below >= -1e-6 && at.x <= gx + 1e-6 && best.is_none_or(|(b, ..)| below < b) {
            best = Some((below, at, v, t.height));
        }
    }
    let (_, at, datum, th) = best.ok_or_else(|| bad(cmd, "draw the grade line over a long section from GEOPROFIL first"))?;
    let ex = on_profile()
        .filter_map(|e| if let EntityKind::Text(t) = &e.kind { Some(t) } else { None })
        .filter_map(|t| {
            let v = t.value.strip_prefix("převýšení ")?.trim_end_matches('×').trim().parse::<f64>().ok()?;
            let p = t.align_pt.unwrap_or(t.insert).xy();
            let d = (p.x - (at.x - th)).hypot(p.y - (at.y - 1.5 * th));
            (d < 0.5 * th && v.is_finite() && v > 0.0).then_some(v)
        })
        .next()
        .unwrap_or(10.0);
    let gx1 = grade.iter().map(|p| p.x).fold(f64::NEG_INFINITY, f64::max);
    let terrain: Vec<Vec<Vec2>> = on_profile()
        .filter_map(|e| if let EntityKind::LwPolyline(p) = &e.kind { Some(p.vertices.iter().map(|v| v.p).collect::<Vec<_>>()) } else { None })
        .filter(|v| v.len() >= 2 && v.iter().all(|p| p.y >= at.y - 1e-6))
        .filter(|v| v.iter().any(|p| p.x >= gx - 1e-6) && v.iter().any(|p| p.x <= gx1 + 1e-6))
        .collect();
    // The grade must be drawn over this profile, not somewhere far above it.
    let top = terrain.iter().flatten().map(|p| p.y).fold(f64::NEG_INFINITY, f64::max);
    if !top.is_finite() || gy > at.y + 3.0 * (top - at.y) + 20.0 * th {
        return Err(bad(cmd, "draw the grade line over a long section from GEOPROFIL first"));
    }
    Ok(Profile { at, datum, ex, th, terrain })
}

fn run_geoniveleta(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "geoniveleta";
    let space = s.space();
    let h = one_target(s, p, "grade", CMD)?;
    let (grade, _) = path_of(s, h, &space).ok_or_else(|| bad(CMD, "the grade line must be a line or polyline"))?;
    if grade.len() < 2 || grade.len() > MAX_GRADE || grade.iter().any(|v| !(v.x.is_finite() && v.y.is_finite())) {
        return Err(bad(CMD, format!("the grade line needs 2 to {MAX_GRADE} vertices")));
    }
    let mut grade = grade;
    if grade.first().zip(grade.last()).is_some_and(|(a, b)| a.x > b.x) {
        grade.reverse();
    }
    if grade.windows(2).any(|w| matches!(w, [a, b] if b.x <= a.x)) {
        return Err(bad(CMD, "the grade line must run left to right along the profile"));
    }
    let prof = find_profile(s, &space, &grade, CMD)?;
    let step = f64_or(p, "step", 10.0);
    let scale = f64_or(p, "scale", 500.0);
    if !(step.is_finite() && step > 0.0 && scale.is_finite() && (1.0..=1e6).contains(&scale)) {
        return Err(bad(CMD, "`step` and `scale` must be positive"));
    }
    let th = 0.002 * scale;
    let height = |y: f64| prof.datum + (y - prof.at.y) / prof.ex;
    let station = |x: f64| x - prof.at.x;
    ensure_layer(s, LAYER_GRADE, 1)?;
    // Vertices: height above, rotated like the profile's heights.
    let mut vertices = Vec::new();
    for v in &grade {
        let z = height(v.y);
        text(
            s,
            &space,
            LAYER_GRADE,
            Vec2::new(v.x - 0.3 * th, v.y + 0.6 * th),
            th * 0.8,
            format!("{z:.2}"),
            std::f64::consts::FRAC_PI_2,
            (HAlign::Left, VAlign::Middle),
        )?;
    }
    // Segments: slope in % and length along the middle.
    let mut segments = Vec::new();
    for w in grade.windows(2) {
        let [a, b] = w else { continue };
        let len = station(b.x) - station(a.x);
        let slope = (height(b.y) - height(a.y)) / len * 100.0;
        let mid = Vec2::new((a.x + b.x) / 2.0, (a.y + b.y) / 2.0);
        let ang = (b.y - a.y).atan2(b.x - a.x);
        let (nx, ny) = (-ang.sin(), ang.cos());
        text(
            s,
            &space,
            LAYER_GRADE,
            Vec2::new(mid.x + nx * 0.6 * th, mid.y + ny * 0.6 * th),
            th * 0.8,
            format!("{} %  {len:.2} m", format!("{slope:+.2}").replace('.', ",")),
            ang,
            (HAlign::Center, VAlign::Bottom),
        )?;
        segments.push(json!({ "length": len, "slope": slope }));
    }
    // Rows under the profile: grade height and difference to the terrain.
    let (row1, row2) = (prof.at.y - 9.0 * prof.th, prof.at.y - 13.0 * prof.th);
    let left = prof.at.x - prof.th;
    text(s, &space, LAYER_GRADE, Vec2::new(left, row1), prof.th * 0.8, "Niveleta".into(), 0.0, (HAlign::Right, VAlign::Middle))?;
    text(s, &space, LAYER_GRADE, Vec2::new(left, row2), prof.th * 0.8, "Rozdíl (+výkop)".into(), 0.0, (HAlign::Right, VAlign::Middle))?;
    let (x0, x1) = (grade.first().map_or(0.0, |v| v.x), grade.last().map_or(0.0, |v| v.x));
    let mut xs: Vec<f64> = grade.iter().map(|v| v.x).collect();
    let first = (station(x0) / step).ceil() as i64;
    let last = (station(x1) / step).floor() as i64;
    if last.saturating_sub(first) > 100_000 {
        return Err(bad(CMD, "too many stations: use a larger `step`"));
    }
    for i in first..=last {
        xs.push(prof.at.x + i as f64 * step);
    }
    xs.sort_by(f64::total_cmp);
    xs.dedup_by(|a, b| (*a - *b).abs() < 1e-6);
    let grade_line = vec![grade.clone()];
    for x in xs {
        let Some(gy) = y_at(&grade_line, x) else { continue };
        let z = height(gy);
        let terrain = y_at(&prof.terrain, x).map(height);
        let diff = terrain.map(|t| t - z);
        let rot = std::f64::consts::FRAC_PI_2;
        text(s, &space, LAYER_GRADE, Vec2::new(x, row1), prof.th * 0.7, format!("{z:.2}"), rot, (HAlign::Center, VAlign::Middle))?;
        if let Some(d) = diff {
            text(
                s,
                &space,
                LAYER_GRADE,
                Vec2::new(x, row2),
                prof.th * 0.7,
                format!("{d:+.2}").replace('.', ","),
                rot,
                (HAlign::Center, VAlign::Middle),
            )?;
        }
        if grade.iter().any(|v| (v.x - x).abs() < 1e-6) {
            vertices.push(json!({ "station": station(x), "height": z, "terrain": terrain, "difference": diff }));
        }
    }
    Ok(json!({
        "vertices": vertices, "segments": segments,
        "message": format!("Grade line: {} segments from {} to {}", segments.len(), station_text(station(x0)), station_text(station(x1))),
    }))
}

/// The point at station `st` along `path` and the unit direction there.
fn frame_at(path: &[Vec2], st: f64) -> Option<(Vec2, Vec2)> {
    let mut left = st;
    let mut last = None;
    for w in path.windows(2) {
        let (Some(&a), Some(&b)) = (w.first(), w.get(1)) else { continue };
        let len = (b.x - a.x).hypot(b.y - a.y);
        if len < 1e-12 {
            continue;
        }
        let u = Vec2::new((b.x - a.x) / len, (b.y - a.y) / len);
        if left <= len + 1e-9 {
            let t = (left / len).clamp(0.0, 1.0);
            return Some((Vec2::new(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)), u));
        }
        left -= len;
        last = Some((b, u));
    }
    last
}

/// Area where `upper` lies above `lower` and where it lies below, over the offsets both cover.
/// Both are (offset, height) runs sorted by offset.
pub fn areas_between(upper: &[(f64, f64)], lower: &[(f64, f64)]) -> (f64, f64) {
    let interp = |l: &[(f64, f64)], x: f64| -> Option<f64> {
        l.windows(2).find_map(|w| match w {
            [a, b] if x >= a.0 - 1e-9 && x <= b.0 + 1e-9 => {
                Some(if (b.0 - a.0).abs() < 1e-12 { a.1 } else { a.1 + (x - a.0) / (b.0 - a.0) * (b.1 - a.1) })
            }
            _ => None,
        })
    };
    let mut xs: Vec<f64> = upper.iter().chain(lower).map(|p| p.0).filter(|x| x.is_finite()).collect();
    xs.sort_by(f64::total_cmp);
    xs.dedup_by(|a, b| (*a - *b).abs() < 1e-9);
    let (mut above, mut below) = (0.0, 0.0);
    for w in xs.windows(2) {
        let [x0, x1] = w else { continue };
        let (Some(u0), Some(l0), Some(u1), Some(l1)) = (interp(upper, *x0), interp(lower, *x0), interp(upper, *x1), interp(lower, *x1)) else {
            continue;
        };
        let (d0, d1) = (u0 - l0, u1 - l1);
        let dx = x1 - x0;
        if d0 >= 0.0 && d1 >= 0.0 {
            above += (d0 + d1) / 2.0 * dx;
        } else if d0 <= 0.0 && d1 <= 0.0 {
            below -= (d0 + d1) / 2.0 * dx;
        } else {
            // The lines cross inside: split at the crossing.
            let t = d0 / (d0 - d1);
            let (a, b) = (d0 * t * dx / 2.0, d1 * (1.0 - t) * dx / 2.0);
            for v in [a, b] {
                if v > 0.0 {
                    above += v;
                } else {
                    below -= v;
                }
            }
        }
    }
    (above, below)
}

fn run_geopricne(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "geopricne";
    let space = s.space();
    let h = one_target(s, p, "path", CMD)?;
    let (path, _) = path_of(s, h, &space).ok_or_else(|| bad(CMD, "the axis must be a line or polyline"))?;
    let at = p
        .get("at")
        .and_then(point_value)
        .filter(|a| a.x.is_finite() && a.y.is_finite())
        .ok_or_else(|| bad(CMD, "`at` ([x, y], where to draw the sections) is required"))?;
    let step = f64_or(p, "step", 20.0);
    let width = f64_or(p, "width", 10.0);
    let ex = f64_or(p, "exaggeration", 1.0);
    let scale = f64_or(p, "scale", 200.0);
    let columns = p.get("columns").and_then(Value::as_u64).unwrap_or(4).clamp(1, 100) as usize;
    if !(step.is_finite()
        && step > 0.0
        && width.is_finite()
        && width > 0.0
        && width < 1e5
        && ex.is_finite()
        && (0.01..=100.0).contains(&ex)
        && scale.is_finite()
        && (1.0..=1e6).contains(&scale))
    {
        return Err(bad(CMD, "`step`, `width`, `exaggeration` and `scale` must be positive"));
    }
    let length: f64 = path.windows(2).map(|w| if let [a, b] = w { (b.x - a.x).hypot(b.y - a.y) } else { 0.0 }).sum();
    if !(length.is_finite() && length > 0.0) {
        return Err(bad(CMD, "the axis has no length"));
    }
    let n = (length / step).floor() as usize;
    if n + 1 > MAX_SECTIONS {
        return Err(bad(CMD, "too many sections: use a larger `step`"));
    }
    let mut stations: Vec<f64> = (0..=n).map(|i| i as f64 * step).collect();
    if stations.last().is_some_and(|l| length - l > 1e-6) {
        stations.push(length);
    }
    let terrain = tin(s, &space)?;
    if terrain.is_empty() {
        return Err(bad(CMD, "no terrain model: run GEODMT first"));
    }
    let design = tin_on(s, &space, LAYER_DESIGN)?;
    // Every section's terrain and design lines, as (offset, height).
    let mut cuts: Vec<(f64, Vec<(f64, f64)>, Vec<(f64, f64)>)> = Vec::new();
    for st in &stations {
        let Some((c, u)) = frame_at(&path, *st) else { continue };
        let nrm = Vec2::new(-u.y, u.x);
        let line = [Vec2::new(c.x + nrm.x * width, c.y + nrm.y * width), Vec2::new(c.x - nrm.x * width, c.y - nrm.y * width)];
        let to_offset = |v: Vec<(f64, f64)>| v.into_iter().map(|(d, z)| (d - width, z)).collect::<Vec<_>>();
        let t = to_offset(section(&terrain, &line));
        let dl = if design.is_empty() { Vec::new() } else { to_offset(section(&design, &line)) };
        cuts.push((*st, t, dl));
    }
    let zs = cuts.iter().flat_map(|(_, t, d)| t.iter().chain(d).map(|p| p.1));
    let (zmin, zmax) = zs.fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), z| (lo.min(z), hi.max(z)));
    if !zmin.is_finite() {
        return Err(bad(CMD, "the sections don't cross the terrain model"));
    }
    let th = 0.002 * scale;
    let cell_w = 2.0 * width + 8.0 * th;
    let cell_h = ((zmax - zmin) + 2.0) * ex + 10.0 * th;
    ensure_layer(s, LAYER_CROSS, 4)?;
    if !design.is_empty() {
        ensure_layer(s, LAYER_CROSS_DESIGN, 1)?;
    }
    let mut out = Vec::new();
    let mut prev: Option<(f64, f64, f64)> = None;
    let (mut cut_vol, mut fill_vol) = (0.0, 0.0);
    for (i, (st, t, d)) in cuts.iter().enumerate() {
        let (col, row) = (i % columns, i / columns);
        // Sections fill rows downwards from `at`.
        let o = Vec2::new(at.x + col as f64 * cell_w + 4.0 * th + width, at.y - row as f64 * cell_h);
        let local: Vec<f64> = t.iter().chain(d.iter()).map(|p| p.1).collect();
        let datum = (local.iter().copied().fold(f64::INFINITY, f64::min) - 1.0).floor();
        if !datum.is_finite() {
            out.push(json!({ "station": st }));
            continue;
        }
        let y = |z: f64| o.y + 3.0 * th + (z - datum) * ex;
        let poly = |line: &[(f64, f64)]| LwPolyline {
            vertices: line.iter().map(|(x, z)| PolyVertex::new(Vec2::new(o.x + x, y(*z)))).collect(),
            closed: false,
            const_width: 0.0,
            elevation: 0.0,
            plinegen: false,
        };
        if t.len() >= 2 {
            add(s, &space, LAYER_CROSS, EntityKind::LwPolyline(poly(t)))?;
        }
        if d.len() >= 2 {
            add(s, &space, LAYER_CROSS_DESIGN, EntityKind::LwPolyline(poly(d)))?;
        }
        let base = o.y + 3.0 * th;
        add(s, &space, LAYER_CROSS, EntityKind::Line(Line { a: Vec3::new(o.x - width, base, 0.0), b: Vec3::new(o.x + width, base, 0.0) }))?;
        let top = local.iter().copied().fold(datum, f64::max);
        add(s, &space, LAYER_CROSS, EntityKind::Line(Line { a: Vec3::new(o.x, base, 0.0), b: Vec3::new(o.x, y(top) + th, 0.0) }))?;
        text(
            s,
            &space,
            LAYER_CROSS,
            Vec2::new(o.x - width - 0.5 * th, base),
            th * 0.8,
            format!("S.R. {datum:.2}"),
            0.0,
            (HAlign::Right, VAlign::Middle),
        )?;
        text(s, &space, LAYER_CROSS, Vec2::new(o.x, y(top) + 2.0 * th), th, station_text(*st), 0.0, (HAlign::Center, VAlign::Bottom))?;
        if let Some(zt) = height_on(t, 0.0) {
            text(
                s,
                &space,
                LAYER_CROSS,
                Vec2::new(o.x + 0.4 * th, base - 1.2 * th),
                th * 0.7,
                format!("{zt:.2}"),
                0.0,
                (HAlign::Left, VAlign::Middle),
            )?;
        }
        if d.len() >= 2 {
            let (cut, fill) = areas_between(t, d);
            text(
                s,
                &space,
                LAYER_CROSS,
                Vec2::new(o.x, base - 2.6 * th),
                th * 0.8,
                format!("V {} m²  N {} m²", fmt1(cut), fmt1(fill)),
                0.0,
                (HAlign::Center, VAlign::Middle),
            )?;
            if let Some((ps, pc, pf)) = prev {
                cut_vol += (pc + cut) / 2.0 * (st - ps);
                fill_vol += (pf + fill) / 2.0 * (st - ps);
            }
            prev = Some((*st, cut, fill));
            out.push(json!({ "station": st, "cut": cut, "fill": fill }));
        } else {
            out.push(json!({ "station": st }));
        }
    }
    let with_design = !design.is_empty();
    Ok(json!({
        "sections": out,
        "cutVolume": with_design.then_some(cut_vol),
        "fillVolume": with_design.then_some(fill_vol),
        "message": if with_design {
            format!("{} cross sections every {step} m; cut {cut_vol:.1} m³, fill {fill_vol:.1} m³ (average end areas)", cuts.len())
        } else {
            format!("{} cross sections every {step} m, {width} m to each side", cuts.len())
        },
    }))
}

fn height_on(line: &[(f64, f64)], x: f64) -> Option<f64> {
    line.windows(2).find_map(|w| match w {
        [a, b] if x >= a.0 && x <= b.0 && b.0 > a.0 => Some(a.1 + (x - a.0) / (b.0 - a.0) * (b.1 - a.1)),
        _ => None,
    })
}

fn fmt1(v: f64) -> String {
    format!("{v:.1}").replace('.', ",")
}

struct NivM;

impl Interactive for NivM {
    fn name(&self) -> &'static str {
        "GEONIVELETA"
    }
    fn prompt(&self, _s: &Session) -> Prompt {
        Prompt::new("Select the grade line drawn over the long section", Accept::SELECT)
    }
    fn input(&mut self, s: &mut Session, i: Input) -> Result<Step> {
        match i {
            Input::Cancel | Input::Enter => Ok(Step::Done),
            Input::Pick(hs) => match hs.first() {
                Some(h) => {
                    let r = run_geoniveleta(s, &json!({ "grade": h.hex() }));
                    say(s, r);
                    Ok(Step::Done)
                }
                None => Ok(Step::Continue),
            },
            _ => Ok(Step::Continue),
        }
    }
}

#[derive(Default)]
struct PricneM {
    path: Option<Handle>,
    step: Option<f64>,
    width: Option<f64>,
}

impl Interactive for PricneM {
    fn name(&self) -> &'static str {
        "GEOPRICNE"
    }
    fn prompt(&self, _s: &Session) -> Prompt {
        match (self.path, self.step, self.width) {
            (None, ..) => Prompt::new("Select the axis", Accept::SELECT),
            (_, None, _) => Prompt::new("Distance between sections (m)", Accept::NUMBER).default("20"),
            (_, _, None) => Prompt::new("Width to each side (m)", Accept::NUMBER).default("10"),
            _ => Prompt::new("Where to draw the sections", Accept::POINT),
        }
    }
    fn input(&mut self, s: &mut Session, i: Input) -> Result<Step> {
        if matches!(i, Input::Cancel) {
            return Ok(Step::Cancel);
        }
        if self.path.is_none() {
            if let Input::Pick(hs) = i {
                self.path = hs.first().copied();
            }
            return Ok(Step::Continue);
        }
        let number = match &i {
            Input::Text(t) | Input::Keyword(t) => typed_number(t).filter(|v| *v > 0.0),
            _ => None,
        };
        if self.step.is_none() {
            match (&i, number) {
                (Input::Enter, _) => self.step = Some(20.0),
                (_, Some(v)) => self.step = Some(v),
                _ => s.echo("Enter the distance in metres, like 20."),
            }
            return Ok(Step::Continue);
        }
        if self.width.is_none() {
            match (&i, number) {
                (Input::Enter, _) => self.width = Some(10.0),
                (_, Some(v)) => self.width = Some(v),
                _ => s.echo("Enter the width in metres, like 10."),
            }
            return Ok(Step::Continue);
        }
        let (Input::Point(at), Some(h)) = (i, self.path) else { return Ok(Step::Continue) };
        let r = run_geopricne(s, &json!({ "path": h.hex(), "step": self.step, "width": self.width, "at": [at.x, at.y] }));
        say(s, r);
        Ok(Step::Done)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cadcraft_doc::Face3d;

    fn ramp(s: &mut Session, layer: &str, f: impl Fn(f64, f64) -> f64) {
        let space = s.space();
        let n = 10;
        let v = |i: usize, j: usize| {
            let (x, y) = (i as f64 * 10.0, j as f64 * 10.0 - 50.0);
            Vec3::new(x, y, f(x, y))
        };
        for i in 0..n {
            for j in 0..n {
                let (a, b, c, d) = (v(i, j), v(i + 1, j), v(i + 1, j + 1), v(i, j + 1));
                for t in [[a, b, c], [a, c, d]] {
                    add(s, &space, layer, EntityKind::Face3d(Face3d { corners: [t[0], t[1], t[2], t[2]], hidden_edges: 0 })).unwrap();
                }
            }
        }
    }

    fn pline(s: &mut Session, pts: &[(f64, f64)]) -> String {
        let space = s.space();
        let pl = LwPolyline {
            vertices: pts.iter().map(|(x, y)| PolyVertex::new(Vec2::new(*x, *y))).collect(),
            closed: false,
            const_width: 0.0,
            elevation: 0.0,
            plinegen: false,
        };
        add(s, &space, "0", EntityKind::LwPolyline(pl)).unwrap().hex()
    }

    #[test]
    fn a_grade_line_over_a_profile() {
        let mut s = Session::new();
        // Terrain rising 1 m per 10 m eastwards from 100 m.
        ramp(&mut s, super::super::geo_dmt::LAYER_TIN, |x, _| 100.0 + x / 10.0);
        let axis = pline(&mut s, &[(0.0, 0.0), (100.0, 0.0)]);
        let r = s.execute("geoprofil", &json!({ "path": axis, "at": [0, -200], "datum": 99, "exaggeration": 10 })).unwrap();
        assert_eq!(r["datum"], 99.0);
        // A flat grade at 105 m from station 0 to 100: y = -200 + (105 - 99) × 10 = -140.
        let g = pline(&mut s, &[(0.0, -140.0), (50.0, -140.0), (100.0, -120.0)]);
        let r = s.execute("geoniveleta", &json!({ "grade": g })).unwrap();
        let v = r["vertices"].as_array().unwrap();
        assert_eq!(v.len(), 3, "{r}");
        assert!((v[0]["height"].as_f64().unwrap() - 105.0).abs() < 1e-9, "{r}");
        // Terrain 100 at station 0: 5 m of fill (difference −5).
        assert!((v[0]["difference"].as_f64().unwrap() + 5.0).abs() < 1e-9, "{r}");
        // Last vertex: 107 m against terrain 110 m: 3 m of cut.
        assert!((v[2]["height"].as_f64().unwrap() - 107.0).abs() < 1e-9 && (v[2]["difference"].as_f64().unwrap() - 3.0).abs() < 1e-9, "{r}");
        let segs = r["segments"].as_array().unwrap();
        assert!(segs[0]["slope"].as_f64().unwrap().abs() < 1e-12 && (segs[1]["slope"].as_f64().unwrap() - 4.0).abs() < 1e-9, "{r}");
        // A line far from any profile is refused.
        let lost = pline(&mut s, &[(0.0, 500.0), (10.0, 500.0)]);
        assert!(s.execute("geoniveleta", &json!({ "grade": lost })).is_err());
    }

    #[test]
    fn cross_sections_with_a_design() {
        let mut s = Session::new();
        ramp(&mut s, super::super::geo_dmt::LAYER_TIN, |_, y| 100.0 + y / 10.0);
        // Design flat at 100: across the axis (y from +10 to −10) the terrain is 1 m above on the
        // left and 1 m below on the right: 5 m² of cut and 5 m² of fill per section.
        ramp(&mut s, LAYER_DESIGN, |_, _| 100.0);
        let axis = pline(&mut s, &[(5.0, 0.0), (65.0, 0.0)]);
        let r = s.execute("geopricne", &json!({ "path": axis, "at": [0, -300], "step": 20, "width": 10 })).unwrap();
        let secs = r["sections"].as_array().unwrap();
        assert_eq!(secs.len(), 4, "{r}");
        for sec in secs {
            assert!((sec["cut"].as_f64().unwrap() - 5.0).abs() < 1e-6 && (sec["fill"].as_f64().unwrap() - 5.0).abs() < 1e-6, "{r}");
        }
        assert!((r["cutVolume"].as_f64().unwrap() - 300.0).abs() < 1e-6, "{r}");
        let labels: Vec<String> =
            s.doc().unwrap().model.iter().filter_map(|e| if let EntityKind::Text(t) = &e.kind { Some(t.value.clone()) } else { None }).collect();
        assert!(labels.iter().any(|t| t == "0+040.00") && labels.iter().any(|t| t == "V 5,0 m²  N 5,0 m²"), "{labels:?}");
    }

    #[test]
    fn areas_between_lines() {
        let up = [(0.0, 2.0), (10.0, 2.0)];
        let lo = [(0.0, 0.0), (10.0, 4.0)];
        let (a, b) = areas_between(&up, &lo);
        assert!((a - 5.0).abs() < 1e-12 && (b - 5.0).abs() < 1e-12, "{a} {b}");
        assert_eq!(areas_between(&[], &lo), (0.0, 0.0));
        assert_eq!(station_text(1234.5), "1+234.50");
    }

    #[test]
    fn hostile_input_is_refused_not_panicking() {
        let mut s = Session::new();
        for (c, p) in [
            ("geoniveleta", json!({})),
            ("geoniveleta", json!({ "grade": "zz" })),
            ("geopricne", json!({})),
            ("geopricne", json!({ "path": "zz", "at": [0, 0] })),
        ] {
            assert!(s.execute(c, &p).is_err(), "{c} {p}");
        }
        let axis = pline(&mut s, &[(0.0, 0.0), (10.0, 0.0)]);
        for p in [
            json!({ "path": axis, "at": [0, 0] }),
            json!({ "path": axis, "at": [0, 0], "step": 0 }),
            json!({ "path": axis, "at": [0, 0], "step": 1e-9 }),
        ] {
            assert!(s.execute("geopricne", &p).is_err(), "{p}");
        }
        let back = pline(&mut s, &[(10.0, 0.0), (5.0, 1.0), (8.0, 2.0)]);
        assert!(s.execute("geoniveleta", &json!({ "grade": back })).is_err());
        let _ = frame_at(&[], 1.0);
        let _ = areas_between(&[(f64::NAN, 1.0), (1.0, 1.0)], &[(0.0, 0.0), (1.0, 0.0)]);
    }
}
