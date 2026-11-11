//! Survey add-on: hypsometry (GEOHYPSO), the terrain model coloured by height bands.
//!
//! Every triangle of the TIN is cut exactly at the band limits (the terrain is flat within a
//! triangle), and the pieces of each band become solid hatches in that band's colour, from green
//! for the lowest through yellow to brown for the highest, on `GEO_hypsometrie`. A legend with
//! the band limits is drawn beside the model.

use cadcraft_color::{Color, Rgb};
use cadcraft_doc::{Common, EntityKind, HAlign, Handle, Hatch, HatchLoop, Space, VAlign};
use cadcraft_geom::{PolyVertex, Vec2, Vec3};
use serde_json::{Value, json};

use super::geo_calc::{bbox, clip_half, ensure_layer, plane, say, signed_area, text, tin, typed_number};
use super::{CommandSpec, bad, f64_or, point_value};
use crate::{Accept, Input, Interactive, Prompt, Result, Session, Step};

pub const LAYER_HYPSO: &str = "GEO_hypsometrie";

/// Bands drawn at once.
const MAX_BANDS: usize = 200;
/// Loops in one hatch; a band with more is split into several hatches.
const MAX_LOOPS: usize = 4_000;
/// Pieces produced at once.
const MAX_PIECES: usize = 5_000_000;

pub(super) fn specs() -> Vec<CommandSpec> {
    vec![
        CommandSpec::new("geohypso", "Hypsometry...", run_geohypso)
            .menu(&["Geo", "Hypsometry..."])
            .alias(&["geohypsometrie", "geocolorheights"])
            .params("{interval?: band height in metres (default: about 10 bands), legend?: [x, y] its upper left corner (default: right of the model), scale?: 1:N for the legend (default 500), replace?: bool (default true)} → {bands, pieces, from, to, message}")
            .interactive(|_| Ok(Box::new(HypsoM))),
    ]
}

/// A round band height giving about `n` bands over `range`.
pub fn nice_interval(range: f64, n: f64) -> f64 {
    if !(range.is_finite() && range > 0.0) {
        return 1.0;
    }
    let raw = range / n;
    let mag = 10f64.powf(raw.log10().floor());
    [1.0, 2.0, 2.5, 5.0, 10.0].iter().map(|k| k * mag).find(|v| *v >= raw).unwrap_or(10.0 * mag)
}

/// The colour of band `i` of `n`: green, yellow, brown, then a light top.
pub fn band_colour(i: usize, n: usize) -> Rgb {
    let stops: [(f64, (f64, f64, f64)); 4] =
        [(0.0, (60.0, 141.0, 47.0)), (0.4, (232.0, 212.0, 77.0)), (0.8, (156.0, 91.0, 46.0)), (1.0, (230.0, 225.0, 220.0))];
    let t = if n <= 1 { 0.0 } else { i as f64 / (n - 1) as f64 };
    let mut out = stops[0].1;
    for w in stops.windows(2) {
        let [(t0, c0), (t1, c1)] = w else { continue };
        if t >= *t0 && t <= *t1 {
            let k = (t - t0) / (t1 - t0);
            out = (c0.0 + k * (c1.0 - c0.0), c0.1 + k * (c1.1 - c0.1), c0.2 + k * (c1.2 - c0.2));
        }
    }
    Rgb(out.0.round() as u8, out.1.round() as u8, out.2.round() as u8)
}

/// The pieces of each band `[lo + i·step, lo + (i+1)·step)` of the triangles.
pub fn bands(tris: &[[Vec3; 3]], lo: f64, step: f64, n: usize) -> std::result::Result<Vec<Vec<Vec<Vec2>>>, String> {
    let mut out: Vec<Vec<Vec<Vec2>>> = vec![Vec::new(); n];
    let mut pieces = 0usize;
    for t in tris {
        let Some((a, b, c)) = plane(t) else { continue };
        let z = |q: Vec2| a * q.x + b * q.y + c;
        let mut flat = vec![t[0].xy(), t[1].xy(), t[2].xy()];
        if signed_area(&flat) < 0.0 {
            flat.swap(1, 2);
        }
        let (tz0, tz1) = t.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(l, h), v| (l.min(v.z), h.max(v.z)));
        let first = (((tz0 - lo) / step).floor().max(0.0) as usize).min(n.saturating_sub(1));
        let last = (((tz1 - lo) / step).floor().max(0.0) as usize).min(n.saturating_sub(1));
        for (i, band) in out.iter_mut().enumerate().take(last + 1).skip(first) {
            let (b0, b1) = (lo + i as f64 * step, lo + (i + 1) as f64 * step);
            let mut p = clip_half(&flat, |q| z(q) - b0);
            if i + 1 < n {
                p = clip_half(&p, |q| b1 - z(q));
            }
            if p.len() >= 3 && signed_area(&p).abs() > 1e-12 {
                band.push(p);
                pieces += 1;
                if pieces > MAX_PIECES {
                    return Err("the terrain model is too large to colour at once".into());
                }
            }
        }
    }
    Ok(out)
}

fn solid(loops: Vec<Vec<Vec2>>) -> EntityKind {
    EntityKind::Hatch(Hatch {
        pattern: "SOLID".into(),
        solid: true,
        loops: loops
            .into_iter()
            .map(|l| HatchLoop { vertices: l.into_iter().map(PolyVertex::new).collect(), outer: true, sources: Vec::new() })
            .collect(),
        scale: 1.0,
        angle: 0.0,
        associative: false,
        // Normal: the pieces only touch, and "ignore" would take a piece whose edge lies on its
        // neighbour's for an island and drop it.
        style: 0,
        elevation: 0.0,
        gradient: None,
        origin: Vec2::ZERO,
        background: None,
        pattern_lines: Vec::new(),
    })
}

fn clear_layer(s: &mut Session, space: &Space, layer: &str) -> Result<()> {
    let d = s.doc_mut()?;
    let doomed: Vec<Handle> =
        d.space(space).map(|st| st.iter().filter(|e| e.common.layer.eq_ignore_ascii_case(layer)).map(|e| e.handle).collect()).unwrap_or_default();
    for h in doomed {
        d.remove_entity(h);
    }
    Ok(())
}

fn run_geohypso(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "geohypso";
    let space = s.space();
    let tris = tin(s, &space)?;
    if tris.is_empty() {
        return Err(bad(CMD, "no terrain model: run GEODMT first"));
    }
    let (zmin, zmax) = tris.iter().flatten().fold((f64::INFINITY, f64::NEG_INFINITY), |(l, h), v| (l.min(v.z), h.max(v.z)));
    if !(zmin.is_finite() && zmax.is_finite()) {
        return Err(bad(CMD, "the terrain model has no heights"));
    }
    let step = match p.get("interval") {
        None | Some(Value::Null) => nice_interval(zmax - zmin, 10.0),
        Some(v) => v.as_f64().filter(|v| v.is_finite() && *v > 0.0).ok_or_else(|| bad(CMD, "`interval` must be a positive height"))?,
    };
    let scale = f64_or(p, "scale", 500.0);
    if !(scale.is_finite() && (1.0..=1e6).contains(&scale)) {
        return Err(bad(CMD, "`scale` must be a map scale like 500"));
    }
    let lo = (zmin / step).floor() * step;
    let n = (((zmax - lo) / step).floor() as usize).saturating_add(1).max(1);
    if n > MAX_BANDS {
        return Err(bad(CMD, format!("{n} bands: use a larger `interval`")));
    }
    let pieces = bands(&tris, lo, step, n).map_err(|e| bad(CMD, e))?;
    if p.get("replace").and_then(Value::as_bool).unwrap_or(true) {
        clear_layer(s, &space, LAYER_HYPSO)?;
    }
    ensure_layer(s, LAYER_HYPSO, 7)?;
    let mut count = 0;
    for (i, band) in pieces.into_iter().enumerate() {
        count += band.len();
        let colour = Color::True(band_colour(i, n));
        let mut rest = band;
        while !rest.is_empty() {
            let take = rest.len().min(MAX_LOOPS);
            let chunk: Vec<Vec<Vec2>> = rest.drain(..take).collect();
            let common = Common { layer: LAYER_HYPSO.into(), color: colour, ..s.current_common() };
            s.doc_mut()?.add(&space, common, solid(chunk)).map_err(|e| bad(CMD, e.to_string()))?;
        }
    }
    // Legend: one swatch per band, highest on top.
    let plan: Vec<Vec2> = tris.iter().flatten().map(|v| v.xy()).collect();
    let (_, pmax) = bbox(&plan);
    let k = scale / 1000.0;
    let at = match p.get("legend") {
        None | Some(Value::Null) => Vec2::new(pmax.x + 10.0 * k, pmax.y),
        Some(v) => point_value(v).filter(|a| a.x.is_finite() && a.y.is_finite()).ok_or_else(|| bad(CMD, "`legend` must be a point [x, y]"))?,
    };
    let (w, h) = (8.0 * k, 5.0 * k);
    let decimals = if step.fract().abs() < 1e-9 { 0 } else { 2 };
    for i in 0..n {
        let row = (n - 1 - i) as f64;
        let y0 = at.y - (row + 1.0) * h;
        let sw = vec![Vec2::new(at.x, y0), Vec2::new(at.x + w, y0), Vec2::new(at.x + w, y0 + h), Vec2::new(at.x, y0 + h)];
        let common = Common { layer: LAYER_HYPSO.into(), color: Color::True(band_colour(i, n)), ..s.current_common() };
        s.doc_mut()?.add(&space, common, solid(vec![sw])).map_err(|e| bad(CMD, e.to_string()))?;
        let (b0, b1) = (lo + i as f64 * step, lo + (i + 1) as f64 * step);
        text(
            s,
            &space,
            LAYER_HYPSO,
            Vec2::new(at.x + w + 2.0 * k, y0 + h / 2.0),
            2.5 * k,
            format!("{b0:.decimals$} – {b1:.decimals$} m").replace('.', ","),
            0.0,
            (HAlign::Left, VAlign::Middle),
        )?;
    }
    text(s, &space, LAYER_HYPSO, Vec2::new(at.x, at.y + 2.0 * k), 3.0 * k, "Výškové stupně".into(), 0.0, (HAlign::Left, VAlign::Bottom))?;
    Ok(json!({
        "bands": n, "pieces": count, "from": lo, "to": lo + n as f64 * step, "interval": step,
        "message": format!("{n} height bands of {step} m from {lo:.2} m"),
    }))
}

struct HypsoM;

impl Interactive for HypsoM {
    fn name(&self) -> &'static str {
        "GEOHYPSO"
    }
    fn prompt(&self, _s: &Session) -> Prompt {
        Prompt::new("Band height in metres, or Enter for about ten bands", Accept::NUMBER)
    }
    fn input(&mut self, s: &mut Session, i: Input) -> Result<Step> {
        let params = match i {
            Input::Cancel => return Ok(Step::Cancel),
            Input::Enter => json!({}),
            Input::Text(t) | Input::Keyword(t) => match typed_number(&t).filter(|v| *v > 0.0) {
                Some(v) => json!({ "interval": v }),
                None => {
                    s.echo("Enter the band height in metres, like 1 or 0.5.");
                    return Ok(Step::Continue);
                }
            },
            _ => return Ok(Step::Continue),
        };
        let r = run_geohypso(s, &params);
        say(s, r);
        Ok(Step::Done)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cadcraft_doc::Face3d;

    fn ramp() -> Vec<[Vec3; 3]> {
        let (a, b, c, d) = (Vec3::new(0.0, 0.0, 100.0), Vec3::new(20.0, 0.0, 104.0), Vec3::new(20.0, 20.0, 104.0), Vec3::new(0.0, 20.0, 100.0));
        vec![[a, b, c], [a, c, d]]
    }

    #[test]
    fn bands_cover_the_model_exactly() {
        let b = bands(&ramp(), 100.0, 1.0, 4).unwrap();
        assert_eq!(b.len(), 4);
        // Each 1 m band is a 5 m strip: 100 m².
        for band in &b {
            let a: f64 = band.iter().map(|p| signed_area(p).abs()).sum();
            assert!((a - 100.0).abs() < 1e-9, "{a}");
        }
        assert_eq!(nice_interval(7.3, 10.0), 1.0);
        assert_eq!(nice_interval(40.0, 10.0), 5.0);
        assert_eq!(nice_interval(0.0, 10.0), 1.0);
        assert_eq!(band_colour(0, 5), Rgb(60, 141, 47));
    }

    #[test]
    fn the_command_colours_and_writes_a_legend() {
        let mut s = Session::new();
        assert!(s.execute("geohypso", &json!({})).is_err());
        let space = s.space();
        for t in ramp() {
            let common = Common { layer: super::super::geo_dmt::LAYER_TIN.into(), ..s.current_common() };
            s.doc_mut().unwrap().add(&space, common, EntityKind::Face3d(Face3d { corners: [t[0], t[1], t[2], t[2]], hidden_edges: 0 })).unwrap();
        }
        let r = s.execute("geohypso", &json!({ "interval": 1 })).unwrap();
        assert_eq!(r["bands"], 5, "{r}");
        let texts: Vec<String> =
            s.doc().unwrap().model.iter().filter_map(|e| if let EntityKind::Text(t) = &e.kind { Some(t.value.clone()) } else { None }).collect();
        assert!(texts.iter().any(|t| t == "102 – 103 m"), "{texts:?}");
        // Again: the old colouring is replaced, not doubled.
        let before = s.doc().unwrap().model.iter().count();
        s.execute("geohypso", &json!({ "interval": 1 })).unwrap();
        assert_eq!(s.doc().unwrap().model.iter().count(), before);
        for p in [json!({ "interval": 0 }), json!({ "interval": 1e-9 }), json!({ "scale": -1 }), json!({ "legend": "x" })] {
            assert!(s.execute("geohypso", &p).is_err(), "{p}");
        }
    }
}
