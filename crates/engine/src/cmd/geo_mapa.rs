//! Survey add-on: drawing the map.
//!
//! - GEOZNACKA: a point map symbol (our own simple designs, kept as blocks named `GEO_Z_…` so
//!   they can be edited or redefined) at a point, sized for the map scale.
//! - GEOKRESBA: automatic drawing from the point codes GEOBODY imported: a code table says which
//!   code is a symbol and which a line; points of a line code are joined in number order. A code
//!   is letters with an optional number (PL1, PL2: two separate fences).
//! - GEOLINIE: a line symbol along a line or polyline: fence ticks, a wall's second face.
//! - GEOSVAH: slope hachures between the top and bottom edges of an embankment or cutting.
//! - GEOMLISTY: the map sheet grid of S-JTSK (SM5 and its 1:2000, 1:1000, 1:500 parts) over the
//!   drawing, each sheet labelled with its corner coordinates.

use std::collections::BTreeMap;
use std::sync::Arc;

use cadcraft_doc::{Block, Circle, Entity, EntityKind, HAlign, Handle, Insert, Line, LwPolyline, Polyline3d, Space, VAlign};
use cadcraft_geom::{PolyVertex, Vec2, Vec3};
use serde_json::{Value, json};

use super::geo::LAYER_CODES;
use super::geo_calc::{add, ensure_layer, one_target, path_of, say, text, typed_number};
use super::geo_pohled::numbered_points;
use super::{CommandSpec, bad, f64_or, point_value, str_param};
use crate::{Accept, Input, Interactive, Prompt, Result, Session, Step};

pub const LAYER_SYMBOLS: &str = "GEO_znacky";
pub const LAYER_SLOPE: &str = "GEO_svahy";
pub const LAYER_SHEETS: &str = "GEO_mapove_listy";

/// Sheets or hachures drawn at once.
const MAX_ITEMS: usize = 200_000;

/// The symbols: name, Czech description, and strokes in mm on paper around the insertion point
/// (circles as (x, y, r); polylines as point lists).
pub fn symbols() -> Vec<(&'static str, &'static str, Vec<(f64, f64, f64)>, Vec<Vec<(f64, f64)>>)> {
    vec![
        ("sachta", "šachta", vec![(0.0, 0.0, 1.25)], vec![]),
        (
            "vpust",
            "uliční vpusť",
            vec![],
            vec![vec![(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0), (-1.0, -1.0)], vec![(-1.0, -1.0), (1.0, 1.0)]],
        ),
        ("hydrant", "hydrant", vec![(0.0, 0.0, 0.9)], vec![vec![(0.0, 0.9), (0.0, 2.2)], vec![(-0.7, 2.2), (0.7, 2.2)]]),
        ("sloup", "sloup", vec![(0.0, 0.0, 0.8), (0.0, 0.0, 0.3)], vec![]),
        ("lampa", "lampa", vec![(0.0, 0.0, 0.7)], vec![vec![(0.0, 0.7), (0.0, 1.6)], vec![(0.6, 0.6), (1.2, 1.2)], vec![(-0.6, 0.6), (-1.2, 1.2)]]),
        ("strom_listnaty", "listnatý strom", vec![(0.0, 0.0, 1.6), (0.0, 0.0, 0.2)], vec![]),
        ("strom_jehlicnaty", "jehličnatý strom", vec![(0.0, 0.0, 1.6)], vec![vec![(0.0, 1.2), (1.0, -0.6), (-1.0, -0.6), (0.0, 1.2)]]),
        ("ker", "keř", vec![(-0.6, 0.0, 0.6), (0.6, 0.0, 0.6), (0.0, 0.8, 0.6)], vec![]),
        ("dopravni_znacka", "dopravní značka", vec![(0.0, 2.3, 0.7)], vec![vec![(0.0, 0.0), (0.0, 1.6)]]),
        ("pevny_bod", "pevný bod", vec![(0.0, 0.0, 0.25)], vec![vec![(0.0, 1.5), (1.3, -0.75), (-1.3, -0.75), (0.0, 1.5)]]),
        ("hranicni_znak", "hraniční znak", vec![(0.0, 0.0, 0.6)], vec![]),
        ("soupe", "šoupě", vec![], vec![vec![(-1.0, -0.6), (-1.0, 0.6), (1.0, -0.6), (1.0, 0.6), (-1.0, -0.6)]]),
    ]
}

/// The default code table: code letters → what they draw.
pub const DEFAULT_TABLE: &str = "\
SA symbol sachta
VP symbol vpust
HY symbol hydrant
SL symbol sloup
LA symbol lampa
ST symbol strom_listnaty
SJ symbol strom_jehlicnaty
KE symbol ker
DZ symbol dopravni_znacka
PB symbol pevny_bod
HZ symbol hranicni_znak
SO symbol soupe
BU linie budova uzavrit
PL linie plot
ZD linie zed
OB linie obrubnik
HV linie hrana_vozovky
CE linie cesta
HS linie horni_hrana_svahu
DS linie dolni_hrana_svahu
VO linie vodni_tok
";

pub(super) fn specs() -> Vec<CommandSpec> {
    vec![
        CommandSpec::new("geoznacka", "Map Symbol...", run_geoznacka)
            .menu(&["Geo", "Map", "Map Symbol..."])
            .alias(&["geosymbol"])
            .params("{symbol: name (sachta, vpust, hydrant, sloup, lampa, strom_listnaty, strom_jehlicnaty, ker, dopravni_znacka, pevny_bod, hranicni_znak, soupe), at: [x, y], rotation?: degrees, scale?: map scale 1:N (default 500)} → {handle, message}; without `symbol` it lists them")
            .interactive(|_| Ok(Box::new(ZnackaM::default()))),
        CommandSpec::new("geokresba", "Draw from Point Codes...", run_geokresba)
            .menu(&["Geo", "Map", "Draw from Point Codes..."])
            .alias(&["geocodes", "geoautokresba"])
            .params("{table?: lines \"CODE symbol name\" or \"CODE linie name\" (default: the built-in table, see `geokresba {\"show\": true}`), scale?: map scale 1:N (default 500), show?: bool, return the table} → {symbols, lines, unknown: [codes], message}")
            .interactive(|_| Ok(Box::new(RunM { name: "GEOKRESBA", cmd: "geokresba" }))),
        CommandSpec::new("geolinie", "Line Symbol...", run_geolinie)
            .menu(&["Geo", "Map", "Line Symbol..."])
            .alias(&["geoline"])
            .params("{path?: handle (default: the selected one), style: \"plot\" (fence ticks) | \"zed\" (wall, second face) | \"ziva\" (hedge), side?: \"left\" | \"right\" (default left), scale?: 1:N (default 500)} → {items, message}")
            .interactive(|_| Ok(Box::new(LinieM::default()))),
        CommandSpec::new("geosvah", "Slope Hachures...", run_geosvah)
            .menu(&["Geo", "Map", "Slope Hachures..."])
            .alias(&["geoslope"])
            .params("{top: handle of the top edge, bottom: handle of the bottom edge, scale?: 1:N (default 500), spacing?: mm on paper (default 2)} → {hachures, message}")
            .interactive(|_| Ok(Box::new(SvahM::default()))),
        CommandSpec::new("geomlisty", "Map Sheets...", run_geomlisty)
            .menu(&["Geo", "Map", "Map Sheets..."])
            .alias(&["geosheets"])
            .params("{scale?: 5000 (SM5, 2500 × 2000 m) | 2000 | 1000 | 500 (default 5000), from?: [x, y], to?: [x, y] the area (default: the drawing's GEO_body points)} → {sheets, message}")
            .interactive(|_| Ok(Box::new(NumM { name: "GEOMLISTY", cmd: "geomlisty", key: "scale", prompt: "Map sheet scale 1: (5000, 2000, 1000, 500)", default: 5000.0 }))),
    ]
}

/// Define the block for `symbol` if the drawing hasn't got it.
fn ensure_symbol(s: &mut Session, symbol: &str) -> Result<String> {
    let all = symbols();
    let (name, _, circles, polys) = all.iter().find(|(n, ..)| *n == symbol).ok_or_else(|| {
        bad("geoznacka", format!("unknown symbol \"{symbol}\": use {}", all.iter().map(|(n, ..)| *n).collect::<Vec<_>>().join(", ")))
    })?;
    let block = format!("GEO_Z_{name}");
    let d = s.doc_mut()?;
    if !d.blocks.contains_key(&block) {
        let mut blk = Block::new(&block);
        for (x, y, r) in circles {
            let h = d.new_handle();
            let mut e = Entity::new(h, EntityKind::Circle(Circle { center: Vec3::new(*x, *y, 0.0), radius: *r }));
            e.common.layer = "0".into();
            blk.entities.push(e);
        }
        for pl in polys {
            let h = d.new_handle();
            let mut e = Entity::new(
                h,
                EntityKind::LwPolyline(LwPolyline {
                    vertices: pl.iter().map(|(x, y)| PolyVertex::new(Vec2::new(*x, *y))).collect(),
                    closed: false,
                    const_width: 0.0,
                    elevation: 0.0,
                    plinegen: false,
                }),
            );
            e.common.layer = "0".into();
            blk.entities.push(e);
        }
        d.blocks.insert(block.clone(), Arc::new(blk));
    }
    Ok(block)
}

fn place_symbol(s: &mut Session, space: &Space, symbol: &str, at: Vec3, rotation: f64, scale: f64) -> Result<Handle> {
    let block = ensure_symbol(s, symbol)?;
    let k = scale / 1000.0;
    let ins = Insert {
        block,
        insert: at,
        scale: Vec3::new(k, k, k),
        rotation,
        attribs: Vec::new(),
        cols: 1,
        rows: 1,
        col_spacing: 0.0,
        row_spacing: 0.0,
        clip: None,
    };
    add(s, space, LAYER_SYMBOLS, EntityKind::Insert(ins))
}

fn map_scale(p: &Value, cmd: &str) -> Result<f64> {
    let scale = f64_or(p, "scale", 500.0);
    if !(scale.is_finite() && (1.0..=1e6).contains(&scale)) {
        return Err(bad(cmd, "`scale` must be a map scale like 500 or 1000"));
    }
    Ok(scale)
}

fn run_geoznacka(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "geoznacka";
    let Some(symbol) = str_param(p, "symbol") else {
        let list: Vec<Value> = symbols().iter().map(|(n, cz, ..)| json!({ "name": n, "description": cz })).collect();
        return Ok(json!({ "symbols": list, "message": format!("Symbols: {}", symbols().iter().map(|(n, ..)| *n).collect::<Vec<_>>().join(", ")) }));
    };
    let at = p.get("at").and_then(point_value).filter(|a| a.x.is_finite() && a.y.is_finite()).ok_or_else(|| bad(CMD, "`at` ([x, y]) is required"))?;
    let rotation = f64_or(p, "rotation", 0.0);
    if !rotation.is_finite() {
        return Err(bad(CMD, "`rotation` must be an angle in degrees"));
    }
    let scale = map_scale(p, CMD)?;
    let space = s.space();
    ensure_layer(s, LAYER_SYMBOLS, 7)?;
    let h = place_symbol(s, &space, symbol, Vec3::new(at.x, at.y, 0.0), rotation.to_radians(), scale)?;
    Ok(json!({ "handle": h.hex(), "message": format!("Symbol {symbol}") }))
}

/// What a code table entry draws.
#[derive(Clone, Debug, PartialEq)]
pub enum Draw {
    Symbol(String),
    /// A line style, and whether its last point joins the first (a building).
    Line(String, bool),
}

/// Parse a code table: "CODE symbol name" / "CODE linie name" per line, # comments.
pub fn parse_table(t: &str) -> std::result::Result<BTreeMap<String, Draw>, String> {
    let names: Vec<&str> = symbols().iter().map(|(n, ..)| *n).collect();
    let mut out = BTreeMap::new();
    for (i, line) in t.lines().enumerate() {
        let line = line.split('#').next().unwrap_or("").trim();
        if line.is_empty() {
            continue;
        }
        let f: Vec<&str> = line.split_whitespace().collect();
        let (Some(code), Some(kind), Some(name)) = (f.first(), f.get(1), f.get(2)) else {
            return Err(format!("line {}: expected \"CODE symbol name\" or \"CODE linie name\"", i + 1));
        };
        if code.len() > 16 || name.len() > 64 || !name.chars().all(|c| c.is_alphanumeric() || c == '_' || c == '-') {
            return Err(format!("line {}: code or name too long, or name with odd characters", i + 1));
        }
        let draw = match kind.to_ascii_lowercase().as_str() {
            "symbol" | "znacka" | "značka" => {
                if !names.contains(name) {
                    return Err(format!("line {}: unknown symbol \"{name}\"", i + 1));
                }
                Draw::Symbol((*name).to_string())
            }
            "linie" | "line" => {
                Draw::Line((*name).to_string(), f.get(3).is_some_and(|c| ["uzavrit", "uzavřít", "closed"].contains(&c.to_lowercase().as_str())))
            }
            other => return Err(format!("line {}: \"{other}\" is neither symbol nor linie", i + 1)),
        };
        out.insert(code.to_ascii_uppercase(), draw);
        if out.len() > 10_000 {
            return Err("at most 10 000 codes".into());
        }
    }
    Ok(out)
}

/// The letters of a code (its kind) and the whole code (which line it belongs to).
pub fn code_kind(code: &str) -> String {
    code.trim().chars().take_while(|c| c.is_alphabetic()).collect::<String>().to_uppercase()
}

fn number_key(n: &str) -> (u8, i64, String) {
    match n.trim().parse::<i64>() {
        Ok(v) => (0, v, String::new()),
        Err(_) => (1, 0, n.to_string()),
    }
}

fn run_geokresba(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "geokresba";
    if p.get("show").and_then(Value::as_bool).unwrap_or(false) {
        return Ok(json!({ "table": DEFAULT_TABLE, "message": "The built-in code table" }));
    }
    let table = parse_table(str_param(p, "table").unwrap_or(DEFAULT_TABLE)).map_err(|e| bad(CMD, e))?;
    let scale = map_scale(p, CMD)?;
    let space = s.space();
    let pts = numbered_points(s, &space, None)?;
    // Each point's code is the GEO_kody text GEOBODY wrote beside it.
    let codes: Vec<(Vec2, f64, String)> = {
        let d = s.doc()?;
        let st = d.space(&space).ok_or_else(|| bad(CMD, "no drawing space"))?;
        st.iter()
            .filter(|e| e.common.layer.eq_ignore_ascii_case(LAYER_CODES))
            .filter_map(|e| if let EntityKind::Text(t) = &e.kind { Some((t.insert.xy(), t.height, t.value.trim().to_string())) } else { None })
            .filter(|c| c.1.is_finite() && c.1 > 0.0)
            .collect()
    };
    let code_of = |q: Vec3| -> Option<String> {
        codes
            .iter()
            .filter_map(|(at, h, v)| {
                // GEOBODY puts the code at 0.6·h right and 2.75·h below the point, at 0.8 the size.
                let h0 = h / 0.8;
                let want = Vec2::new(q.x + 0.6 * h0, q.y - 2.75 * h0);
                let d = (at.x - want.x).hypot(at.y - want.y);
                (d < 0.5 * h0).then_some((d, v))
            })
            .min_by(|a, b| a.0.total_cmp(&b.0))
            .map(|(_, v)| v.clone())
    };
    let mut lines: BTreeMap<String, (String, bool, Vec<(String, Vec3)>)> = BTreeMap::new();
    let mut n_sym = 0;
    let mut unknown: Vec<String> = Vec::new();
    ensure_layer(s, LAYER_SYMBOLS, 7)?;
    for (num, q) in &pts {
        let Some(code) = code_of(*q) else { continue };
        if code.is_empty() {
            continue;
        }
        match table.get(&code_kind(&code)) {
            Some(Draw::Symbol(sym)) => {
                place_symbol(s, &space, sym, Vec3::new(q.x, q.y, 0.0), 0.0, scale)?;
                n_sym += 1;
            }
            Some(Draw::Line(name, closed)) => {
                lines.entry(code.to_uppercase()).or_insert_with(|| (name.clone(), *closed, Vec::new())).2.push((num.clone(), *q))
            }
            None => {
                if !unknown.contains(&code) && unknown.len() < 100 {
                    unknown.push(code.clone());
                }
            }
        }
    }
    let mut n_lines = 0;
    for (_, (name, closed, mut members)) in lines {
        if members.len() < 2 {
            continue;
        }
        members.sort_by_key(|a| number_key(&a.0));
        let layer = format!("GEO_L_{name}");
        ensure_layer(s, &layer, 7)?;
        let closed = closed && members.len() >= 3;
        let h = add(s, &space, &layer, EntityKind::Polyline3d(Polyline3d { points: members.into_iter().map(|(_, q)| q).collect(), closed }))?;
        // Fences and walls get their line symbol too.
        if name == "plot" || name == "zed" {
            run_geolinie(s, &json!({ "path": h.hex(), "style": name, "scale": scale }))?;
        }
        n_lines += 1;
    }
    Ok(json!({
        "symbols": n_sym, "lines": n_lines, "unknown": unknown,
        "message": format!("{n_sym} symbols and {n_lines} lines drawn from point codes{}", if unknown.is_empty() { String::new() } else { format!("; codes not in the table: {}", unknown.join(", ")) }),
    }))
}

/// Points along `path` every `step`, with the unit direction there.
fn along_every(path: &[Vec2], step: f64, offset: f64) -> Vec<(Vec2, Vec2)> {
    let mut out = Vec::new();
    let mut next = offset;
    let mut start = 0.0;
    for w in path.windows(2) {
        let [a, b] = w else { continue };
        let len = (b.x - a.x).hypot(b.y - a.y);
        if len < 1e-12 {
            continue;
        }
        let u = Vec2::new((b.x - a.x) / len, (b.y - a.y) / len);
        while next <= start + len + 1e-9 && out.len() < MAX_ITEMS {
            let t = next - start;
            out.push((Vec2::new(a.x + u.x * t, a.y + u.y * t), u));
            next += step;
        }
        start += len;
    }
    out
}

fn run_geolinie(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "geolinie";
    let space = s.space();
    let h = one_target(s, p, "path", CMD)?;
    let (mut path, closed) = path_of(s, h, &space).ok_or_else(|| bad(CMD, "the path must be a line or polyline"))?;
    if closed && let Some(f) = path.first().copied() {
        path.push(f);
    }
    let style = str_param(p, "style").unwrap_or("plot");
    let side = match str_param(p, "side").unwrap_or("left") {
        "left" | "vlevo" => 1.0,
        "right" | "vpravo" => -1.0,
        o => return Err(bad(CMD, format!("`side` must be left or right, not \"{o}\""))),
    };
    let k = map_scale(p, CMD)? / 1000.0;
    let layer = format!("GEO_L_{style}");
    ensure_layer(s, &layer, 7)?;
    let mut items = 0;
    match style {
        "plot" | "ziva" => {
            // Fence: ticks every 4 mm; hedge: small half circles every 2.5 mm.
            let step = if style == "plot" { 4.0 * k } else { 2.5 * k };
            for (q, u) in along_every(&path, step, step / 2.0) {
                let n = Vec2::new(-u.y * side, u.x * side);
                let kind = if style == "plot" {
                    EntityKind::Line(Line { a: Vec3::new(q.x, q.y, 0.0), b: Vec3::new(q.x + n.x * 1.2 * k, q.y + n.y * 1.2 * k, 0.0) })
                } else {
                    EntityKind::Circle(Circle { center: Vec3::new(q.x + n.x * 0.7 * k, q.y + n.y * 0.7 * k, 0.0), radius: 0.6 * k })
                };
                add(s, &space, &layer, kind)?;
                items += 1;
            }
        }
        "zed" => {
            // The wall's other face 0.6 mm aside.
            let off = super::geo_rek::offset(&path, false, 0.6 * k * side);
            if let Some(off) = off.filter(|o| o.len() >= 2) {
                let pl = LwPolyline {
                    vertices: off.into_iter().map(PolyVertex::new).collect(),
                    closed: false,
                    const_width: 0.0,
                    elevation: 0.0,
                    plinegen: false,
                };
                add(s, &space, &layer, EntityKind::LwPolyline(pl))?;
                items = 1;
            }
        }
        o => return Err(bad(CMD, format!("unknown style \"{o}\": use plot, zed or ziva"))),
    }
    Ok(json!({ "items": items, "message": format!("Line symbol {style}: {items} parts") }))
}

/// The point of `path` nearest to `q`.
fn nearest_on(path: &[Vec2], q: Vec2) -> Option<Vec2> {
    path.windows(2)
        .filter_map(|w| {
            let [a, b] = w else { return None };
            let (dx, dy) = (b.x - a.x, b.y - a.y);
            let l2 = dx * dx + dy * dy;
            let t = if l2 > 0.0 { (((q.x - a.x) * dx + (q.y - a.y) * dy) / l2).clamp(0.0, 1.0) } else { 0.0 };
            let p = Vec2::new(a.x + t * dx, a.y + t * dy);
            Some(((p.x - q.x).hypot(p.y - q.y), p))
        })
        .min_by(|a, b| a.0.total_cmp(&b.0))
        .map(|(_, p)| p)
}

fn run_geosvah(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "geosvah";
    let space = s.space();
    let top = one_target(s, p, "top", CMD)?;
    let bottom = one_target(s, p, "bottom", CMD)?;
    let (tp, _) = path_of(s, top, &space).ok_or_else(|| bad(CMD, "the top edge must be a line or polyline"))?;
    let (bp, _) = path_of(s, bottom, &space).ok_or_else(|| bad(CMD, "the bottom edge must be a line or polyline"))?;
    if tp.len() < 2 || bp.len() < 2 {
        return Err(bad(CMD, "both edges need at least two vertices"));
    }
    let k = map_scale(p, CMD)? / 1000.0;
    let spacing = f64_or(p, "spacing", 2.0);
    if !(spacing.is_finite() && spacing >= 0.2) {
        return Err(bad(CMD, "`spacing` must be at least 0.2 mm"));
    }
    ensure_layer(s, LAYER_SLOPE, 7)?;
    let mut n = 0;
    for (i, (q, _)) in along_every(&tp, spacing * k, 0.0).into_iter().enumerate() {
        let Some(b) = nearest_on(&bp, q) else { continue };
        // Long and short hachures by turns, from the top down.
        let f = if i % 2 == 0 { 1.0 } else { 0.5 };
        let end = Vec2::new(q.x + (b.x - q.x) * f, q.y + (b.y - q.y) * f);
        if (end.x - q.x).hypot(end.y - q.y) < 1e-9 {
            continue;
        }
        add(s, &space, LAYER_SLOPE, EntityKind::Line(Line { a: Vec3::new(q.x, q.y, 0.0), b: Vec3::new(end.x, end.y, 0.0) }))?;
        n += 1;
    }
    Ok(json!({ "hachures": n, "message": format!("{n} slope hachures") }))
}

/// Sheet size in metres (along Y, along X) for a map scale.
pub fn sheet_size(scale: f64) -> Option<(f64, f64)> {
    match scale.round() as i64 {
        5000 => Some((2500.0, 2000.0)),
        2000 => Some((1250.0, 1000.0)),
        1000 => Some((625.0, 500.0)),
        500 => Some((312.5, 250.0)),
        _ => None,
    }
}

fn run_geomlisty(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "geomlisty";
    let scale = f64_or(p, "scale", 5000.0);
    let (wy, wx) = sheet_size(scale).ok_or_else(|| bad(CMD, "`scale` must be 5000, 2000, 1000 or 500"))?;
    let space = s.space();
    let (lo, hi) = match (p.get("from").and_then(point_value), p.get("to").and_then(point_value)) {
        (Some(a), Some(b)) => (Vec2::new(a.x.min(b.x), a.y.min(b.y)), Vec2::new(a.x.max(b.x), a.y.max(b.y))),
        _ => {
            let pts = numbered_points(s, &space, None)?;
            if pts.is_empty() {
                return Err(bad(CMD, "give the area with `from` and `to`, or import points first"));
            }
            pts.iter().fold((Vec2::new(f64::INFINITY, f64::INFINITY), Vec2::new(f64::NEG_INFINITY, f64::NEG_INFINITY)), |(l, h), (_, q)| {
                (Vec2::new(l.x.min(q.x), l.y.min(q.y)), Vec2::new(h.x.max(q.x), h.y.max(q.y)))
            })
        }
    };
    if !(lo.x.is_finite() && lo.y.is_finite() && hi.x.is_finite() && hi.y.is_finite()) {
        return Err(bad(CMD, "the area has no extent"));
    }
    if !(lo.x < -300_000.0 && lo.y < -900_000.0) {
        return Err(bad(CMD, "map sheets need S-JTSK coordinates"));
    }
    // In the drawing x = −Y and y = −X: sheets lie on whole multiples of their size in Y and X.
    let (y0, y1) = ((-hi.x / wy).floor(), (-lo.x / wy).ceil());
    let (x0, x1) = ((-hi.y / wx).floor(), (-lo.y / wx).ceil());
    let count = (y1 - y0) * (x1 - x0);
    if !(count.is_finite() && count >= 0.0 && count <= MAX_ITEMS as f64) {
        return Err(bad(CMD, "too many sheets"));
    }
    ensure_layer(s, LAYER_SHEETS, 4)?;
    let th = wx / 40.0;
    let mut n = 0;
    for iy in (y0 as i64)..(y1 as i64) {
        for ix in (x0 as i64)..(x1 as i64) {
            let (ya, xa) = (iy as f64 * wy, ix as f64 * wx);
            let corners = [Vec2::new(-ya, -xa), Vec2::new(-(ya + wy), -xa), Vec2::new(-(ya + wy), -(xa + wx)), Vec2::new(-ya, -(xa + wx))];
            let pl = LwPolyline {
                vertices: corners.into_iter().map(PolyVertex::new).collect(),
                closed: true,
                const_width: 0.0,
                elevation: 0.0,
                plinegen: false,
            };
            add(s, &space, LAYER_SHEETS, EntityKind::LwPolyline(pl))?;
            let c = Vec2::new(-(ya + wy / 2.0), -(xa + wx / 2.0));
            text(s, &space, LAYER_SHEETS, c, th, format!("Y {ya:.1}–{:.1}", ya + wy), 0.0, (HAlign::Center, VAlign::Bottom))?;
            text(
                s,
                &space,
                LAYER_SHEETS,
                Vec2::new(c.x, c.y - 0.3 * th),
                th,
                format!("X {xa:.1}–{:.1}", xa + wx),
                0.0,
                (HAlign::Center, VAlign::Top),
            )?;
            n += 1;
        }
    }
    Ok(json!({ "sheets": n, "message": format!("{n} map sheets 1:{}", scale.round()) }))
}

// --- prompts -------------------------------------------------------------------------------------

#[derive(Default)]
struct ZnackaM {
    symbol: Option<String>,
}

impl Interactive for ZnackaM {
    fn name(&self) -> &'static str {
        "GEOZNACKA"
    }
    fn prompt(&self, _s: &Session) -> Prompt {
        match &self.symbol {
            None => Prompt::new("Symbol (sachta, vpust, hydrant, sloup, lampa, strom_listnaty, strom_jehlicnaty, ker, dopravni_znacka, pevny_bod, hranicni_znak, soupe)", Accept::TEXT)
                .default("sachta"),
            Some(_) => Prompt::new("Insertion point", Accept::POINT),
        }
    }
    fn input(&mut self, s: &mut Session, i: Input) -> Result<Step> {
        match (&self.symbol, i) {
            (_, Input::Cancel) => Ok(Step::Cancel),
            (None, Input::Enter) => {
                self.symbol = Some("sachta".into());
                Ok(Step::Continue)
            }
            (None, Input::Text(t) | Input::Keyword(t)) => {
                let t = t.trim().to_lowercase();
                if symbols().iter().any(|(n, ..)| *n == t) {
                    self.symbol = Some(t);
                } else {
                    s.echo("Unknown symbol.");
                }
                Ok(Step::Continue)
            }
            (Some(sym), Input::Point(at)) => {
                let r = run_geoznacka(s, &json!({ "symbol": sym, "at": [at.x, at.y] }));
                say(s, r);
                Ok(Step::Continue)
            }
            (Some(_), Input::Enter) => Ok(Step::Done),
            _ => Ok(Step::Continue),
        }
    }
}

struct RunM {
    name: &'static str,
    cmd: &'static str,
}

impl Interactive for RunM {
    fn name(&self) -> &'static str {
        self.name
    }
    fn prompt(&self, _s: &Session) -> Prompt {
        Prompt::new("Map scale 1:", Accept::NUMBER).default("500")
    }
    fn input(&mut self, s: &mut Session, i: Input) -> Result<Step> {
        let scale = match i {
            Input::Cancel => return Ok(Step::Cancel),
            Input::Enter => 500.0,
            Input::Text(t) | Input::Keyword(t) => match typed_number(&t).filter(|v| *v >= 1.0) {
                Some(v) => v,
                None => {
                    s.echo("Enter the map scale, like 500.");
                    return Ok(Step::Continue);
                }
            },
            _ => return Ok(Step::Continue),
        };
        let r = s.execute(self.cmd, &json!({ "scale": scale }));
        say(s, r);
        Ok(Step::Done)
    }
}

struct NumM {
    name: &'static str,
    cmd: &'static str,
    key: &'static str,
    prompt: &'static str,
    default: f64,
}

impl Interactive for NumM {
    fn name(&self) -> &'static str {
        self.name
    }
    fn prompt(&self, _s: &Session) -> Prompt {
        Prompt::new(self.prompt, Accept::NUMBER).default(format!("{}", self.default))
    }
    fn input(&mut self, s: &mut Session, i: Input) -> Result<Step> {
        let v = match i {
            Input::Cancel => return Ok(Step::Cancel),
            Input::Enter => self.default,
            Input::Text(t) | Input::Keyword(t) => match typed_number(&t) {
                Some(v) => v,
                None => return Ok(Step::Continue),
            },
            _ => return Ok(Step::Continue),
        };
        let mut m = serde_json::Map::new();
        m.insert(self.key.into(), json!(v));
        let r = s.execute(self.cmd, &Value::Object(m));
        say(s, r);
        Ok(Step::Done)
    }
}

#[derive(Default)]
struct LinieM {
    path: Option<Handle>,
}

impl Interactive for LinieM {
    fn name(&self) -> &'static str {
        "GEOLINIE"
    }
    fn prompt(&self, _s: &Session) -> Prompt {
        match self.path {
            None => Prompt::new("Select the line", Accept::SELECT),
            Some(_) => Prompt::new("Style [plot/zed/ziva]", Accept::TEXT).default("plot"),
        }
    }
    fn input(&mut self, s: &mut Session, i: Input) -> Result<Step> {
        match (self.path, i) {
            (_, Input::Cancel) => Ok(Step::Cancel),
            (None, Input::Pick(hs)) => {
                self.path = hs.first().copied();
                Ok(Step::Continue)
            }
            (Some(h), Input::Enter) => {
                let r = run_geolinie(s, &json!({ "path": h.hex(), "style": "plot" }));
                say(s, r);
                Ok(Step::Done)
            }
            (Some(h), Input::Text(t) | Input::Keyword(t)) => {
                let r = run_geolinie(s, &json!({ "path": h.hex(), "style": t.trim().to_lowercase() }));
                say(s, r);
                Ok(Step::Done)
            }
            _ => Ok(Step::Continue),
        }
    }
}

#[derive(Default)]
struct SvahM {
    top: Option<Handle>,
}

impl Interactive for SvahM {
    fn name(&self) -> &'static str {
        "GEOSVAH"
    }
    fn prompt(&self, _s: &Session) -> Prompt {
        match self.top {
            None => Prompt::new("Select the top edge of the slope", Accept::SELECT),
            Some(_) => Prompt::new("Select the bottom edge", Accept::SELECT),
        }
    }
    fn input(&mut self, s: &mut Session, i: Input) -> Result<Step> {
        match (self.top, i) {
            (_, Input::Cancel) => Ok(Step::Cancel),
            (None, Input::Pick(hs)) => {
                self.top = hs.first().copied();
                Ok(Step::Continue)
            }
            (Some(t), Input::Pick(hs)) => match hs.first() {
                Some(b) => {
                    let r = run_geosvah(s, &json!({ "top": t.hex(), "bottom": b.hex() }));
                    say(s, r);
                    Ok(Step::Done)
                }
                None => Ok(Step::Continue),
            },
            _ => Ok(Step::Continue),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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
    fn symbols_are_blocks_at_map_scale() {
        let mut s = Session::new();
        let r = s.execute("geoznacka", &json!({ "symbol": "hydrant", "at": [10, 20], "scale": 500 })).unwrap();
        let h = Handle::parse_hex(r["handle"].as_str().unwrap()).unwrap();
        let d = s.doc().unwrap();
        let EntityKind::Insert(ins) = &d.model.get(h).unwrap().kind else { panic!("not an insert") };
        assert_eq!(ins.block, "GEO_Z_hydrant");
        assert!((ins.scale.x - 0.5).abs() < 1e-12);
        assert!(d.blocks.get("GEO_Z_hydrant").is_some_and(|b| !b.entities.is_empty()));
        assert!(s.execute("geoznacka", &json!({})).unwrap()["symbols"].as_array().unwrap().len() >= 12);
        assert!(s.execute("geoznacka", &json!({ "symbol": "drak", "at": [0, 0] })).is_err());
    }

    #[test]
    fn drawing_from_codes() {
        let mut s = Session::new();
        let list = "1 0 0 300 PL1\n2 10 0 300 PL1\n3 20 5 300 PL1\n4 0 10 300 PL2\n5 5 10 300 PL2\n6 3 3 300 SA\n7 6 6 300 XX\n";
        s.execute("geobody", &json!({ "text": list, "sjtsk": false })).unwrap();
        let r = s.execute("geokresba", &json!({})).unwrap();
        assert_eq!(r["symbols"], 1, "{r}");
        assert_eq!(r["lines"], 2, "{r}");
        assert_eq!(r["unknown"], json!(["XX"]));
        let d = s.doc().unwrap();
        let fences: Vec<usize> = d
            .model
            .iter()
            .filter(|e| e.common.layer == "GEO_L_plot")
            .filter_map(|e| if let EntityKind::Polyline3d(p) = &e.kind { Some(p.points.len()) } else { None })
            .collect();
        assert!(fences.contains(&3) && fences.contains(&2), "{fences:?}");
        // A custom table.
        let r = s.execute("geokresba", &json!({ "table": "XX symbol strom_listnaty" })).unwrap();
        assert_eq!(r["symbols"], 1);
        for t in ["XX", "XX symbol drak", "XX tvar plot", &format!("XX linie {}", "a".repeat(100))] {
            assert!(parse_table(t).is_err(), "{t}");
        }
        assert_eq!(code_kind("pl12"), "PL");
    }

    #[test]
    fn line_symbols_and_slopes() {
        let mut s = Session::new();
        let fence = pline(&mut s, &[(0.0, 0.0), (20.0, 0.0)]);
        let r = s.execute("geolinie", &json!({ "path": fence, "style": "plot", "scale": 500 })).unwrap();
        // Ticks every 2 m from 1 m: 10 of them.
        assert_eq!(r["items"], 10, "{r}");
        assert_eq!(s.execute("geolinie", &json!({ "path": fence, "style": "zed" })).unwrap()["items"], 1);
        assert!(s.execute("geolinie", &json!({ "path": fence, "style": "x" })).is_err());
        let top = pline(&mut s, &[(0.0, 10.0), (10.0, 10.0)]);
        let bottom = pline(&mut s, &[(0.0, 5.0), (10.0, 5.0)]);
        let r = s.execute("geosvah", &json!({ "top": top, "bottom": bottom, "scale": 500 })).unwrap();
        // Every 1 m along 10 m: 11 hachures.
        assert_eq!(r["hachures"], 11, "{r}");
    }

    #[test]
    fn sm5_sheets_cover_the_points() {
        let mut s = Session::new();
        s.execute("geobody", &json!({ "text": "1 742100 1043100 300\n2 744900 1043900 300\n" })).unwrap();
        let r = s.execute("geomlisty", &json!({})).unwrap();
        // Y 742100..744900 spans sheets 740000–742500, 742500–745000; X within 1042000–1044000.
        assert_eq!(r["sheets"], 2, "{r}");
        let r = s.execute("geomlisty", &json!({ "scale": 2000 })).unwrap();
        assert_eq!(r["sheets"], 3, "{r}");
        assert!(s.execute("geomlisty", &json!({ "scale": 1234 })).is_err());
        let mut local = Session::new();
        assert!(local.execute("geomlisty", &json!({ "from": [0, 0], "to": [10, 10] })).is_err());
    }
}
