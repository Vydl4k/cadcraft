//! Survey add-on: points from a coordinate list (GEOBODY).
//!
//! A coordinate list is a text file with one point per line: number, Y, X, Z and code in some
//! order, separated by spaces, tabs, commas or semicolons (a decimal comma is accepted when the
//! separator isn't a comma). Lines that don't hold a point (headers, comments) are skipped and
//! counted.
//!
//! Czech S-JTSK coordinates are positive (Y about 400 000–950 000 m, X about 900 000–1 350 000 m)
//! and their axes point west and south; in the drawing they become x = −Y, y = −X, so north is up,
//! as Czech survey drawings are drawn. `sjtsk: "auto"` (the default) does that when the
//! coordinates fall in S-JTSK's range.
//!
//! Each point is drawn as a POINT (with its height as Z) on `GEO_body`, its number on
//! `GEO_cisla`, its height on `GEO_vysky` and its code (when it has one) on `GEO_kody`. How
//! they are written comes from the drawing's survey settings (GEONASTAVENI): each kind of label
//! has its text style and its height on paper (2 mm by default, 1 m at 1:500, the drawing in
//! metres), the number sits on a chosen side of the point, heights are written whole or in part,
//! and a number already taken is kept, skipped, renumbered, replaced or averaged. A call's own
//! parameters override the settings and become the drawing's settings.
//!
//! `only` loads just some points: numbers, ranges and prefixes (`"1-100, 205, 40*"`). A second
//! height column (`Z2`, e.g. the bottom of a pit) is written under the code. Twelve-digit
//! cadastral point numbers (cadastral area, ZPMZ, point) can be shortened to `ZPMZ-point`.

use std::collections::{BTreeMap, BTreeSet};
use std::sync::Arc;

use cadcraft_doc::{Attrib, Block, Common, Entity, EntityKind, HAlign, Handle, Insert, Point, Text, VAlign};
use cadcraft_geom::{Vec2, Vec3};
use serde_json::{Value, json};

use super::file::base64_decode;
use super::geo_nastaveni::{self as nast, Duplicates, Nastaveni, Pismo};
use super::geo_pohled::{POINT_BLOCK, point_labels};
use super::{CommandSpec, bad, str_param};
use crate::{Accept, EngineError, Input, Interactive, Prompt, Result, Session, Step};

const CMD: &str = "geobody";
/// Largest coordinate list read.
const MAX_BYTES: u64 = 64 << 20;
/// Points imported at once.
const MAX_POINTS: usize = 500_000;

pub const LAYER_POINTS: &str = "GEO_body";
pub const LAYER_NUMBERS: &str = "GEO_cisla";
pub const LAYER_HEIGHTS: &str = "GEO_vysky";
pub const LAYER_CODES: &str = "GEO_kody";
pub const LAYER_HEIGHTS2: &str = "GEO_vysky2";

pub(super) fn specs() -> Vec<CommandSpec> {
    vec![
        CommandSpec::new(CMD, "Import Points (Coordinate List)...", run_geobody)
            .menu(&["Geo", "Import Points..."])
            .alias(&["-geobody", "geopoints"])
            .params("{path | data: base64 | text, columns?: order of P(number) Y X Z Z2 K(code) and -(skip), default \"PYXZK\", sjtsk?: \"auto\"|true|false (x=-Y, y=-X; default auto), only?: numbers to load, e.g. \"1-100, 205, 40*\", and any GEONASTAVENI setting (scale, decimals, fonts, corner, digits, duplicates, ku, z3d, symbols, block, numbers, heights, codes), which then stays the drawing's} → {imported, skipped, duplicates, sjtsk, extents, message}")
            .interactive(|_| Ok(Box::new(BodyM::default()))),
    ]
}

/// One surveyed point.
#[derive(Clone, Debug, PartialEq)]
pub struct Bod {
    pub cislo: String,
    pub y: f64,
    pub x: f64,
    pub z: Option<f64>,
    pub kod: String,
    /// A second height (`Z2` column).
    pub z2: Option<f64>,
}

/// What a column of the list holds.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Col {
    Number,
    Y,
    X,
    Z,
    Z2,
    Code,
    Skip,
}

fn columns(spec: &str) -> std::result::Result<Vec<Col>, String> {
    let mut cols = Vec::new();
    let mut chars = spec.chars().filter(|c| !c.is_whitespace()).peekable();
    while let Some(c) = chars.next() {
        cols.push(match c.to_ascii_uppercase() {
            'P' | 'C' | 'N' => Col::Number,
            'Y' => Col::Y,
            'X' => Col::X,
            'Z' | 'H' if chars.peek() == Some(&'2') => {
                chars.next();
                Col::Z2
            }
            'Z' | 'H' => Col::Z,
            'W' => Col::Z2,
            'K' => Col::Code,
            '-' | '_' => Col::Skip,
            other => return Err(format!("unknown column \"{other}\" in \"{spec}\": use P (number), Y, X, Z, Z2, K (code) and - (skip)")),
        });
        if cols.len() > 16 {
            return Err("at most 16 columns".into());
        }
    }
    if !cols.contains(&Col::Y) || !cols.contains(&Col::X) {
        return Err(format!("the columns \"{spec}\" need Y and X"));
    }
    Ok(cols)
}

/// Text of a coordinate list: UTF-8, else Windows-1250 (Czech Windows programs).
pub(super) fn decode(bytes: &[u8]) -> String {
    let bytes = bytes.strip_prefix(b"\xEF\xBB\xBF").unwrap_or(bytes);
    match std::str::from_utf8(bytes) {
        Ok(s) => s.to_string(),
        Err(_) => cadcraft_dxf::decode_code_page(bytes, "ANSI_1250").unwrap_or_else(|| String::from_utf8_lossy(bytes).into_owned()),
    }
}

fn number(t: &str, comma_decimal: bool) -> Option<f64> {
    let t = t.trim();
    let v: f64 = if comma_decimal { t.replace(',', ".").parse().ok()? } else { t.parse().ok()? };
    v.is_finite().then_some(v)
}

/// The points of a coordinate list, and how many non-empty lines held none.
pub fn parse(text: &str, spec: &str) -> std::result::Result<(Vec<Bod>, usize), String> {
    let cols = columns(spec)?;
    let mut out = Vec::new();
    let mut skipped = 0;
    for line in text.lines() {
        let line = line.trim();
        if line.is_empty() {
            continue;
        }
        if line.starts_with(['#', ';', '*', '%']) || line.starts_with("//") {
            skipped += 1;
            continue;
        }
        let (fields, comma_decimal): (Vec<&str>, bool) = if line.contains(';') {
            (line.split(';').map(str::trim).collect(), true)
        } else if line.contains('\t') {
            (line.split('\t').map(str::trim).collect(), true)
        } else if line.contains(',') && line.split(',').count() >= 3 {
            (line.split(',').map(str::trim).collect(), false)
        } else {
            (line.split_whitespace().collect(), false)
        };
        let mut b = Bod { cislo: String::new(), y: f64::NAN, x: f64::NAN, z: None, kod: String::new(), z2: None };
        for (i, c) in cols.iter().enumerate() {
            let f = fields.get(i).copied().unwrap_or("");
            match c {
                Col::Number => b.cislo = f.chars().take(64).collect(),
                Col::Y => b.y = number(f, comma_decimal).unwrap_or(f64::NAN),
                Col::X => b.x = number(f, comma_decimal).unwrap_or(f64::NAN),
                Col::Z => b.z = if f.is_empty() { None } else { number(f, comma_decimal) },
                Col::Z2 => b.z2 = if f.is_empty() { None } else { number(f, comma_decimal) },
                Col::Code => {
                    // The code is the rest of the line when it's the last column (codes may hold spaces).
                    let rest = if i + 1 == cols.len() { fields.get(i..).map(|r| r.join(" ")).unwrap_or_default() } else { f.to_string() };
                    b.kod = rest.chars().take(64).collect();
                }
                Col::Skip => {}
            }
        }
        if !b.y.is_finite() || !b.x.is_finite() || b.y.abs() > 1e9 || b.x.abs() > 1e9 {
            skipped += 1;
            continue;
        }
        if out.len() >= MAX_POINTS {
            return Err(format!("the list has more than {MAX_POINTS} points"));
        }
        out.push(b);
    }
    Ok((out, skipped))
}

/// Whether the points look like S-JTSK (positive Y and X in its range).
pub fn looks_sjtsk(pts: &[Bod]) -> bool {
    !pts.is_empty() && pts.iter().all(|b| (300_000.0..=1_000_000.0).contains(&b.y) && (900_000.0..=1_400_000.0).contains(&b.x))
}

fn read_path(path: &str) -> Result<Vec<u8>> {
    #[cfg(not(target_arch = "wasm32"))]
    {
        let p = std::path::Path::new(path);
        let len = std::fs::metadata(p).map_err(|e| bad(CMD, format!("{path}: {e}")))?.len();
        if len > MAX_BYTES {
            return Err(bad(CMD, format!("{path} is larger than {} MB", MAX_BYTES >> 20)));
        }
        std::fs::read(p).map_err(|e| bad(CMD, format!("{path}: {e}")))
    }
    #[cfg(target_arch = "wasm32")]
    {
        let _ = (path, MAX_BYTES);
        Err(bad(CMD, "paths are not available on the web; pass `data` or `text`"))
    }
}

/// Which point numbers to load: numbers, `a-b` ranges and `prefix*`.
#[derive(Debug, PartialEq)]
enum Sel {
    One(String),
    Range(u64, u64),
    Prefix(String),
}

fn parse_only(t: &str) -> std::result::Result<Vec<Sel>, String> {
    let mut out = Vec::new();
    for part in t.split([',', ';']).map(str::trim).filter(|p| !p.is_empty()) {
        if out.len() >= 1000 {
            return Err("at most 1000 parts in `only`".into());
        }
        if let Some(p) = part.strip_suffix('*') {
            out.push(Sel::Prefix(p.trim().to_string()));
        } else if let Some((a, b)) = part.split_once('-')
            && let (Ok(a), Ok(b)) = (a.trim().parse::<u64>(), b.trim().parse::<u64>())
            && a <= b
        {
            // "1250-2" (descending) is a cadastral number, not a range.
            out.push(Sel::Range(a, b));
        } else {
            out.push(Sel::One(part.to_string()));
        }
    }
    if out.is_empty() {
        return Err("`only` names no points".into());
    }
    Ok(out)
}

fn selected(sel: &[Sel], number: &str) -> bool {
    sel.iter().any(|s| match s {
        Sel::One(n) => n == number,
        Sel::Prefix(p) => number.starts_with(p.as_str()),
        Sel::Range(a, b) => number.parse::<u64>().is_ok_and(|n| (*a..=*b).contains(&n)),
    })
}

/// A twelve-digit cadastral point number (3 digits cadastral area, 5 ZPMZ, 4 point) as
/// `ZPMZ-point` without leading zeros (just the point when the ZPMZ is 0); other numbers as they are.
pub fn ku_number(n: &str) -> String {
    if n.len() != 12 || !n.bytes().all(|b| b.is_ascii_digit()) {
        return n.to_string();
    }
    let trim = |s: &str| {
        let t = s.trim_start_matches('0');
        if t.is_empty() { "0".to_string() } else { t.to_string() }
    };
    let (zpmz, bod) = (n.get(3..8).unwrap_or(""), n.get(8..12).unwrap_or(""));
    if zpmz.bytes().all(|b| b == b'0') { trim(bod) } else { format!("{}-{}", trim(zpmz), trim(bod)) }
}

/// Apply the duplicate rule. `taken`: numbers already in the drawing. Returns how many points
/// had a number already taken.
fn resolve_duplicates(pts: Vec<Bod>, rule: Duplicates, taken: &BTreeSet<String>) -> (Vec<Bod>, usize) {
    let mut seen: BTreeSet<String> = taken.clone();
    let mut dups = 0;
    match rule {
        Duplicates::Keep => {
            for b in &pts {
                if !b.cislo.is_empty() && !seen.insert(b.cislo.clone()) {
                    dups += 1;
                }
            }
            (pts, dups)
        }
        Duplicates::Skip => {
            let mut out = Vec::with_capacity(pts.len());
            for b in pts {
                if b.cislo.is_empty() || seen.insert(b.cislo.clone()) {
                    out.push(b);
                } else {
                    dups += 1;
                }
            }
            (out, dups)
        }
        Duplicates::Renumber => {
            let mut next = seen.iter().chain(pts.iter().map(|b| &b.cislo)).filter_map(|n| n.parse::<u64>().ok()).max().unwrap_or(0);
            let mut out = Vec::with_capacity(pts.len());
            for mut b in pts {
                if !b.cislo.is_empty() && seen.contains(&b.cislo) {
                    dups += 1;
                    if b.cislo.parse::<u64>().is_ok() {
                        next = next.saturating_add(1);
                        b.cislo = next.to_string();
                    } else {
                        let base = b.cislo.clone();
                        let mut k = 2u32;
                        while seen.contains(&format!("{base}_{k}")) && k < u32::MAX {
                            k += 1;
                        }
                        b.cislo = format!("{base}_{k}");
                    }
                }
                seen.insert(b.cislo.clone());
                out.push(b);
            }
            (out, dups)
        }
        Duplicates::Replace | Duplicates::Average => {
            // Within the list: one point per number, where its number first appears.
            let mut groups: BTreeMap<String, Vec<Bod>> = BTreeMap::new();
            let mut order: Vec<Option<String>> = Vec::new();
            let mut unnamed: Vec<Bod> = Vec::new();
            for b in pts {
                if b.cislo.is_empty() {
                    order.push(None);
                    unnamed.push(b);
                    continue;
                }
                let g = groups.entry(b.cislo.clone()).or_default();
                if g.is_empty() {
                    order.push(Some(b.cislo.clone()));
                } else {
                    dups += 1;
                }
                g.push(b);
            }
            let mut unnamed = unnamed.into_iter();
            let mut out = Vec::with_capacity(order.len());
            for o in order {
                match o {
                    None => out.extend(unnamed.next()),
                    Some(n) => {
                        let Some(g) = groups.remove(&n) else { continue };
                        if rule == Duplicates::Replace {
                            out.extend(g.into_iter().last());
                            continue;
                        }
                        let k = g.len() as f64;
                        let mean = |f: &dyn Fn(&Bod) -> Option<f64>| {
                            let v: Vec<f64> = g.iter().filter_map(f).collect();
                            (!v.is_empty()).then(|| v.iter().sum::<f64>() / v.len() as f64)
                        };
                        let mut b = g.first().cloned().unwrap_or(Bod { cislo: n.clone(), y: 0.0, x: 0.0, z: None, kod: String::new(), z2: None });
                        b.y = g.iter().map(|b| b.y).sum::<f64>() / k;
                        b.x = g.iter().map(|b| b.x).sum::<f64>() / k;
                        b.z = mean(&|b: &Bod| b.z);
                        b.z2 = mean(&|b: &Bod| b.z2);
                        out.push(b);
                    }
                }
            }
            (out, dups)
        }
    }
}

/// Where GEOBODY puts a point's labels, in drawing units from the point.
struct Layout {
    hn: f64,
    hz: f64,
    hk: f64,
    dx: f64,
    /// The row just above the point and the one just below it (number and height, swapped
    /// when the number goes below).
    upper: f64,
    lower: f64,
    code_y: f64,
    z2_y: f64,
}

impl Layout {
    fn of(n: &Nastaveni, corner: nast::Corner) -> Self {
        let (hn, hz, hk) = (n.height(&n.numbers), n.height(&n.heights), n.height(&n.codes));
        let dx = if corner.left() { -0.6 * hn } else { 0.6 * hn };
        let (upper, lower) = (0.25 * hn, -0.25 * hn - hz.max(hn));
        let code_y = lower - 0.5 * hz - hk;
        Layout { hn, hz, hk, dx, upper, lower, code_y, z2_y: code_y - 0.5 * hk - hz }
    }
}

/// Where a label on `layer` may sit relative to its point, in units of its own height, for any
/// side of the point (the settings may have changed since the points were loaded).
pub(super) fn label_slots(n: &Nastaveni, layer: &str) -> Vec<Vec2> {
    use nast::Corner::*;
    let mut out = Vec::new();
    for c in [RightUp, RightDown, LeftUp, LeftDown] {
        let l = Layout::of(n, c);
        let (num_y, z_y) = if c.number_below() { (l.lower, l.upper) } else { (l.upper, l.lower) };
        let (y, h) = match layer {
            LAYER_NUMBERS => (num_y, l.hn),
            LAYER_HEIGHTS => (z_y, l.hz),
            LAYER_CODES => (l.code_y, l.hk),
            _ => (l.z2_y, l.hz),
        };
        if h > 0.0 {
            out.push(Vec2::new(l.dx / h, y / h));
        }
    }
    out
}

/// One label: `at` is its anchor, left-aligned (or right-aligned when `right`).
#[allow(clippy::too_many_arguments)]
fn label_text(at: Vec2, height: f64, value: String, style: &str, p: &Pismo, right: bool) -> Text {
    Text {
        insert: Vec3::new(at.x, at.y, 0.0),
        align_pt: right.then(|| Vec3::new(at.x, at.y, 0.0)),
        height,
        value,
        rotation: 0.0,
        width_factor: p.width,
        oblique: p.slant.to_radians(),
        style: style.into(),
        halign: if right { HAlign::Right } else { HAlign::Left },
        valign: VAlign::Baseline,
    }
}

/// Define GEO_BOD: attribute definitions only (the point stays a POINT of its own).
fn ensure_point_block(s: &mut Session, n: &Nastaveni) -> Result<()> {
    let d = s.doc_mut()?;
    if d.blocks.contains_key(POINT_BLOCK) {
        return Ok(());
    }
    let mut blk = Block::new(POINT_BLOCK);
    let rows = [
        ("CISLO", "Point number", nast::STYLE_NUMBERS, &n.numbers, 0.25),
        ("VYSKA", "Height", nast::STYLE_HEIGHTS, &n.heights, -1.25),
        ("KOD", "Code", nast::STYLE_CODES, &n.codes, -2.75),
    ];
    for (tag, prompt, style, p, dy) in rows {
        let h = d.new_handle();
        let mut e = Entity::new(
            h,
            EntityKind::AttDef(Attrib {
                tag: tag.into(),
                text: label_text(Vec2::new(0.6, dy), 1.0, String::new(), style, p, false),
                invisible: false,
                constant: false,
                prompt: prompt.into(),
                props: Default::default(),
                multiline: None,
                modes: Default::default(),
            }),
        );
        e.common.layer = "0".into();
        blk.entities.push(e);
    }
    d.blocks.insert(POINT_BLOCK.into(), Arc::new(blk));
    Ok(())
}

pub(super) fn run_geobody(s: &mut Session, p: &Value) -> Result<Value> {
    let (text, file) = if let Some(t) = str_param(p, "text") {
        (t.to_string(), "text".to_string())
    } else if let Some(path) = str_param(p, "path") {
        (decode(&read_path(path)?), path.to_string())
    } else {
        let data = str_param(p, "data").ok_or_else(|| bad(CMD, "`path`, `data` (base64) or `text` is required"))?;
        (decode(&base64_decode(data).ok_or_else(|| bad(CMD, "invalid base64"))?), "data".to_string())
    };
    let spec = str_param(p, "columns").unwrap_or("PYXZK");
    let (mut pts, mut skipped) = parse(&text, spec).map_err(|e| bad(CMD, e))?;
    // The drawing's settings with this call's own on top; they become the drawing's.
    let mut n = nast::load(s);
    nast::apply(&mut n, p).map_err(|e| match e {
        EngineError::BadParams { msg, .. } => bad(CMD, msg),
        other => other,
    })?;
    if n.ku {
        for b in &mut pts {
            b.cislo = ku_number(&b.cislo);
        }
    }
    if let Some(only) = str_param(p, "only") {
        let sel = parse_only(only).map_err(|e| bad(CMD, e))?;
        let before = pts.len();
        pts.retain(|b| selected(&sel, &b.cislo));
        skipped += before - pts.len();
    }
    if pts.is_empty() {
        return Err(bad(CMD, format!("no points found (columns \"{spec}\"; {skipped} lines skipped)")));
    }
    let sjtsk = match p.get("sjtsk") {
        None => looks_sjtsk(&pts),
        Some(Value::Bool(b)) => *b,
        Some(Value::String(t)) if t.eq_ignore_ascii_case("auto") => looks_sjtsk(&pts),
        Some(_) => return Err(bad(CMD, "`sjtsk` must be true, false or \"auto\"")),
    };
    let space = s.space();
    let taken: BTreeSet<String> = {
        let d = s.doc()?;
        match d.space(&space) {
            Some(st) => {
                let existing: Vec<Vec3> = st
                    .iter()
                    .filter(|e| e.common.layer.eq_ignore_ascii_case(LAYER_POINTS))
                    .filter_map(|e| if let EntityKind::Point(q) = &e.kind { Some(q.p) } else { None })
                    .collect();
                point_labels(st, &existing, LAYER_NUMBERS, "CISLO", &label_slots(&n, LAYER_NUMBERS)).into_iter().flatten().collect()
            }
            None => BTreeSet::new(),
        }
    };
    let (pts, duplicates) = resolve_duplicates(pts, n.duplicates, &taken);
    nast::save(s, &n)?;

    {
        let d = s.doc_mut()?;
        for (name, aci) in [(LAYER_POINTS, 7), (LAYER_NUMBERS, 3), (LAYER_HEIGHTS, 1), (LAYER_CODES, 5), (LAYER_HEIGHTS2, 6)] {
            if (name != LAYER_HEIGHTS2 || pts.iter().any(|b| b.z2.is_some()))
                && d.ensure_layer(name)
                && let Some(l) = d.layer_mut(name)
            {
                l.color = cadcraft_color::Color::Index(aci);
            }
        }
    }
    if n.block {
        ensure_point_block(s, &n)?;
    }
    let l = Layout::of(&n, n.corner);
    let (hn, hz, hk, left, dx) = (l.hn, l.hz, l.hk, n.corner.left(), l.dx);
    let (upper, lower, code_y, z2_y) = (l.upper, l.lower, l.code_y, l.z2_y);
    let mut handles = Vec::new();
    let (mut lo, mut hi) = (Vec2::new(f64::INFINITY, f64::INFINITY), Vec2::new(f64::NEG_INFINITY, f64::NEG_INFINITY));
    for b in &pts {
        let at = if sjtsk { Vec2::new(-b.y, -b.x) } else { Vec2::new(b.y, b.x) };
        lo = Vec2::new(lo.x.min(at.x), lo.y.min(at.y));
        hi = Vec2::new(hi.x.max(at.x), hi.y.max(at.y));
        let z = if n.z3d { b.z.unwrap_or(0.0) } else { 0.0 };
        let common = Common { layer: LAYER_POINTS.into(), ..s.current_common() };
        let pt = Point { p: Vec3::new(at.x, at.y, z), angle: 0.0 };
        let hd = s.doc_mut()?.add(&space, common, EntityKind::Point(pt)).map_err(|e| bad(CMD, e.to_string()))?;
        handles.push(hd);
        let (num_y, z_y) = if n.corner.number_below() { (lower, upper) } else { (upper, lower) };
        let mut labels: Vec<(&str, &str, &str, Text)> = Vec::new();
        if n.show_numbers && !b.cislo.is_empty() {
            labels.push((
                LAYER_NUMBERS,
                "CISLO",
                nast::STYLE_NUMBERS,
                label_text(at + Vec2::new(dx, num_y), hn, b.cislo.clone(), nast::STYLE_NUMBERS, &n.numbers, left),
            ));
        }
        if n.show_heights
            && let Some(zv) = b.z
        {
            labels.push((
                LAYER_HEIGHTS,
                "VYSKA",
                nast::STYLE_HEIGHTS,
                label_text(at + Vec2::new(dx, z_y), hz, n.height_text(zv, &b.kod), nast::STYLE_HEIGHTS, &n.heights, left),
            ));
        }
        if n.show_codes && !b.kod.is_empty() {
            labels.push((
                LAYER_CODES,
                "KOD",
                nast::STYLE_CODES,
                label_text(at + Vec2::new(dx, code_y), hk, b.kod.clone(), nast::STYLE_CODES, &n.codes, left),
            ));
        }
        if n.show_heights
            && let Some(z2) = b.z2
        {
            labels.push((
                LAYER_HEIGHTS2,
                "VYSKA2",
                nast::STYLE_HEIGHTS,
                label_text(at + Vec2::new(dx, z2_y), hz, n.height_text(z2, &b.kod), nast::STYLE_HEIGHTS, &n.heights, left),
            ));
        }
        if n.block {
            // Z2 stays a text of its own: the block has no attribute for it.
            let (attribs, rest): (Vec<_>, Vec<_>) = labels.into_iter().partition(|l| l.1 != "VYSKA2");
            let attribs: Vec<Attrib> = attribs
                .into_iter()
                .map(|(_, tag, _, text)| Attrib {
                    tag: tag.into(),
                    text,
                    invisible: false,
                    constant: false,
                    prompt: String::new(),
                    props: Default::default(),
                    multiline: None,
                    modes: Default::default(),
                })
                .collect();
            let ins = Insert {
                block: POINT_BLOCK.into(),
                insert: Vec3::new(at.x, at.y, 0.0),
                scale: Vec3::new(1.0, 1.0, 1.0),
                rotation: 0.0,
                attribs,
                cols: 1,
                rows: 1,
                col_spacing: 0.0,
                row_spacing: 0.0,
                clip: None,
            };
            let common = Common { layer: LAYER_NUMBERS.into(), ..s.current_common() };
            s.doc_mut()?.add(&space, common, EntityKind::Insert(ins)).map_err(|e| bad(CMD, e.to_string()))?;
            labels = rest;
        }
        for (layer, _, _, text) in labels {
            let common = Common { layer: layer.into(), ..s.current_common() };
            s.doc_mut()?.add(&space, common, EntityKind::Text(text)).map_err(|e| bad(CMD, e.to_string()))?;
        }
    }
    let mut symbols = String::new();
    if n.symbols {
        let hs: Vec<String> = handles.iter().map(|h: &Handle| h.hex()).collect();
        match s.execute("geokresba", &json!({ "handles": hs, "scale": n.scale })) {
            Ok(r) => {
                if let Some(m) = r.get("message").and_then(Value::as_str) {
                    symbols = format!("; {m}");
                }
            }
            Err(e) => symbols = format!("; map symbols not drawn: {e}"),
        }
    }
    s.set_selection(handles.clone());
    let name = std::path::Path::new(&file).file_name().map(|n| n.to_string_lossy().to_string()).unwrap_or_default();
    let dup_note = match (duplicates, n.duplicates) {
        (0, _) => String::new(),
        (k, Duplicates::Keep) => format!("; {k} numbers already taken (kept)"),
        (k, Duplicates::Skip) => format!("; {k} points with a taken number skipped"),
        (k, Duplicates::Renumber) => format!("; {k} points renumbered"),
        (k, Duplicates::Replace) => format!("; {k} repeated numbers replaced by the later point"),
        (k, Duplicates::Average) => format!("; {k} repeated numbers averaged"),
    };
    Ok(json!({
        "imported": handles.len(),
        "skipped": skipped,
        "duplicates": duplicates,
        "sjtsk": sjtsk,
        "extents": [[lo.x, lo.y], [hi.x, hi.y]],
        "message": format!(
            "{} points imported from {name}{}{}{dup_note}{symbols}",
            handles.len(),
            if sjtsk { " (S-JTSK: x = -Y, y = -X)" } else { "" },
            if skipped > 0 { format!("; {skipped} lines without a point skipped") } else { String::new() }
        ),
    }))
}

#[derive(Default)]
struct BodyM {
    path: Option<String>,
    columns: Option<String>,
}

impl Interactive for BodyM {
    fn name(&self) -> &'static str {
        "GEOBODY"
    }

    fn prompt(&self, s: &Session) -> Prompt {
        match (&self.path, &self.columns) {
            (None, _) => Prompt::new("Enter name of coordinate list to import", Accept::TEXT),
            (Some(_), None) => Prompt::new("Columns (P number, Y, X, Z, Z2, K code, - skip)", Accept::TEXT).default("PYXZK"),
            // Labels are 2 mm on paper at this scale.
            _ => Prompt::new("Drawing scale 1: (sets the label size)", Accept::NUMBER).default(format!("{}", nast::scale(s))),
        }
    }

    fn input(&mut self, s: &mut Session, i: Input) -> Result<Step> {
        let t = match &i {
            Input::Cancel => return Ok(Step::Cancel),
            Input::Text(t) | Input::Keyword(t) => t.trim().trim_matches('"').to_string(),
            Input::Enter => String::new(),
            _ => return Ok(Step::Continue),
        };
        let Some(path) = self.path.clone() else {
            if !t.is_empty() {
                self.path = Some(t);
            }
            return Ok(Step::Continue);
        };
        let Some(spec) = self.columns.clone() else {
            let spec = if t.is_empty() { "PYXZK".to_string() } else { t };
            match columns(&spec) {
                Ok(_) => self.columns = Some(spec),
                Err(e) => s.echo(e),
            }
            return Ok(Step::Continue);
        };
        let scale = if t.is_empty() {
            nast::scale(s)
        } else {
            // "1:200", "1 : 200" and "200" all mean 1:200.
            let n = t.rsplit(':').next().unwrap_or(&t).trim().replace(',', ".");
            match n.parse::<f64>().ok().filter(|v| v.is_finite() && (1.0..=1_000_000.0).contains(v)) {
                Some(v) => v,
                None => {
                    s.echo("Enter the drawing scale, like 200, 500 or 1000.");
                    return Ok(Step::Continue);
                }
            }
        };
        match run_geobody(s, &json!({ "path": path, "columns": spec, "scale": scale })) {
            Ok(r) => {
                if let Some(m) = r.get("message").and_then(Value::as_str) {
                    s.echo(m.to_string());
                }
                // Show what came in.
                let _ = s.execute("zoom", &json!({ "mode": "extents" }));
            }
            Err(e) => s.echo(match e {
                EngineError::BadParams { msg, .. } => msg,
                other => other.to_string(),
            }),
        }
        Ok(Step::Done)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_icon_asks_for_the_scale() {
        let dir = std::env::temp_dir().join(format!("geobody-scale-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("body.txt");
        std::fs::write(&path, "1 10 20 300\n2 30 40 301\n").unwrap();
        let label_height = |s: &Session| {
            s.doc().unwrap().model.iter().find_map(|e| match &e.kind {
                EntityKind::Text(t) if e.common.layer == LAYER_NUMBERS => Some(t.height),
                _ => None,
            })
        };
        // File, Enter for the columns, then 1:200: labels 0.4 m.
        let mut s = Session::new();
        s.start("geobody").unwrap();
        s.input(Input::Text(path.to_string_lossy().into_owned())).unwrap();
        s.input(Input::Enter).unwrap();
        assert!(s.doc().unwrap().model.iter().next().is_none(), "nothing before the scale is given");
        s.input(Input::Text("1:200".into())).unwrap();
        assert!((label_height(&s).unwrap() - 0.4).abs() < 1e-12);
        // Enter keeps 1:500 (1 m); a wrong answer asks again.
        let mut s = Session::new();
        s.start("geobody").unwrap();
        s.input(Input::Text(path.to_string_lossy().into_owned())).unwrap();
        s.input(Input::Text("PYXZ".into())).unwrap();
        s.input(Input::Text("abc".into())).unwrap();
        assert!(label_height(&s).is_none());
        s.input(Input::Enter).unwrap();
        assert!((label_height(&s).unwrap() - 1.0).abs() < 1e-12);
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn lists_in_the_usual_shapes() {
        let (p, skipped) =
            parse("cislo Y X Z kod\n4001 742123.45 1043210.10 312.55 plot\n4002\t742124,00\t1043211,50\t312,60\tzed dum\n", "PYXZK").unwrap();
        assert_eq!(skipped, 1);
        assert_eq!(p.len(), 2);
        assert_eq!(p[0], Bod { cislo: "4001".into(), y: 742123.45, x: 1043210.10, z: Some(312.55), kod: "plot".into(), z2: None });
        assert_eq!((p[1].y, p[1].x, p[1].z), (742124.0, 1043211.5, Some(312.6)));
        assert_eq!(p[1].kod, "zed dum");
        let (p, _) = parse("1;742123,4;1043210,1;312,5\n2,742124.0,1043211.5,312.6\n3 742125 1043212\n", "PYXZ").unwrap();
        assert_eq!(p.len(), 3);
        assert_eq!(p[2].z, None);
        assert!(looks_sjtsk(&p));
        let (p, _) = parse("1 10 20 5\n", "PXYZ").unwrap();
        assert_eq!((p[0].y, p[0].x), (20.0, 10.0));
        assert!(!looks_sjtsk(&p));
    }

    #[test]
    fn bad_column_specs_are_refused() {
        for spec in ["PXZ", "", "PYXQ", "YYYYYYYYYYYYYYYYX"] {
            assert!(columns(spec).is_err(), "{spec}");
        }
    }

    #[test]
    fn imports_sjtsk_north_up() {
        let mut s = Session::new();
        let r = s.execute(CMD, &json!({ "text": "1 742100 1043200 300.123 hrana\n2 742000 1043000 301\n" })).unwrap();
        assert_eq!(r["imported"], 2);
        assert_eq!(r["sjtsk"], true);
        let d = s.doc().unwrap();
        let pts: Vec<_> = d.model.iter().filter_map(|e| if let EntityKind::Point(p) = &e.kind { Some(p.p) } else { None }).collect();
        assert_eq!(pts[0], Vec3::new(-742100.0, -1043200.0, 300.123));
        let texts: Vec<_> = d
            .model
            .iter()
            .filter_map(|e| if let EntityKind::Text(t) = &e.kind { Some((e.common.layer.clone(), t.value.clone())) } else { None })
            .collect();
        assert!(texts.contains(&(LAYER_HEIGHTS.to_string(), "300.12".to_string())), "{texts:?}");
        assert!(texts.contains(&(LAYER_CODES.to_string(), "hrana".to_string())), "{texts:?}");
    }

    #[test]
    fn hostile_input_is_refused_not_panicking() {
        let mut s = Session::new();
        for p in [
            json!({}),
            json!({ "text": "" }),
            json!({ "text": "a b c d" }),
            json!({ "text": "1 NaN inf 3" }),
            json!({ "text": "1 1e300 2 3" }),
            json!({ "text": "1 2 3", "columns": "PQ" }),
            json!({ "text": "1 2 3", "scale": -5 }),
            json!({ "text": "1 2 3", "sjtsk": 7 }),
        ] {
            assert!(s.execute(CMD, &p).is_err(), "{p}");
        }
    }

    fn texts_on(s: &Session, layer: &str) -> Vec<Text> {
        s.doc()
            .unwrap()
            .model
            .iter()
            .filter(|e| e.common.layer == layer)
            .filter_map(|e| if let EntityKind::Text(t) = &e.kind { Some(t.clone()) } else { None })
            .collect()
    }

    #[test]
    fn point_loading_options() {
        // Z2, a selection of numbers and 12-digit numbers.
        let list = "001012500002 10 20 300 1 299.5\n001000000007 11 21 301 T\n5 12 22 302\n6 13 23 303\n";
        let mut s = Session::new();
        let r = s.execute(CMD, &json!({ "text": list, "columns": "PYXZKZ2", "ku": true, "only": "1250-2, 7, 5-5" })).unwrap();
        assert_eq!(r["imported"], 3, "{r}");
        let nums: Vec<String> = texts_on(&s, LAYER_NUMBERS).into_iter().map(|t| t.value).collect();
        assert_eq!(nums, ["1250-2", "7", "5"]);
        assert_eq!(texts_on(&s, LAYER_HEIGHTS2).len(), 1);
        assert_eq!(texts_on(&s, LAYER_HEIGHTS2)[0].value, "299.50");
        // The settings stay with the drawing.
        assert!(nast::load(&s).ku);
        assert_eq!(ku_number("001000000007"), "7");
        assert_eq!(ku_number("123"), "123");
    }

    #[test]
    fn duplicates_follow_the_rule() {
        let list = "1 10 20 300\n1 12 22 302\n2 30 40 310\n";
        let count = |rule: &str| {
            let mut s = Session::new();
            let r = s.execute(CMD, &json!({ "text": list, "duplicates": rule })).unwrap();
            (r["imported"].as_u64().unwrap(), texts_on(&s, LAYER_NUMBERS).into_iter().map(|t| t.value).collect::<Vec<_>>(), s)
        };
        assert_eq!(count("keep").0, 3);
        assert_eq!(count("skip").0, 2);
        assert_eq!(count("renumber").1, ["1", "3", "2"]);
        let (n, _, s) = count("average");
        assert_eq!(n, 2);
        let z: Vec<String> = texts_on(&s, LAYER_HEIGHTS).into_iter().map(|t| t.value).collect();
        assert_eq!(z, ["301.00", "310.00"]);
        let (_, _, s) = count("replace");
        assert_eq!(texts_on(&s, LAYER_HEIGHTS)[0].value, "302.00");
        // Against points already in the drawing.
        let mut s = Session::new();
        s.execute(CMD, &json!({ "text": "1 0 0 1\n" })).unwrap();
        let r = s.execute(CMD, &json!({ "text": "1 5 5 2\n", "duplicates": "skip" }));
        assert!(r.is_err() || r.unwrap()["imported"] == 0);
    }

    #[test]
    fn labels_follow_the_settings() {
        let mut s = Session::new();
        s.execute(CMD, &json!({ "text": "7 10 20 245.374 X1\n", "corner": "LD", "digits": "tens", "scale": 1000, "fonts": { "numbers": { "mm": 3, "font": "Arial" } } })).unwrap();
        let num = &texts_on(&s, LAYER_NUMBERS)[0];
        assert_eq!(num.halign, HAlign::Right);
        assert!(num.insert.x < 10.0 && num.insert.y < 20.0, "left and below: {:?}", num.insert);
        assert!((num.height - 3.0).abs() < 1e-9);
        assert_eq!(num.style, nast::STYLE_NUMBERS);
        assert_eq!(texts_on(&s, LAYER_HEIGHTS)[0].value, "45.37");
        // Other survey commands still find the number and the code.
        let space = s.space();
        let found = super::super::geo_pohled::numbered_points(&s, &space, None).unwrap();
        assert_eq!(found[0].0, "7");
    }

    #[test]
    fn points_as_blocks_keep_their_numbers() {
        let mut s = Session::new();
        s.execute(CMD, &json!({ "text": "7 10 20 245 X1\n8 30 40 246\n", "block": true })).unwrap();
        let d = s.doc().unwrap();
        assert!(d.blocks.contains_key(POINT_BLOCK));
        let inserts = d.model.iter().filter(|e| matches!(e.kind, EntityKind::Insert(_))).count();
        assert_eq!(inserts, 2);
        assert!(texts_on(&s, LAYER_NUMBERS).is_empty());
        let space = s.space();
        let found = super::super::geo_pohled::numbered_points(&s, &space, None).unwrap();
        let mut nums: Vec<String> = found.into_iter().map(|f| f.0).collect();
        nums.sort();
        assert_eq!(nums, ["7", "8"]);
    }

    #[test]
    fn hostile_options_are_refused_not_panicking() {
        let mut s = Session::new();
        for p in [
            json!({ "text": "1 2 3", "only": "" }),
            json!({ "text": "1 2 3", "only": ",,," }),
            json!({ "text": "1 2 3", "duplicates": "boom" }),
            json!({ "text": "1 2 3", "corner": 5 }),
            json!({ "text": "1 2 3", "columns": "PYXZ2Z2Z2Z2Z2Z2Z2Z2Z2Z2Z2Z2Z2Z2" }),
        ] {
            let _ = s.execute(CMD, &p);
        }
        let big = (0..2000).map(|i| i.to_string()).collect::<Vec<_>>().join(",");
        assert!(s.execute(CMD, &json!({ "text": "1 2 3", "only": big })).is_err());
        assert!(s.execute(CMD, &json!({ "text": "18446744073709551615 2 3\n18446744073709551615 2 4\n", "duplicates": "renumber" })).is_ok());
    }
}
