//! Survey add-on: cadastral maps in the older text formats VKM (Czech DKM / KM-D) and VGI
//! (Slovak cadastre), GEOVKM.
//!
//! VKM after ČÚZK's description of the old exchange format (part 2, section 4.4): logical
//! records start with `&` on a new line — `&V` header (file name, reduction constants Yo Xo),
//! `&R` extent and scale factor (with `R`, coordinates are reduced by Yo, Xo), `&D`
//! identification, `&U nn` layer, `&L` line element, `&T` text element, `&G` geometric plan,
//! `&S` coordinate list, `&*` comment, `&K` end. A line element is a list of points, one per
//! line, each starting with its connection to the previous point: `P` none (a new start), `L`
//! straight, `R` circular arc through three consecutive points (in pairs), `K` circle by centre
//! and `R=` radius, `C` interpolated curve. Coordinates are Y X in S-JTSK; point numbers are
//! `B=` (group) and `C=` (own); `S=` a point symbol; texts are quoted with `D=` alignment, `H=`
//! height in mm, `U=` rotation in gon.
//!
//! VGI uses the same element syntax, with `&O layer number` object records naming the layers and
//! `&A` attributes; its connection codes are taken to be those of VKM.
//!
//! Drawing coordinates are S-JTSK's x = −Y, y = −X. Each layer goes on its own KN_ layer.

use std::collections::BTreeMap;

use cadcraft_doc::{Circle, EntityKind, HAlign, LwPolyline, Point, VAlign};
use cadcraft_geom::{PolyVertex, Vec2, Vec3};
use serde_json::{Value, json};

use super::geo_calc::{add, ensure_layer, say, text};
use super::geo_vfk::bulge;
use super::{CommandSpec, bad, f64_or, str_param};
use crate::{Accept, Input, Interactive, Prompt, Result, Session, Step};

/// Lines read from one file.
const MAX_LINES: usize = 20_000_000;
/// Points of one element.
const MAX_ELEMENT: usize = 1_000_000;

pub(super) fn specs() -> Vec<CommandSpec> {
    vec![
        CommandSpec::new("geovkm", "Import Cadastral Map (VKM, VGI)...", run_geovkm)
            .menu(&["Geo", "Cadastre", "Import Cadastral Map (VKM, VGI)..."])
            .alias(&["geovgi", "-geovkm"])
            .params("{path | text: a VKM or VGI file, points?: bool, draw numbered points with their numbers (default false), scale?: map scale 1:N for texts (default: the file's)} → {format, lines, arcs, circles, texts, symbols, points, layers, message}")
            .interactive(|_| Ok(Box::new(FileM))),
    ]
}

/// Czech names of the VKM layers (ČÚZK table I).
fn vkm_layer(n: &str) -> String {
    let name = match n.trim_start_matches('0') {
        "1" => "hranice_parcel",
        "2" => "parcelni_cisla",
        "3" => "znacky_druhu_pozemku",
        "4" => "vnitrni_kresba",
        "5" => "znacky_budov",
        "6" => "ostatni_prvky",
        "7" => "popis",
        "8" => "body",
        "10" => "ramy_listu",
        "11" => "BPEJ",
        _ => "",
    };
    let n: String = n.chars().filter(char::is_ascii_alphanumeric).take(8).collect();
    if name.is_empty() { format!("KN_VKM_{n}") } else { format!("KN_VKM_{n:0>2}_{name}") }
}

/// Decode a VKM/VGI file: UTF-8 if it is, else DOS 852 ("Latin 2 PC", VKM's code page) or
/// Windows-1250, whichever reads more like Czech or Slovak.
pub fn decode_latin2(bytes: &[u8]) -> String {
    let bytes = bytes.strip_prefix(b"\xEF\xBB\xBF").unwrap_or(bytes);
    if let Ok(s) = std::str::from_utf8(bytes) {
        return s.to_string();
    }
    // Lower-case accented letters count for a reading; capitals right after a lower-case letter
    // (a word read in the wrong code page) and box-drawing characters count against it.
    let score = |s: &str| {
        let mut n: i64 = 0;
        let mut prev_lower = false;
        for c in s.chars() {
            if "áčďéěíňóřšťúůýžľĺŕôä".contains(c) {
                n += 1;
            } else if ("ÁČĎÉĚÍŇÓŘŠŤÚŮÝŽĽĹŔÔÄ".contains(c) && prev_lower) || ('\u{2500}'..='\u{259F}').contains(&c) {
                n -= 1;
            }
            prev_lower = c.is_lowercase();
        }
        n
    };
    let a = craft_dxf::decode_code_page(bytes, "DOS852");
    let b = craft_dxf::decode_code_page(bytes, "ANSI_1250");
    match (a, b) {
        (Some(a), Some(b)) => {
            if score(&b) > score(&a) {
                b
            } else {
                a
            }
        }
        (Some(a), None) => a,
        (None, Some(b)) => b,
        (None, None) => String::from_utf8_lossy(bytes).into_owned(),
    }
}

/// `KEY=value` parameters after the positional fields.
fn params<'a>(fields: impl Iterator<Item = &'a str>) -> BTreeMap<String, String> {
    fields.filter_map(|f| f.split_once('=')).map(|(k, v)| (k.to_ascii_uppercase(), v.to_string())).collect()
}

fn num(t: &str) -> Option<f64> {
    t.trim().replace(',', ".").parse::<f64>().ok().filter(|v| v.is_finite() && v.abs() < 1e9)
}

/// One point of a line element.
#[derive(Clone, Debug)]
pub struct Pt {
    pub conn: char,
    pub at: Vec2,
    pub number: Option<String>,
    pub radius: Option<f64>,
    pub symbol: Option<String>,
}

/// A parsed file.
#[derive(Debug, Default)]
pub struct Map {
    pub vgi: bool,
    pub scale: f64,
    /// (layer, points) per line element.
    pub elements: Vec<(String, Vec<Pt>)>,
    /// (layer, position, text, height in mm, rotation in gon, alignment 1–9).
    pub texts: Vec<(String, Vec2, String, f64, f64, u8)>,
    /// (number, position, height) from &S coordinate lists.
    pub list: Vec<(String, Vec2, f64)>,
}

/// Split a text record's quoted text out of its line: the text sits between the first quote
/// mark after the coordinates and the next one.
fn quoted(rest: &str) -> Option<(String, &str)> {
    const QUOTES: &[char] = &['\'', '"', '‚', '‘', '’', '„', '“', '”', '`', '´'];
    let start = rest.find(QUOTES)?;
    let open = rest.get(start..)?.chars().next()?;
    let body = rest.get(start + open.len_utf8()..)?;
    let end = body.find(QUOTES)?;
    let close = body.get(end..)?.chars().next()?;
    Some((body.get(..end)?.to_string(), body.get(end + close.len_utf8()..)?))
}

pub fn parse(text: &str) -> std::result::Result<Map, String> {
    let mut m = Map { scale: 1000.0, ..Map::default() };
    let (mut yo, mut xo, mut reduced) = (0.0, 0.0, false);
    let mut layer = String::from("KN_VKM");
    let mut plan: Option<String> = None;
    let mut current: Option<(String, Vec<Pt>)> = None;
    let mut in_list = false;
    let mut header_seen = false;
    let mut group: Option<String> = None;
    let finish = |cur: &mut Option<(String, Vec<Pt>)>, out: &mut Vec<(String, Vec<Pt>)>| {
        if let Some(e) = cur.take().filter(|e| !e.1.is_empty()) {
            out.push(e);
        }
    };
    for (i, raw) in text.lines().enumerate() {
        if i >= MAX_LINES {
            return Err(format!("more than {MAX_LINES} lines"));
        }
        let line = raw.trim();
        if line.is_empty() {
            continue;
        }
        let mut f = line.split_whitespace();
        let Some(head) = f.next() else { continue };
        let layer_now = |l: &str| match &plan {
            Some(g) => format!("{l}_GP{g}"),
            None => l.to_string(),
        };
        // A point of a line element (first or continuation).
        let point = |conn: char, f: &mut std::str::SplitWhitespace<'_>, group: &mut Option<String>| -> Option<Pt> {
            let (y, x) = (num(f.next()?)?, num(f.next()?)?);
            let p = params(f);
            if let Some(b) = p.get("B") {
                *group = Some(b.clone());
            }
            let number = p.get("C").map(|c| match group {
                Some(b) => format!("{b}{c:0>4}"),
                None => c.clone(),
            });
            let (y, x) = if reduced { (y + yo, x + xo) } else { (y, x) };
            Some(Pt {
                conn,
                at: Vec2::new(-y, -x),
                number,
                radius: p.get("R").and_then(|r| num(r)).filter(|r| *r > 0.0),
                symbol: p.get("S").cloned(),
            })
        };
        if let Some(rec) = head.strip_prefix('&') {
            let kind = rec.chars().next().map(|c| c.to_ascii_uppercase());
            if kind != Some('L') && kind != Some('*') {
                finish(&mut current, &mut m.elements);
            }
            in_list = false;
            match kind {
                Some('V') => {
                    header_seen = true;
                    // VGI headers read "&V name YX ..."; VKM "&V name Yo Xo ...".
                    let rest: Vec<&str> = f.collect();
                    if rest.get(1).is_some_and(|t| t.eq_ignore_ascii_case("YX")) {
                        m.vgi = true;
                    } else if let (Some(a), Some(b)) = (rest.get(1).and_then(|t| num(t)), rest.get(2).and_then(|t| num(t))) {
                        (yo, xo) = (a, b);
                    }
                }
                Some('R') => {
                    let rest: Vec<&str> = f.collect();
                    if let Some(s) = rest.get(4).and_then(|t| num(t)).filter(|s| (1.0..=1e6).contains(s)) {
                        m.scale = s;
                    }
                    reduced = rest.get(5).is_some_and(|t| t.eq_ignore_ascii_case("R"));
                }
                Some('B') => {
                    // VGI map parameters: MIERKA= is the scale.
                    if let Some(s) = params(f).get("MIERKA").and_then(|s| num(s)).filter(|s| (1.0..=1e6).contains(s)) {
                        m.scale = s;
                    }
                }
                Some('U') => {
                    if let Some(n) = f.next() {
                        layer = vkm_layer(n);
                    }
                }
                Some('O') => {
                    m.vgi = true;
                    if let Some(n) = f.next() {
                        let n: String = n.chars().filter(|c| c.is_alphanumeric() || *c == '_').take(32).collect();
                        layer = format!("KN_VGI_{n}");
                    }
                }
                Some('G') => {
                    let g = params(f).get("G").cloned().unwrap_or_default();
                    let g: String = g.chars().filter(char::is_ascii_alphanumeric).take(8).collect();
                    plan = Some(g);
                }
                Some('S') => in_list = true,
                Some('L') => {
                    finish(&mut current, &mut m.elements);
                    let conn = f.next().and_then(|c| c.chars().next()).map(|c| c.to_ascii_uppercase()).unwrap_or('P');
                    if let Some(p) = point(conn, &mut f, &mut group) {
                        current = Some((layer_now(&layer), vec![p]));
                    }
                }
                Some('T') => {
                    let (Some(y), Some(x)) = (f.next().and_then(num), f.next().and_then(num)) else { continue };
                    let after = line.splitn(4, char::is_whitespace).nth(3).unwrap_or("");
                    let Some((t, rest)) = quoted(after) else { continue };
                    let p = params(rest.split_whitespace());
                    let h = p.get("H").and_then(|v| num(v)).filter(|h| *h > 0.0 && *h < 100.0).unwrap_or(1.7);
                    let u = p.get("U").and_then(|v| num(v)).unwrap_or(0.0);
                    let d = p.get("D").and_then(|v| v.parse::<u8>().ok()).filter(|d| (1..=9).contains(d)).unwrap_or(2);
                    let (y, x) = if reduced { (y + yo, x + xo) } else { (y, x) };
                    if m.texts.len() < MAX_ELEMENT {
                        m.texts.push((layer_now(&layer), Vec2::new(-y, -x), t.chars().take(200).collect(), h, u, d));
                    }
                }
                Some('K') => break,
                _ => {}
            }
            continue;
        }
        if in_list {
            // &S lines: number Y X [height] [quality].
            let (Some(n), Some(y), Some(x)) = (Some(head), f.next().and_then(num), f.next().and_then(num)) else { continue };
            let z = f.next().and_then(num).unwrap_or(0.0);
            let (y, x) = if reduced { (y + yo, x + xo) } else { (y, x) };
            if m.list.len() < MAX_ELEMENT {
                m.list.push((n.chars().take(24).collect(), Vec2::new(-y, -x), z));
            }
            continue;
        }
        // A continuation point: "L y x ...", "R y x ...", "P y x ...".
        let conn = head.chars().next().map(|c| c.to_ascii_uppercase()).unwrap_or(' ');
        if head.chars().count() == 1
            && "PLRKC".contains(conn)
            && let Some((_, pts)) = current.as_mut()
            && pts.len() < MAX_ELEMENT
            && let Some(p) = point(conn, &mut f, &mut group)
        {
            pts.push(p);
        }
    }
    finish(&mut current, &mut m.elements);
    if !header_seen && m.elements.is_empty() && m.texts.is_empty() {
        return Err("no &V header or map elements: is this a VKM or VGI file?".into());
    }
    Ok(m)
}

/// Sample a Catmull-Rom curve through `pts` (an interpolated curve, connection `C`).
fn curve(pts: &[Vec2]) -> Vec<Vec2> {
    let n = pts.len();
    let mut out = Vec::new();
    for i in 0..n.saturating_sub(1) {
        let (Some(&p1), Some(&p2)) = (pts.get(i), pts.get(i + 1)) else { continue };
        let p0 = if i == 0 { p1 } else { pts.get(i - 1).copied().unwrap_or(p1) };
        let p3 = pts.get(i + 2).copied().unwrap_or(p2);
        for k in 0..8 {
            let t = k as f64 / 8.0;
            let (t2, t3) = (t * t, t * t * t);
            let f = |a: f64, b: f64, c: f64, d: f64| {
                0.5 * (2.0 * b + (-a + c) * t + (2.0 * a - 5.0 * b + 4.0 * c - d) * t2 + (-a + 3.0 * b - 3.0 * c + d) * t3)
            };
            out.push(Vec2::new(f(p0.x, p1.x, p2.x, p3.x), f(p0.y, p1.y, p2.y, p3.y)));
        }
    }
    if let Some(l) = pts.last() {
        out.push(*l);
    }
    out
}

/// What one element draws: polylines (with arcs as bulges), circles, and isolated symbol points.
pub enum Shape {
    Poly(Vec<PolyVertex>),
    Circle(Vec2, f64),
    Symbol(Vec2, String),
}

pub fn shapes(pts: &[Pt]) -> Vec<Shape> {
    let mut out = Vec::new();
    let mut run: Vec<PolyVertex> = Vec::new();
    let flush = |run: &mut Vec<PolyVertex>, out: &mut Vec<Shape>| {
        if run.len() >= 2 {
            out.push(Shape::Poly(std::mem::take(run)));
        } else {
            run.clear();
        }
    };
    let mut i = 0;
    while let Some(p) = pts.get(i) {
        match p.conn {
            'K' => {
                flush(&mut run, &mut out);
                if let Some(r) = p.radius {
                    out.push(Shape::Circle(p.at, r));
                }
                i += 1;
            }
            'R' if !run.is_empty() => match pts.get(i + 1).filter(|q| q.conn == 'R') {
                Some(end) => {
                    let start = run.last().map(|v| v.p).unwrap_or(p.at);
                    if let (Some(b), Some(last)) = (bulge(start, p.at, end.at), run.last_mut()) {
                        last.bulge = b;
                    }
                    run.push(PolyVertex::new(end.at));
                    i += 2;
                }
                None => {
                    run.push(PolyVertex::new(p.at));
                    i += 1;
                }
            },
            'C' if !run.is_empty() => {
                // The curve runs through the last point and every C point that follows.
                let mut through: Vec<Vec2> = run.last().map(|v| vec![v.p]).unwrap_or_default();
                while let Some(q) = pts.get(i).filter(|q| q.conn == 'C') {
                    through.push(q.at);
                    i += 1;
                }
                run.pop();
                run.extend(curve(&through).into_iter().map(PolyVertex::new));
            }
            'P' => {
                flush(&mut run, &mut out);
                // An isolated point with a symbol: a symbol, not the start of a line.
                let next_joins = pts.get(i + 1).is_some_and(|q| q.conn != 'P' && q.conn != 'K');
                if !next_joins && let Some(s) = &p.symbol {
                    out.push(Shape::Symbol(p.at, s.clone()));
                }
                run.push(PolyVertex::new(p.at));
                i += 1;
            }
            _ => {
                run.push(PolyVertex::new(p.at));
                i += 1;
            }
        }
    }
    flush(&mut run, &mut out);
    out
}

fn align(d: u8) -> (HAlign, VAlign) {
    let h = match d {
        1 | 4 | 7 => HAlign::Left,
        3 | 6 | 9 => HAlign::Right,
        _ => HAlign::Center,
    };
    let v = match d {
        1..=3 => VAlign::Bottom,
        4..=6 => VAlign::Top,
        _ => VAlign::Middle,
    };
    (h, v)
}

fn run_geovkm(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "geovkm";
    let source = match (str_param(p, "text"), str_param(p, "path")) {
        (Some(t), _) => t.to_string(),
        (None, Some(path)) => {
            #[cfg(not(target_arch = "wasm32"))]
            {
                crate::file_access::check_read(path).map_err(|e| bad(CMD, e))?;
                let meta = std::fs::metadata(path).map_err(|e| bad(CMD, format!("{path}: {e}")))?;
                if meta.len() > 1024 * 1024 * 1024 {
                    return Err(bad(CMD, format!("{path}: larger than 1 GB")));
                }
                decode_latin2(&std::fs::read(path).map_err(|e| bad(CMD, format!("{path}: {e}")))?)
            }
            #[cfg(target_arch = "wasm32")]
            return Err(bad(CMD, format!("{path}: files can't be read on the web; pass `text`")));
        }
        (None, None) => return Err(bad(CMD, "`path` (the VKM or VGI file) or `text` is required")),
    };
    let m = parse(&source).map_err(|e| bad(CMD, e))?;
    let scale = match p.get("scale") {
        None | Some(Value::Null) => m.scale,
        Some(_) => f64_or(p, "scale", m.scale),
    };
    if !(scale.is_finite() && (1.0..=1e6).contains(&scale)) {
        return Err(bad(CMD, "`scale` must be a map scale like 1000"));
    }
    let space = s.space();
    let mut layers: Vec<String> = Vec::new();
    let mut ensure = |s: &mut Session, l: &str| -> Result<()> {
        if !layers.iter().any(|x| x == l) {
            ensure_layer(s, l, 7)?;
            layers.push(l.to_string());
        }
        Ok(())
    };
    let (mut n_lines, mut n_arcs, mut n_circles, mut n_symbols) = (0, 0, 0, 0);
    let sym_r = 0.0008 * scale;
    for (layer, pts) in &m.elements {
        for shape in shapes(pts) {
            ensure(s, layer)?;
            match shape {
                Shape::Poly(v) => {
                    n_arcs += v.iter().filter(|q| q.bulge != 0.0).count();
                    add(
                        s,
                        &space,
                        layer,
                        EntityKind::LwPolyline(LwPolyline { vertices: v, closed: false, const_width: 0.0, elevation: 0.0, plinegen: false }),
                    )?;
                    n_lines += 1;
                }
                Shape::Circle(c, r) => {
                    add(s, &space, layer, EntityKind::Circle(Circle { center: Vec3::new(c.x, c.y, 0.0), radius: r }))?;
                    n_circles += 1;
                }
                Shape::Symbol(c, code) => {
                    // The symbol's place, a small circle, and its code beside it.
                    add(s, &space, layer, EntityKind::Point(Point { p: Vec3::new(c.x, c.y, 0.0), angle: 0.0 }))?;
                    add(s, &space, layer, EntityKind::Circle(Circle { center: Vec3::new(c.x, c.y, 0.0), radius: sym_r }))?;
                    let code: String = code.chars().take(8).collect();
                    text(s, &space, layer, Vec2::new(c.x + 1.5 * sym_r, c.y), sym_r * 1.5, code, 0.0, (HAlign::Left, VAlign::Middle))?;
                    n_symbols += 1;
                }
            }
        }
    }
    for (layer, at, t, h, u, d) in &m.texts {
        ensure(s, layer)?;
        text(s, &space, layer, *at, h / 1000.0 * scale, t.clone(), u * std::f64::consts::PI / 200.0, align(*d))?;
    }
    let mut n_points = 0;
    if p.get("points").and_then(Value::as_bool).unwrap_or(false) {
        let mut seen = std::collections::HashSet::new();
        let pts = m
            .elements
            .iter()
            .flat_map(|(_, v)| v.iter())
            .filter_map(|q| q.number.as_ref().map(|n| (n.clone(), q.at, 0.0)))
            .chain(m.list.iter().cloned());
        let layer = "KN_body_cisla";
        let h = 0.0012 * scale;
        for (n, at, z) in pts {
            if !seen.insert(((at.x * 100.0).round() as i64, (at.y * 100.0).round() as i64)) {
                continue;
            }
            ensure(s, layer)?;
            add(s, &space, layer, EntityKind::Point(Point { p: Vec3::new(at.x, at.y, z), angle: 0.0 }))?;
            text(s, &space, layer, Vec2::new(at.x + 0.5 * h, at.y + 0.3 * h), h, n, 0.0, (HAlign::Left, VAlign::Baseline))?;
            n_points += 1;
        }
    }
    let format = if m.vgi { "VGI" } else { "VKM" };
    if n_lines + n_circles + n_symbols + m.texts.len() + n_points == 0 {
        return Err(bad(CMD, format!("the {format} file has no map elements")));
    }
    Ok(json!({
        "format": format, "lines": n_lines, "arcs": n_arcs, "circles": n_circles, "texts": m.texts.len(), "symbols": n_symbols, "points": n_points, "layers": layers,
        "message": format!("{format} cadastral map: {n_lines} lines ({n_arcs} arcs), {n_circles} circles, {} texts, {n_symbols} symbols on {} layers", m.texts.len(), layers.len()),
    }))
}

struct FileM;

impl Interactive for FileM {
    fn name(&self) -> &'static str {
        "GEOVKM"
    }
    fn prompt(&self, _s: &Session) -> Prompt {
        Prompt::new("VKM or VGI file", Accept::TEXT)
    }
    fn input(&mut self, s: &mut Session, i: Input) -> Result<Step> {
        match i {
            Input::Cancel | Input::Enter => Ok(Step::Done),
            Input::Text(t) | Input::Keyword(t) => {
                let r = run_geovkm(s, &json!({ "path": t.trim().trim_matches('"') }));
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

    /// After the examples of ČÚZK's VKM description, with reduced coordinates.
    const VKM: &str = "&V K123456 740000 1040000 3\r\n&R 1000.00 1000.00 1300.00 1200.00 1000 R\r\n&D D=01012020 P=2 V=1.3 C=12\r\n&* komentář\r\n\
&U 1\r\n&L P 1270.00 1160.00 B=09900001 C=0001\r\nL 1261.37 1070.26 C=0020\r\nL 1200.00 1070.26 C=0021\r\n\
&L P 1100.00 1100.00 B=09900002 C=0030\r\nR 1110.00 1110.00 C=0031\r\nR 1120.00 1100.00 C=0032\r\n\
&L K 1163.09 1096.91 B=09900002 C=0012 R=5.00\r\n&U 2\r\n&T 1232.87 1084.25 ‚1' K=0028\r\n&T 1170.78 1025.65 'Řečice' H=2.5 U=100 D=8\r\n\
&U 3\r\n&L P 1054.31 1092.10 S=301\r\n&U 4\r\n&L P 1000.00 1000.00\r\nC 1010.00 1005.00\r\nC 1020.00 1000.00\r\nC 1030.00 1005.00\r\n&K\r\n";

    #[test]
    fn vkm_elements() {
        let m = parse(VKM).unwrap();
        assert!(!m.vgi);
        assert_eq!(m.elements.len(), 5);
        assert_eq!(m.texts.len(), 2);
        // Reduced coordinates are restored: Y 741270, X 1041160 → drawing (−741270, −1041160).
        let first = &m.elements[0].1[0];
        assert!((first.at.x + 741270.0).abs() < 1e-9 && (first.at.y + 1041160.0).abs() < 1e-9, "{first:?}");
        assert_eq!(first.number.as_deref(), Some("099000010001"));
        assert_eq!(m.elements[0].0, "KN_VKM_01_hranice_parcel");
        let mut s = Session::new();
        let r = s.execute("geovkm", &json!({ "text": VKM })).unwrap();
        assert_eq!(
            (r["lines"].as_u64(), r["arcs"].as_u64(), r["circles"].as_u64(), r["symbols"].as_u64(), r["texts"].as_u64()),
            (Some(3), Some(1), Some(1), Some(1), Some(2)),
            "{r}"
        );
        let d = s.doc().unwrap();
        // The arc from (1100, 1100) through (1110, 1110) to (1120, 1100) is a half circle.
        let arc = d
            .model
            .iter()
            .filter_map(|e| if let EntityKind::LwPolyline(p) = &e.kind { Some(p) } else { None })
            .find(|p| p.vertices.iter().any(|v| v.bulge != 0.0))
            .unwrap();
        assert!((arc.vertices[0].bulge.abs() - 1.0).abs() < 1e-9, "{arc:?}");
        // Texts at map scale: 2.5 mm at 1:1000 is 2.5 m, turned 100 gon.
        let t = d.model.iter().filter_map(|e| if let EntityKind::Text(t) = &e.kind { Some(t) } else { None }).find(|t| t.value == "Řečice").unwrap();
        assert!((t.height - 2.5).abs() < 1e-9 && (t.rotation - std::f64::consts::FRAC_PI_2).abs() < 1e-9, "{t:?}");
        assert!(d.model.iter().any(|e| matches!(&e.kind, EntityKind::Text(t) if t.value == "1")));
        // The interpolated curve passes through its points.
        let c = d
            .model
            .iter()
            .filter(|e| e.common.layer == "KN_VKM_04_vnitrni_kresba")
            .find_map(|e| if let EntityKind::LwPolyline(p) = &e.kind { Some(p) } else { None })
            .unwrap();
        assert!(c.vertices.len() > 10 && c.vertices.iter().any(|v| (v.p.x + 741010.0).abs() < 1e-9 && (v.p.y + 1041005.0).abs() < 1e-9));
        let r = s.execute("geovkm", &json!({ "text": VKM, "points": true })).unwrap();
        assert!(r["points"].as_u64().unwrap() >= 7, "{r}");
    }

    #[test]
    fn vgi_objects_and_layers() {
        let vgi = "&V KN804096 YX CM 0 0 3\n&R 575000 1277000 576000 1278000 1000\n&B KVALITA=2 MIERKA=1000 MAPA=KN\n&O KLADPAR 1\n&A PARCIS=5174.000\n\
&L P 575799.89 1277411.84 B=160001 C=321 K=1\nL 575793.58 1277432.11 C=405\n&T 575793.91 1277466.80 '5174' D=5 F=1 H=1.6 K=1\n&L P 575793.56 1277463.72 K=1 S=42\n&O ZAPPAR 3\n&L P 575826.50 1277511.16 B=160001 C=694 K=60\nL 575830.00 1277515.00\n&K\n";
        let m = parse(vgi).unwrap();
        assert!(m.vgi);
        assert_eq!(m.elements.len(), 3);
        assert_eq!(m.elements[0].0, "KN_VGI_KLADPAR");
        assert_eq!(m.elements[2].0, "KN_VGI_ZAPPAR");
        let mut s = Session::new();
        let r = s.execute("geovkm", &json!({ "text": vgi })).unwrap();
        assert_eq!(r["format"], "VGI");
        assert_eq!((r["lines"].as_u64(), r["symbols"].as_u64(), r["texts"].as_u64()), (Some(2), Some(1), Some(1)), "{r}");
    }

    #[test]
    fn code_pages() {
        // "Řečice" in DOS 852 and in Windows-1250.
        assert_eq!(decode_latin2(&[0xFC, 0x65, 0x9F, 0x69, 0x63, 0x65]), "Řečice");
        assert_eq!(decode_latin2(&[0xD8, 0xEC, 0xE8, 0x69, 0x63, 0x65]), "Řěčice");
        assert_eq!(decode_latin2("Řečice".as_bytes()), "Řečice");
    }

    #[test]
    fn hostile_input_is_refused_not_panicking() {
        let mut s = Session::new();
        for p in [
            json!({}),
            json!({ "text": "" }),
            json!({ "text": "hello\nworld" }),
            json!({ "text": "&V X 0 0\n&K" }),
            json!({ "path": "/nonexistent.vkm" }),
            json!({ "text": VKM, "scale": 0 }),
        ] {
            assert!(s.execute("geovkm", &p).is_err(), "{p}");
        }
        for junk in [
            "&L",
            "&L P x y",
            "&L R 1 1\nR 2 2",
            "&T 1 2 'open",
            "&T 1 2",
            "&L K 1 1 R=-5",
            "&V\n&R 1 2 3\nL 1 1",
            "&U\n&O\n&G",
            "&L P 1 1\nC 2 2\nC",
            "&S\n1 2\nx y z",
        ] {
            if let Ok(m) = parse(junk) {
                for (_, pts) in &m.elements {
                    let _ = shapes(pts);
                }
            }
        }
        let _ = curve(&[]);
        let _ = quoted("‚x");
    }
}
