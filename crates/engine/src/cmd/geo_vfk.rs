//! Survey add-on: the cadastral map from a VFK file (GEOVFK).
//!
//! VFK is the public text exchange format of the Czech cadastre (ČÚZK, "Výměnný formát ISKN v
//! textovém tvaru"). Blocks are declared by `&B<NAME>;COL TYPE;…` lines and filled by
//! `&D<NAME>;value;…` rows (`;` separated, text in double quotes with `""` for a quote, `¤` at the
//! end of a line continuing the row). The map's geometry:
//!
//! - SOBR: map points (ID, CISLO_BODU, SOURADNICE_Y, SOURADNICE_X), S-JTSK;
//! - SBP: the points (BP_ID → SOBR) of each line in order (PORADOVE_CISLO_BODU), belonging to a
//!   parcel boundary (HP_ID), a building outline (OB_ID) or another map element (DPM_ID), with
//!   PARAMETRY_SPOJENI saying when three points are an arc (16) or a circle (15, or "15 r");
//! - PAR: parcels and their numbers; OBDEBO: where a parcel's number is written.
//!
//! The lines go on KN_hranice_parcel, KN_budovy and KN_dalsi_prvky; parcel numbers on
//! KN_parcelni_cisla ("st." for building plots). Drawing coordinates are S-JTSK's x = −Y, y = −X.

use std::collections::HashMap;

use cadcraft_doc::{Circle, EntityKind, HAlign, LwPolyline, Point, VAlign};
use cadcraft_geom::{PolyVertex, Vec2, Vec3};
use serde_json::{Value, json};

use super::geo::decode;
use super::geo_calc::{add, ensure_layer, text};
use super::{CommandSpec, bad, f64_or, str_param};
use crate::{Accept, Input, Interactive, Prompt, Result, Session, Step};

pub const LAYER_BOUNDARIES: &str = "KN_hranice_parcel";
pub const LAYER_BUILDINGS: &str = "KN_budovy";
pub const LAYER_OTHER: &str = "KN_dalsi_prvky";
pub const LAYER_NUMBERS: &str = "KN_parcelni_cisla";
pub const LAYER_POINTS: &str = "KN_body";

/// Rows read from one file.
const MAX_ROWS: usize = 20_000_000;
/// Points of one line.
const MAX_LINE: usize = 1_000_000;

pub(super) fn specs() -> Vec<CommandSpec> {
    vec![
        CommandSpec::new("geovfk", "Import Cadastral Map (VFK)...", run_geovfk)
            .menu(&["Geo", "Cadastre", "Import Cadastral Map (VFK)..."])
            .alias(&["geokatastr", "-geovfk"])
            .params("{path | text: the VFK file, scale?: map scale 1:N for parcel numbers (default 1000), points?: bool, draw the map points with their numbers (default false)} → {boundaries, buildings, other, parcels, points, message}")
            .interactive(|_| Ok(Box::new(VfkM))),
    ]
}

/// One VFK block: column names and rows.
#[derive(Debug, Default)]
pub struct Table {
    pub cols: HashMap<String, usize>,
    pub rows: Vec<Vec<String>>,
}

impl Table {
    pub fn get<'a>(&self, row: &'a [String], col: &str) -> Option<&'a str> {
        self.cols.get(col).and_then(|i| row.get(*i)).map(String::as_str).filter(|v| !v.is_empty())
    }
    fn num(&self, row: &[String], col: &str) -> Option<f64> {
        self.get(row, col).and_then(|v| v.parse::<f64>().ok()).filter(|v| v.is_finite())
    }
}

/// Split one `&D` row's values: `;` separated, text quoted with `""` inside.
fn split_values(s: &str) -> Vec<String> {
    let mut out = Vec::new();
    let mut cur = String::new();
    let mut quoted = false;
    let mut chars = s.chars().peekable();
    while let Some(c) = chars.next() {
        match c {
            '"' if quoted && chars.peek() == Some(&'"') => {
                cur.push('"');
                chars.next();
            }
            '"' => quoted = !quoted,
            ';' if !quoted => out.push(std::mem::take(&mut cur)),
            c => cur.push(c),
        }
    }
    out.push(cur);
    out
}

/// The blocks of a VFK text (only those in `wanted`).
pub fn parse_vfk(text: &str, wanted: &[&str]) -> std::result::Result<HashMap<String, Table>, String> {
    let mut tables: HashMap<String, Table> = HashMap::new();
    let mut rows = 0usize;
    let mut pending = String::new();
    for raw in text.lines() {
        let line = raw.trim_end_matches('\r');
        // A row continues on the next line after `¤`.
        if let Some(head) = line.strip_suffix('¤') {
            pending.push_str(head);
            pending.push('\n');
            continue;
        }
        let line = if pending.is_empty() { line.to_string() } else { std::mem::take(&mut pending) + line };
        if let Some(rest) = line.strip_prefix("&B") {
            let mut parts = rest.split(';');
            let Some(name) = parts.next().map(str::trim) else { continue };
            if !wanted.contains(&name) {
                continue;
            }
            let cols = parts.enumerate().filter_map(|(i, c)| c.split_whitespace().next().map(|n| (n.to_string(), i))).collect();
            tables.insert(name.to_string(), Table { cols, rows: Vec::new() });
        } else if let Some(rest) = line.strip_prefix("&D") {
            let Some((name, values)) = rest.split_once(';') else { continue };
            let Some(t) = tables.get_mut(name.trim()) else { continue };
            rows += 1;
            if rows > MAX_ROWS {
                return Err(format!("more than {MAX_ROWS} rows"));
            }
            t.rows.push(split_values(values));
        }
    }
    Ok(tables)
}

/// Bulge of the arc from `a` through `m` to `b`, for a polyline segment a→b.
pub fn bulge(a: Vec2, m: Vec2, b: Vec2) -> Option<f64> {
    let (ax, ay, bx, by, cx, cy) = (a.x, a.y, m.x, m.y, b.x, b.y);
    let d = 2.0 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by));
    if d.abs() < 1e-12 {
        return None;
    }
    let ux = ((ax * ax + ay * ay) * (by - cy) + (bx * bx + by * by) * (cy - ay) + (cx * cx + cy * cy) * (ay - by)) / d;
    let uy = ((ax * ax + ay * ay) * (cx - bx) + (bx * bx + by * by) * (ax - cx) + (cx * cx + cy * cy) * (bx - ax)) / d;
    let ang = |p: Vec2| (p.y - uy).atan2(p.x - ux);
    let tau = std::f64::consts::TAU;
    // Counter-clockwise sweep from a to b, and whether m lies on it.
    let ccw = (ang(b) - ang(a)).rem_euclid(tau);
    let to_m = (ang(m) - ang(a)).rem_euclid(tau);
    let sweep = if to_m <= ccw { ccw } else { ccw - tau };
    let v = (sweep / 4.0).tan();
    v.is_finite().then_some(v)
}

/// Circle through three points.
fn circle3(a: Vec2, b: Vec2, c: Vec2) -> Option<(Vec2, f64)> {
    let d = 2.0 * (a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y));
    if d.abs() < 1e-12 {
        return None;
    }
    let s = |p: Vec2| p.x * p.x + p.y * p.y;
    let ux = (s(a) * (b.y - c.y) + s(b) * (c.y - a.y) + s(c) * (a.y - b.y)) / d;
    let uy = (s(a) * (c.x - b.x) + s(b) * (a.x - c.x) + s(c) * (b.x - a.x)) / d;
    let r = (a.x - ux).hypot(a.y - uy);
    (r.is_finite() && ux.is_finite() && uy.is_finite()).then_some((Vec2::new(ux, uy), r))
}

/// The pieces of one VFK line: polylines (with bulges) and circles.
pub enum Piece {
    Poly(Vec<PolyVertex>),
    Circle(Vec2, f64),
}

/// Turn the ordered points of a line and its PARAMETRY_SPOJENI into drawing pieces.
pub fn line_pieces(pts: &[Vec2], param: Option<&str>) -> Vec<Piece> {
    let code = param.map(str::trim).unwrap_or("");
    let mut f = code.split_whitespace();
    match (f.next(), f.next().and_then(|r| r.parse::<f64>().ok())) {
        (Some("15"), Some(r)) if r.is_finite() && r > 0.0 => pts.first().map(|c| vec![Piece::Circle(*c, r)]).unwrap_or_default(),
        (Some("15"), _) => match pts {
            [a, b, c] => circle3(*a, *b, *c).map(|(o, r)| vec![Piece::Circle(o, r)]).unwrap_or_default(),
            _ => Vec::new(),
        },
        (Some("16"), _) => match pts {
            [a, m, b] => {
                let bu = bulge(*a, *m, *b).unwrap_or(0.0);
                vec![Piece::Poly(vec![PolyVertex { p: *a, bulge: bu, ..PolyVertex::new(*a) }, PolyVertex::new(*b)])]
            }
            _ => vec![Piece::Poly(pts.iter().copied().map(PolyVertex::new).collect())],
        },
        _ => vec![Piece::Poly(pts.iter().copied().map(PolyVertex::new).collect())],
    }
}

fn run_geovfk(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "geovfk";
    let text_in = match (str_param(p, "text"), str_param(p, "path")) {
        (Some(t), _) => t.to_string(),
        (None, Some(path)) => {
            #[cfg(not(target_arch = "wasm32"))]
            {
                let meta = std::fs::metadata(path).map_err(|e| bad(CMD, format!("{path}: {e}")))?;
                if meta.len() > 2 * 1024 * 1024 * 1024 {
                    return Err(bad(CMD, format!("{path}: larger than 2 GB")));
                }
                decode(&std::fs::read(path).map_err(|e| bad(CMD, format!("{path}: {e}")))?)
            }
            #[cfg(target_arch = "wasm32")]
            return Err(bad(CMD, format!("{path}: files can't be read on the web; pass `text`")));
        }
        (None, None) => return Err(bad(CMD, "`path` (the VFK file) or `text` is required")),
    };
    let scale = f64_or(p, "scale", 1000.0);
    if !(scale.is_finite() && (1.0..=1e6).contains(&scale)) {
        return Err(bad(CMD, "`scale` must be a map scale like 1000"));
    }
    let t = parse_vfk(&text_in, &["SOBR", "SBP", "PAR", "OBDEBO", "OB", "DPM"]).map_err(|e| bad(CMD, e))?;
    let Some(sobr) = t.get("SOBR") else {
        return Err(bad(CMD, "no map points (block SOBR): is this a VFK file with the cadastral map?"));
    };
    // Map points by ID, in the drawing.
    let mut points: HashMap<String, (Vec2, Option<String>)> = HashMap::new();
    for r in &sobr.rows {
        let (Some(id), Some(y), Some(x)) = (sobr.get(r, "ID"), sobr.num(r, "SOURADNICE_Y"), sobr.num(r, "SOURADNICE_X")) else { continue };
        points.insert(id.to_string(), (Vec2::new(-y, -x), sobr.get(r, "CISLO_BODU").map(str::to_string)));
    }
    if points.is_empty() {
        return Err(bad(CMD, "the VFK file has no readable map points (SOBR)"));
    }
    let space = s.space();
    let (mut n_hp, mut n_ob, mut n_dpm) = (0, 0, 0);
    if let Some(sbp) = t.get("SBP") {
        // Group the line points by the element they belong to, in order.
        let mut lines: HashMap<(u8, String), Vec<(f64, Vec2, Option<String>)>> = HashMap::new();
        for r in &sbp.rows {
            let Some(bp) = sbp.get(r, "BP_ID").and_then(|id| points.get(id)) else { continue };
            let order = sbp.num(r, "PORADOVE_CISLO_BODU").unwrap_or(0.0);
            let owner = [("HP_ID", 0u8), ("OB_ID", 1), ("DPM_ID", 2)].iter().find_map(|(c, k)| sbp.get(r, c).map(|id| (*k, id.to_string())));
            let Some(owner) = owner else { continue };
            let list = lines.entry(owner).or_default();
            if list.len() < MAX_LINE {
                list.push((order, bp.0, sbp.get(r, "PARAMETRY_SPOJENI").map(str::to_string)));
            }
        }
        for (layer, aci) in [(LAYER_BOUNDARIES, 7u8), (LAYER_BUILDINGS, 7), (LAYER_OTHER, 8)] {
            ensure_layer(s, layer, aci)?;
        }
        let mut keys: Vec<&(u8, String)> = lines.keys().collect();
        keys.sort();
        for key in keys {
            let Some(list) = lines.get(key) else { continue };
            let mut list: Vec<&(f64, Vec2, Option<String>)> = list.iter().collect();
            list.sort_by(|a, b| a.0.total_cmp(&b.0));
            let pts: Vec<Vec2> = list.iter().map(|(_, q, _)| *q).collect();
            let param = list.iter().find_map(|(_, _, p)| p.as_deref());
            let layer = match key.0 {
                0 => LAYER_BOUNDARIES,
                1 => LAYER_BUILDINGS,
                _ => LAYER_OTHER,
            };
            for piece in line_pieces(&pts, param) {
                match piece {
                    Piece::Poly(v) if v.len() >= 2 => {
                        add(
                            s,
                            &space,
                            layer,
                            EntityKind::LwPolyline(LwPolyline { vertices: v, closed: false, const_width: 0.0, elevation: 0.0, plinegen: false }),
                        )?;
                    }
                    Piece::Circle(c, r) => {
                        add(s, &space, layer, EntityKind::Circle(Circle { center: Vec3::new(c.x, c.y, 0.0), radius: r }))?;
                    }
                    Piece::Poly(_) => continue,
                }
                match key.0 {
                    0 => n_hp += 1,
                    1 => n_ob += 1,
                    _ => n_dpm += 1,
                }
            }
        }
    }
    // Parcel numbers at their definition points.
    let mut n_par = 0;
    if let (Some(par), Some(obdebo)) = (t.get("PAR"), t.get("OBDEBO")) {
        let mut names: HashMap<&str, String> = HashMap::new();
        for r in &par.rows {
            let (Some(id), Some(kmen)) = (par.get(r, "ID"), par.get(r, "KMENOVE_CISLO_PAR")) else { continue };
            let mut name = kmen.to_string();
            if let Some(pod) = par.get(r, "PODDELENI_CISLA_PAR") {
                name = format!("{name}/{pod}");
            }
            // Numbering kind 1: building plots.
            if par.get(r, "DRUH_CISLOVANI_PAR") == Some("1") {
                name = format!("st. {name}");
            }
            names.insert(id, name);
        }
        ensure_layer(s, LAYER_NUMBERS, 3)?;
        let h = 0.002 * scale;
        for r in &obdebo.rows {
            let (Some(pid), Some(y), Some(x)) = (obdebo.get(r, "PAR_ID"), obdebo.num(r, "SOURADNICE_Y"), obdebo.num(r, "SOURADNICE_X")) else {
                continue;
            };
            let Some(name) = names.get(pid) else { continue };
            text(s, &space, LAYER_NUMBERS, Vec2::new(-y, -x), h, name.clone(), 0.0, (HAlign::Center, VAlign::Middle))?;
            n_par += 1;
        }
    }
    let mut n_pts = 0;
    if p.get("points").and_then(Value::as_bool).unwrap_or(false) {
        ensure_layer(s, LAYER_POINTS, 8)?;
        let h = 0.0015 * scale;
        let mut all: Vec<&(Vec2, Option<String>)> = points.values().collect();
        all.sort_by(|a, b| a.0.x.total_cmp(&b.0.x).then(a.0.y.total_cmp(&b.0.y)));
        for (q, num) in all {
            add(s, &space, LAYER_POINTS, EntityKind::Point(Point { p: Vec3::new(q.x, q.y, 0.0), angle: 0.0 }))?;
            if let Some(n) = num {
                text(s, &space, LAYER_POINTS, Vec2::new(q.x + 0.5 * h, q.y + 0.3 * h), h, n.clone(), 0.0, (HAlign::Left, VAlign::Baseline))?;
            }
            n_pts += 1;
        }
    }
    Ok(json!({
        "boundaries": n_hp, "buildings": n_ob, "other": n_dpm, "parcels": n_par, "points": n_pts,
        "message": format!("Cadastral map: {n_hp} parcel boundaries, {n_ob} building outlines, {n_dpm} other lines, {n_par} parcel numbers"),
    }))
}

struct VfkM;

impl Interactive for VfkM {
    fn name(&self) -> &'static str {
        "GEOVFK"
    }
    fn prompt(&self, _s: &Session) -> Prompt {
        Prompt::new("VFK file", Accept::TEXT)
    }
    fn input(&mut self, s: &mut Session, i: Input) -> Result<Step> {
        match i {
            Input::Cancel | Input::Enter => Ok(Step::Done),
            Input::Text(t) | Input::Keyword(t) => {
                let r = run_geovfk(s, &json!({ "path": t.trim().trim_matches('"') }));
                super::geo_calc::say(s, r);
                Ok(Step::Done)
            }
            _ => Ok(Step::Continue),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A tiny cadastral map after the VFK description: two parcels split by a boundary, a
    /// building and an arc.
    fn sample() -> String {
        [
            "&HVERZE;\"6.1\"",
            "&HCODEPAGE;\"UTF-8\"",
            "&BSOBR;ID N30;STAV_DAT N2;KATUZE_KOD N6;CISLO_ZPMZ N5;CISLO_TL N4;CISLO_BODU N12;UPLNE_CISLO N12;SOURADNICE_Y N10.2;SOURADNICE_X N10.2;KODCHB_KOD N2",
            "&DSOBR;1;0;123;45;;1;;743000;1043000;",
            "&DSOBR;2;0;123;45;;2;;743020;1043000;",
            "&DSOBR;3;0;123;45;;3;;743020;1043020;",
            "&DSOBR;4;0;123;45;;4;;743000;1043020;",
            "&DSOBR;5;0;123;45;;5;;743010;1043000;",
            "&DSOBR;6;0;123;45;;6;;743010;1043020;",
            "&DSOBR;7;0;123;45;;7;;743014;1043010;",
            "&BSBP;ID N30;STAV_DAT N2;DATUM_VZNIKU D;DATUM_ZANIKU D;PRIZNAK_KONTEXTU N1;RIZENI_ID_VZNIKU N30;RIZENI_ID_ZANIKU N30;BP_ID N30;PORADOVE_CISLO_BODU N38;OB_ID N30;HP_ID N30;DPM_ID N30;PARAMETRY_SPOJENI T100;ZVB_ID N30",
            "&DSBP;1;0;\"01.01.2020 00:00:00\";\"\";;;;1;1;;100;;;",
            "&DSBP;2;0;\"01.01.2020 00:00:00\";\"\";;;;2;2;;100;;;",
            "&DSBP;3;0;;\"\";;;;2;1;;101;;;",
            "&DSBP;4;0;;\"\";;;;3;2;;101;;;",
            "&DSBP;5;0;;\"\";;;;5;1;;102;;16;",
            "&DSBP;6;0;;\"\";;;;7;2;;102;;16;",
            "&DSBP;7;0;;\"\";;;;6;3;;102;;16;",
            "&DSBP;8;0;;\"\";;;;1;1;500;;;;",
            "&DSBP;9;0;;\"\";;;;5;2;500;;;;",
            "&BPAR;ID N30;STAV_DAT N2;PAR_TYPE T10;KATUZE_KOD N6;DRUH_CISLOVANI_PAR N1;KMENOVE_CISLO_PAR N5;PODDELENI_CISLA_PAR N3;VYMERA_PARCELY N9",
            "&DPAR;9001;0;\"PKN\";123;2;269;2;200",
            "&DPAR;9002;0;\"PKN\";123;1;31;;200",
            "&BOBDEBO;ID N30;STAV_DAT N2;PAR_ID N30;BUD_ID N30;SOURADNICE_Y N10.2;SOURADNICE_X N10.2",
            "&DOBDEBO;1;0;9001;;743005;1043010",
            "&DOBDEBO;2;0;9002;;743016;1043010",
            "&K",
        ]
        .join("\r\n")
    }

    #[test]
    fn values_split_like_vfk() {
        assert_eq!(split_values("1;\"a;b\";\"say \"\"hi\"\"\";;2"), vec!["1", "a;b", "say \"hi\"", "", "2"]);
        let t = parse_vfk("&BX;A N1;B T5\r\n&DX;1;\"line one¤\r\nline two\"\r\n", &["X"]).unwrap();
        assert_eq!(t["X"].rows[0][1], "line one\nline two");
    }

    #[test]
    fn a_small_map_is_drawn() {
        let mut s = Session::new();
        let r = s.execute("geovfk", &json!({ "text": sample() })).unwrap();
        assert_eq!(r["boundaries"], 3, "{r}");
        assert_eq!(r["buildings"], 1, "{r}");
        assert_eq!(r["parcels"], 2, "{r}");
        let d = s.doc().unwrap();
        let labels: Vec<String> = d
            .model
            .iter()
            .filter(|e| e.common.layer == LAYER_NUMBERS)
            .filter_map(|e| if let EntityKind::Text(t) = &e.kind { Some(t.value.clone()) } else { None })
            .collect();
        assert!(labels.contains(&"269/2".to_string()) && labels.contains(&"st. 31".to_string()), "{labels:?}");
        // The arc boundary carries a bulge; coordinates are S-JTSK's x = −Y, y = −X.
        let arc = d
            .model
            .iter()
            .filter_map(|e| if let EntityKind::LwPolyline(p) = &e.kind { Some(p) } else { None })
            .find(|p| p.vertices.iter().any(|v| v.bulge.abs() > 1e-6));
        let arc = arc.expect("an arc boundary");
        assert!((arc.vertices[0].p.x + 743010.0).abs() < 1e-9 && (arc.vertices[0].p.y + 1043000.0).abs() < 1e-9);
        let r = s.execute("geovfk", &json!({ "text": sample(), "points": true })).unwrap();
        assert_eq!(r["points"], 7);
    }

    #[test]
    fn arcs_bulge_the_right_way() {
        // Half circle from (0,0) through (1,1) to (2,0): clockwise, bulge −1.
        let b = bulge(Vec2::new(0.0, 0.0), Vec2::new(1.0, 1.0), Vec2::new(2.0, 0.0)).unwrap();
        assert!((b + 1.0).abs() < 1e-12, "{b}");
        let b = bulge(Vec2::new(0.0, 0.0), Vec2::new(1.0, -1.0), Vec2::new(2.0, 0.0)).unwrap();
        assert!((b - 1.0).abs() < 1e-12, "{b}");
        assert!(bulge(Vec2::ZERO, Vec2::new(1.0, 0.0), Vec2::new(2.0, 0.0)).is_none());
        assert!(matches!(line_pieces(&[Vec2::new(5.0, 5.0)], Some("15 2.5")).as_slice(), [Piece::Circle(_, r)] if (*r - 2.5).abs() < 1e-12));
    }

    #[test]
    fn hostile_input_is_refused_not_panicking() {
        let mut s = Session::new();
        for p in [
            json!({}),
            json!({ "text": "" }),
            json!({ "text": "&BSOBR;ID N30\r\n&DSOBR;\"" }),
            json!({ "text": sample(), "scale": 0 }),
            json!({ "path": "/nonexistent.vfk" }),
        ] {
            assert!(s.execute("geovfk", &p).is_err(), "{p}");
        }
        // Rows that reference missing points or carry junk are skipped, not fatal.
        let junk = "&BSOBR;ID N30;CISLO_BODU N12;SOURADNICE_Y N10.2;SOURADNICE_X N10.2\r\n&DSOBR;1;1;x;y\r\n&DSOBR;2;2;5;6\r\n&BSBP;BP_ID N30;PORADOVE_CISLO_BODU N38;HP_ID N30;PARAMETRY_SPOJENI T100\r\n&DSBP;9;1;5;16\r\n&DSBP;1;;;\r\n";
        assert!(s.execute("geovfk", &json!({ "text": junk })).is_ok());
        let _ = split_values("\"unterminated;");
        let _ = line_pieces(&[], Some("15"));
        let _ = line_pieces(&[], Some("15 -3"));
    }
}
