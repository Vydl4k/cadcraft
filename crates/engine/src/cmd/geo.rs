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
//! `GEO_cisla`, its height on `GEO_vysky` and its code (when it has one) on `GEO_kody`. Text
//! heights follow the map scale: 2 mm on paper (1 m at 1:500), the drawing in metres.

use cadcraft_doc::{Common, EntityKind, HAlign, Point, Text, VAlign};
use cadcraft_geom::{Vec2, Vec3};
use serde_json::{Value, json};

use super::file::base64_decode;
use super::{CommandSpec, bad, bool_or, f64_or, str_param};
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

pub(super) fn specs() -> Vec<CommandSpec> {
    vec![
        CommandSpec::new(CMD, "Import Points (Coordinate List)...", run_geobody)
            .menu(&["Geo", "Import Points..."])
            .alias(&["-geobody", "geopoints"])
            .params("{path | data: base64 | text, columns?: order of P(number) Y X Z K(code) and -(skip), default \"PYXZK\", sjtsk?: \"auto\"|true|false (x=-Y, y=-X; default auto), scale?: map scale 1:N for label size (default 500), decimals?: height decimals (default 2), numbers?: bool, heights?: bool, codes?: bool (default true)} → {imported, skipped, sjtsk, extents, message}")
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
}

/// What a column of the list holds.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Col {
    Number,
    Y,
    X,
    Z,
    Code,
    Skip,
}

fn columns(spec: &str) -> std::result::Result<Vec<Col>, String> {
    let cols: Vec<Col> = spec
        .chars()
        .filter(|c| !c.is_whitespace())
        .map(|c| match c.to_ascii_uppercase() {
            'P' | 'C' | 'N' => Ok(Col::Number),
            'Y' => Ok(Col::Y),
            'X' => Ok(Col::X),
            'Z' | 'H' => Ok(Col::Z),
            'K' => Ok(Col::Code),
            '-' | '_' => Ok(Col::Skip),
            other => Err(format!("unknown column \"{other}\" in \"{spec}\": use P (number), Y, X, Z, K (code) and - (skip)")),
        })
        .collect::<std::result::Result<_, _>>()?;
    if !cols.contains(&Col::Y) || !cols.contains(&Col::X) {
        return Err(format!("the columns \"{spec}\" need Y and X"));
    }
    if cols.len() > 16 {
        return Err("at most 16 columns".into());
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
        let mut b = Bod { cislo: String::new(), y: f64::NAN, x: f64::NAN, z: None, kod: String::new() };
        for (i, c) in cols.iter().enumerate() {
            let f = fields.get(i).copied().unwrap_or("");
            match c {
                Col::Number => b.cislo = f.chars().take(64).collect(),
                Col::Y => b.y = number(f, comma_decimal).unwrap_or(f64::NAN),
                Col::X => b.x = number(f, comma_decimal).unwrap_or(f64::NAN),
                Col::Z => b.z = if f.is_empty() { None } else { number(f, comma_decimal) },
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

fn label(s: &mut Session, layer: &str, at: Vec2, height: f64, value: String, valign: VAlign) -> Result<()> {
    let space = s.space();
    let text = Text {
        insert: Vec3::new(at.x, at.y, 0.0),
        align_pt: None,
        height,
        value,
        rotation: 0.0,
        width_factor: 1.0,
        oblique: 0.0,
        style: "Standard".into(),
        halign: HAlign::Left,
        valign,
    };
    let common = Common { layer: layer.into(), ..s.current_common() };
    s.doc_mut()?.add(&space, common, EntityKind::Text(text)).map(|_| ()).map_err(|e| bad(CMD, e.to_string()))
}

fn run_geobody(s: &mut Session, p: &Value) -> Result<Value> {
    let (text, file) = if let Some(t) = str_param(p, "text") {
        (t.to_string(), "text".to_string())
    } else if let Some(path) = str_param(p, "path") {
        (decode(&read_path(path)?), path.to_string())
    } else {
        let data = str_param(p, "data").ok_or_else(|| bad(CMD, "`path`, `data` (base64) or `text` is required"))?;
        (decode(&base64_decode(data).ok_or_else(|| bad(CMD, "invalid base64"))?), "data".to_string())
    };
    let spec = str_param(p, "columns").unwrap_or("PYXZK");
    let (pts, skipped) = parse(&text, spec).map_err(|e| bad(CMD, e))?;
    if pts.is_empty() {
        return Err(bad(CMD, format!("no points found (columns \"{spec}\"; {skipped} lines skipped)")));
    }
    let sjtsk = match p.get("sjtsk") {
        None => looks_sjtsk(&pts),
        Some(Value::Bool(b)) => *b,
        Some(Value::String(t)) if t.eq_ignore_ascii_case("auto") => looks_sjtsk(&pts),
        Some(_) => return Err(bad(CMD, "`sjtsk` must be true, false or \"auto\"")),
    };
    let scale = f64_or(p, "scale", 500.0);
    if !(scale.is_finite() && (1.0..=1_000_000.0).contains(&scale)) {
        return Err(bad(CMD, "`scale` must be a map scale like 500 or 1000"));
    }
    let decimals = p.get("decimals").and_then(Value::as_u64).unwrap_or(2).min(4) as usize;
    let (numbers, heights, codes) = (bool_or(p, "numbers", true), bool_or(p, "heights", true), bool_or(p, "codes", true));
    // 2 mm on paper, in metres.
    let h = 0.002 * scale;

    {
        let d = s.doc_mut()?;
        for (name, aci) in [(LAYER_POINTS, 7), (LAYER_NUMBERS, 3), (LAYER_HEIGHTS, 1), (LAYER_CODES, 5)] {
            if d.ensure_layer(name)
                && let Some(l) = d.layer_mut(name)
            {
                l.color = cadcraft_color::Color::Index(aci);
            }
        }
    }
    let space = s.space();
    let mut handles = Vec::new();
    let (mut lo, mut hi) = (Vec2::new(f64::INFINITY, f64::INFINITY), Vec2::new(f64::NEG_INFINITY, f64::NEG_INFINITY));
    for b in &pts {
        let at = if sjtsk { Vec2::new(-b.y, -b.x) } else { Vec2::new(b.y, b.x) };
        lo = Vec2::new(lo.x.min(at.x), lo.y.min(at.y));
        hi = Vec2::new(hi.x.max(at.x), hi.y.max(at.y));
        let common = Common { layer: LAYER_POINTS.into(), ..s.current_common() };
        let pt = Point { p: Vec3::new(at.x, at.y, b.z.unwrap_or(0.0)), angle: 0.0 };
        let hd = s.doc_mut()?.add(&space, common, EntityKind::Point(pt)).map_err(|e| bad(CMD, e.to_string()))?;
        handles.push(hd);
        let right = Vec2::new(at.x + 0.6 * h, at.y);
        if numbers && !b.cislo.is_empty() {
            label(s, LAYER_NUMBERS, Vec2::new(right.x, right.y + 0.25 * h), h, b.cislo.clone(), VAlign::Baseline)?;
        }
        if heights && let Some(z) = b.z {
            label(s, LAYER_HEIGHTS, Vec2::new(right.x, right.y - 1.25 * h), h, format!("{z:.decimals$}"), VAlign::Baseline)?;
        }
        if codes && !b.kod.is_empty() {
            label(s, LAYER_CODES, Vec2::new(right.x, right.y - 2.75 * h), 0.8 * h, b.kod.clone(), VAlign::Baseline)?;
        }
    }
    s.set_selection(handles.clone());
    let name = std::path::Path::new(&file).file_name().map(|n| n.to_string_lossy().to_string()).unwrap_or_default();
    Ok(json!({
        "imported": handles.len(),
        "skipped": skipped,
        "sjtsk": sjtsk,
        "extents": [[lo.x, lo.y], [hi.x, hi.y]],
        "message": format!(
            "{} points imported from {name}{}{}",
            handles.len(),
            if sjtsk { " (S-JTSK: x = -Y, y = -X)" } else { "" },
            if skipped > 0 { format!("; {skipped} lines without a point skipped") } else { String::new() }
        ),
    }))
}

#[derive(Default)]
struct BodyM {
    path: Option<String>,
}

impl Interactive for BodyM {
    fn name(&self) -> &'static str {
        "GEOBODY"
    }

    fn prompt(&self, _s: &Session) -> Prompt {
        match self.path {
            None => Prompt::new("Enter name of coordinate list to import", Accept::TEXT),
            Some(_) => Prompt::new("Columns (P number, Y, X, Z, K code, - skip)", Accept::TEXT).default("PYXZK"),
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
        let spec = if t.is_empty() { "PYXZK".to_string() } else { t };
        if let Err(e) = columns(&spec) {
            s.echo(e);
            return Ok(Step::Continue);
        }
        match run_geobody(s, &json!({ "path": path, "columns": spec })) {
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
    fn lists_in_the_usual_shapes() {
        let (p, skipped) =
            parse("cislo Y X Z kod\n4001 742123.45 1043210.10 312.55 plot\n4002\t742124,00\t1043211,50\t312,60\tzed dum\n", "PYXZK").unwrap();
        assert_eq!(skipped, 1);
        assert_eq!(p.len(), 2);
        assert_eq!(p[0], Bod { cislo: "4001".into(), y: 742123.45, x: 1043210.10, z: Some(312.55), kod: "plot".into() });
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
}
