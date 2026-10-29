//! PDFIMPORT: the vector content of a PDF page as editable objects.
//!
//! Lines, curves, filled areas and text of one page come in at a drawing scale (1:50 sheet →
//! real size) in the drawing's units, on `PDF_…` layers (see `cadcraft_io::pdf_read`). The reader
//! is a hook ([`super::file::pdf`]) the app installs with the io layer's.
//!
//! JSON form: `{path | data, page?, scale?, units?, at?, fills?, whiteFills?, text?}`. Typed or
//! from the menu it asks for the file, the page (when there are several), the scale and the
//! insertion point.

use std::sync::atomic::{AtomicU64, Ordering};

use cadcraft_geom::Vec2;
use serde_json::{Value, json};

use super::file::{PdfImportRequest, base64_decode, pdf};
use super::{CommandSpec, bad, bool_or, point_value, str_param};
use crate::{Accept, EngineError, Input, Interactive, Prompt, Result, Session, Step};

/// Points per inch, millimetres per inch.
const MM_PER_POINT: f64 = 25.4 / 72.0;
/// Largest PDF read.
const MAX_BYTES: u64 = 256 << 20;
/// The scale used last (as f64 bits), offered as the next default; 0 = none yet.
static LAST_SCALE: AtomicU64 = AtomicU64::new(0);

pub(super) fn specs() -> Vec<CommandSpec> {
    vec![
        CommandSpec::new("pdfimport", "PDF Import...", run_pdfimport)
            .menu(&["Insert", "PDF Import..."])
            .alias(&["pdfi", "-pdfimport"])
            .params("{path | data: base64, page?: (default 1), scale?: drawing scale 1:N (default 1: the sheet's real size), units?: mm|cm|dm|m|in|ft (default the drawing's INSUNITS, mm when unitless), at?: [x,y] (default 0,0), fills?: bool (default true), whiteFills?: bool (default false), text?: bool (default true)} → {imported, handles, layers, page, pages, message}")
            .interactive(|_| Ok(Box::new(PdfM::default()))),
    ]
}

fn last_scale() -> f64 {
    let v = f64::from_bits(LAST_SCALE.load(Ordering::Relaxed));
    if v.is_finite() && v > 0.0 { v } else { 1.0 }
}

/// Millimetres per drawing unit: from `units`, else the drawing's INSUNITS.
fn mm_per_unit(s: &Session, units: Option<&str>) -> Result<f64> {
    if let Some(u) = units {
        return Ok(match u.trim().to_ascii_lowercase().as_str() {
            "mm" | "millimeters" | "millimetres" => 1.0,
            "cm" | "centimeters" | "centimetres" => 10.0,
            "dm" | "decimeters" | "decimetres" => 100.0,
            "m" | "meters" | "metres" => 1000.0,
            "in" | "inch" | "inches" => 25.4,
            "ft" | "feet" | "foot" => 304.8,
            other => return Err(bad("pdfimport", format!("unknown units \"{other}\": use mm, cm, dm, m, in or ft"))),
        });
    }
    Ok(match s.doc()?.header.i64("INSUNITS", 4) {
        1 => 25.4,
        2 => 304.8,
        5 => 10.0,
        6 => 1000.0,
        14 => 100.0,
        _ => 1.0,
    })
}

fn read_path(path: &str) -> Result<Vec<u8>> {
    #[cfg(not(target_arch = "wasm32"))]
    {
        let p = std::path::Path::new(path);
        let len = std::fs::metadata(p).map_err(|e| bad("pdfimport", format!("{path}: {e}")))?.len();
        if len > MAX_BYTES {
            return Err(bad("pdfimport", format!("{path} is larger than {} MB", MAX_BYTES >> 20)));
        }
        std::fs::read(p).map_err(|e| bad("pdfimport", format!("{path}: {e}")))
    }
    #[cfg(target_arch = "wasm32")]
    {
        let _ = (path, MAX_BYTES);
        Err(bad("pdfimport", "paths are not available on the web; pass `data`"))
    }
}

/// The scale as typed: `50`, `1:50` or `1/50` all mean 1:50.
fn parse_scale(t: &str) -> Option<f64> {
    let t = t.trim();
    let n = match t.split_once([':', '/']) {
        Some((a, b)) => {
            let a: f64 = a.trim().replace(',', ".").parse().ok()?;
            let b: f64 = b.trim().replace(',', ".").parse().ok()?;
            if a <= 0.0 {
                return None;
            }
            b / a
        }
        None => t.replace(',', ".").parse().ok()?,
    };
    (n.is_finite() && (1e-6..=1e6).contains(&n)).then_some(n)
}

fn run_pdfimport(s: &mut Session, p: &Value) -> Result<Value> {
    let hooks = pdf().ok_or_else(|| bad("pdfimport", "PDF import is not available in this build"))?;
    let (bytes, name) = if let Some(path) = str_param(p, "path") {
        (read_path(path)?, path.to_string())
    } else {
        let data = str_param(p, "data").ok_or_else(|| bad("pdfimport", "`path` or `data` (base64) is required"))?;
        (base64_decode(data).ok_or_else(|| bad("pdfimport", "invalid base64"))?, str_param(p, "name").unwrap_or("import.pdf").to_string())
    };
    let page = match p.get("page") {
        None => 1,
        Some(v) => v.as_u64().and_then(|n| usize::try_from(n).ok()).filter(|n| *n >= 1).ok_or_else(|| bad("pdfimport", "`page` must be a page number from 1"))?,
    };
    let scale = match p.get("scale") {
        None => 1.0,
        Some(Value::String(t)) => parse_scale(t).ok_or_else(|| bad("pdfimport", "`scale` must be a number like 50 or \"1:50\""))?,
        Some(v) => v.as_f64().filter(|n| n.is_finite() && (1e-6..=1e6).contains(n)).ok_or_else(|| bad("pdfimport", "`scale` must be a positive number (50 for 1:50)"))?,
    };
    let at = match p.get("at") {
        Some(v) => point_value(v).ok_or_else(|| bad("pdfimport", "`at` must be a point [x, y]"))?,
        None => Vec2::ZERO,
    };
    let mm = mm_per_unit(s, str_param(p, "units"))?;
    let req = PdfImportRequest {
        page,
        units_per_point: MM_PER_POINT * scale / mm,
        fills: bool_or(p, "fills", true),
        white_fills: bool_or(p, "whiteFills", false),
        text: bool_or(p, "text", true),
    };
    let pages = (hooks.pages)(&bytes).map_err(|e| bad("pdfimport", e))?;
    let src = (hooks.import)(&bytes, &req).map_err(|e| bad("pdfimport", e))?;
    for note in &src.read_notes {
        s.echo(note.clone());
    }
    let file = std::path::Path::new(&name).file_name().map(|n| n.to_string_lossy().to_string()).unwrap_or_default();
    let mut r = super::exchange::import_drawing(s, &src, at, 1.0, &file)?;
    LAST_SCALE.store(scale.to_bits(), Ordering::Relaxed);
    if let Some(o) = r.as_object_mut() {
        o.insert("page".into(), json!(page));
        o.insert("pages".into(), json!(pages));
        o.insert("scale".into(), json!(scale));
        let n = o.get("imported").and_then(Value::as_u64).unwrap_or(0);
        o.insert("message".into(), json!(format!("{n} objects imported from page {page} of {file} at 1:{}", fmt_scale(scale))));
    }
    Ok(r)
}

fn fmt_scale(n: f64) -> String {
    let t = format!("{n:.4}");
    t.trim_end_matches('0').trim_end_matches('.').to_string()
}

#[derive(Default)]
enum Stage {
    #[default]
    File,
    Page,
    Scale,
    Point,
}

#[derive(Default)]
struct PdfM {
    stage: Stage,
    path: String,
    pages: usize,
    page: usize,
    scale: f64,
}

/// Typed text, or `None` for Enter.
fn text(i: &Input) -> Option<String> {
    match i {
        Input::Text(t) | Input::Keyword(t) => Some(t.trim().to_string()).filter(|t| !t.is_empty()),
        _ => None,
    }
}

impl Interactive for PdfM {
    fn name(&self) -> &'static str {
        "PDFIMPORT"
    }

    fn prompt(&self, _s: &Session) -> Prompt {
        match self.stage {
            Stage::File => Prompt::new("Enter name of PDF file to import", Accept::TEXT),
            Stage::Page => Prompt::new(format!("Enter page number (1-{})", self.pages), Accept::NUMBER).default("1"),
            Stage::Scale => Prompt::new("Enter the drawing scale of the PDF, 1:", Accept::TEXT).default(fmt_scale(last_scale())),
            Stage::Point => Prompt::new("Specify insertion point", Accept::POINT).default("0,0"),
        }
    }

    fn input(&mut self, s: &mut Session, i: Input) -> Result<Step> {
        if matches!(i, Input::Cancel) {
            return Ok(Step::Cancel);
        }
        match self.stage {
            Stage::File => {
                let Some(path) = text(&i) else { return Ok(Step::Continue) };
                let path = path.trim_matches('"').to_string();
                let hooks = pdf().ok_or_else(|| bad("pdfimport", "PDF import is not available in this build"))?;
                let pages = match read_path(&path).and_then(|b| (hooks.pages)(&b).map_err(|e| bad("pdfimport", e))) {
                    Ok(n) => n,
                    Err(e) => {
                        s.echo(message(e));
                        return Ok(Step::Continue);
                    }
                };
                if pages == 0 {
                    s.echo("The PDF has no pages.");
                    return Ok(Step::Done);
                }
                self.path = path;
                self.pages = pages;
                self.page = 1;
                self.stage = if pages > 1 { Stage::Page } else { Stage::Scale };
                Ok(Step::Continue)
            }
            Stage::Page => {
                let n = match text(&i) {
                    None => 1,
                    Some(t) => match t.parse::<usize>() {
                        Ok(n) if (1..=self.pages).contains(&n) => n,
                        _ => {
                            s.echo(format!("Enter a page number from 1 to {}.", self.pages));
                            return Ok(Step::Continue);
                        }
                    },
                };
                self.page = n;
                self.stage = Stage::Scale;
                Ok(Step::Continue)
            }
            Stage::Scale => {
                self.scale = match text(&i) {
                    None => last_scale(),
                    Some(t) => match parse_scale(&t) {
                        Some(n) => n,
                        None => {
                            s.echo("Enter the scale the PDF is drawn at, like 50 or 1:50.");
                            return Ok(Step::Continue);
                        }
                    },
                };
                self.stage = Stage::Point;
                Ok(Step::Continue)
            }
            Stage::Point => {
                let at = match &i {
                    Input::Point(p) => *p,
                    Input::Enter => Vec2::ZERO,
                    Input::Text(t) => match crate::prompt::parse_point(t, Vec2::ZERO) {
                        Some(p) => p,
                        None => {
                            s.echo("Specify a point, or press Enter for 0,0.");
                            return Ok(Step::Continue);
                        }
                    },
                    _ => return Ok(Step::Continue),
                };
                let p = json!({ "path": self.path, "page": self.page, "scale": self.scale, "at": [at.x, at.y] });
                match run_pdfimport(s, &p) {
                    Ok(r) => {
                        if let Some(m) = r.get("message").and_then(Value::as_str) {
                            s.echo(m.to_string());
                        }
                    }
                    Err(e) => s.echo(message(e)),
                }
                Ok(Step::Done)
            }
        }
    }
}

fn message(e: EngineError) -> String {
    match e {
        EngineError::BadParams { msg, .. } => msg,
        other => other.to_string(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn scales_as_typed() {
        assert_eq!(parse_scale("50"), Some(50.0));
        assert_eq!(parse_scale("1:50"), Some(50.0));
        assert_eq!(parse_scale(" 1 / 100 "), Some(100.0));
        assert_eq!(parse_scale("2,5"), Some(2.5));
        for bad in ["", "0", "-5", "1:0", "0:50", "abc", "1:x", "NaN", "inf", "1e9"] {
            assert_eq!(parse_scale(bad), None, "{bad}");
        }
    }

    #[test]
    fn without_a_reader_it_says_so() {
        let mut s = Session::new();
        let r = s.execute("pdfimport", &json!({ "data": "JVBERg==" }));
        assert!(r.is_err());
    }

    #[test]
    fn hostile_params_are_refused() {
        let mut s = Session::new();
        for p in [json!({}), json!({ "page": 0, "data": "" }), json!({ "scale": -1, "data": "" }), json!({ "units": "parsec", "data": "" })] {
            assert!(s.execute("pdfimport", &p).is_err(), "{p}");
        }
    }
}
