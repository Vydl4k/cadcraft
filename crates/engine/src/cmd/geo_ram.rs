//! Survey add-on: a drawing sheet frame with a title block (GEORAM), in model space at the
//! drawing's scale.
//!
//! Formats are ISO A0–A4, or a strip of n A4 widths (n × 210 × 297 mm, folded to A4), as building
//! survey sheets often are. The frame is the trimmed sheet with a 5 mm border (20 mm on the left
//! for binding when `binding` is on), fold marks every 210 mm along a strip, and the title block
//! in the lower right corner: who surveyed, drew and checked it, the format and date, the place,
//! address, the drawing title with its scale and number, and the ±0,00 height when given.
//! Our own layout, after the usual Czech building drawing practice.

use cadcraft_doc::{EntityKind, HAlign, Line, LwPolyline, VAlign};
use cadcraft_geom::{PolyVertex, Vec2, Vec3};
use serde_json::{Value, json};

use super::geo_calc::{add, ensure_layer, say, text, typed_number};
use super::{CommandSpec, bad, f64_or, point_value, str_param};
use crate::{Accept, Input, Interactive, Prompt, Result, Session, Step};

pub const LAYER_FRAME: &str = "GEO_ram";
pub const LAYER_TITLE: &str = "GEO_rozpiska";

pub(super) fn specs() -> Vec<CommandSpec> {
    vec![
        CommandSpec::new("georam", "Sheet Frame and Title Block...", run_georam)
            .menu(&["Geo", "Sheet Frame and Title Block..."])
            .alias(&["georozpiska", "geoframe"])
            .params("{format?: \"A4\"…\"A0\" or \"3xA4\" (default \"A4\"), landscape?: bool (default: false for A4, true otherwise), scale?: 1:N (default 50), at?: [x, y] lower left corner (default [0, 0]), binding?: bool, 20 mm left margin (default true), title?, place?, address?, number?, part?, surveyed?, drawn?, checked?, date?: text fields of the title block, datum?: height of ±0,00} → {width, height, handles, message}")
            .interactive(|_| Ok(Box::new(RamM::default()))),
    ]
}

/// Sheet size in mm (width, height) for a format name.
pub fn sheet(format: &str, landscape: Option<bool>) -> Option<(f64, f64, usize)> {
    let f = format.trim().to_ascii_uppercase().replace(' ', "");
    if let Some(n) = f.strip_suffix("XA4").or_else(|| f.strip_suffix("A4").filter(|n| !n.is_empty())) {
        let n: usize = n.trim_end_matches(['X', '×']).parse().ok().filter(|n| (1..=20).contains(n))?;
        return Some((210.0 * n as f64, 297.0, n));
    }
    let (short, long) = match f.as_str() {
        "A4" => (210.0, 297.0),
        "A3" => (297.0, 420.0),
        "A2" => (420.0, 594.0),
        "A1" => (594.0, 841.0),
        "A0" => (841.0, 1189.0),
        _ => return None,
    };
    let land = landscape.unwrap_or(f != "A4");
    Some(if land { (long, short, 0) } else { (short, long, 0) })
}

fn field<'a>(p: &'a Value, key: &str) -> Result<&'a str> {
    match p.get(key) {
        None | Some(Value::Null) => Ok(""),
        Some(Value::String(s)) if s.chars().count() <= 200 => Ok(s.as_str()),
        Some(_) => Err(bad("georam", format!("`{key}` must be text of at most 200 characters"))),
    }
}

fn run_georam(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "georam";
    let format = str_param(p, "format").unwrap_or("A4");
    let landscape = p.get("landscape").and_then(Value::as_bool);
    let (w_mm, h_mm, strip) =
        sheet(format, landscape).ok_or_else(|| bad(CMD, format!("unknown format \"{format}\": use A4 to A0, or 2xA4, 3xA4…")))?;
    let scale = f64_or(p, "scale", 50.0);
    if !(scale.is_finite() && (1.0..=100_000.0).contains(&scale)) {
        return Err(bad(CMD, "`scale` must be a drawing scale like 50 or 100"));
    }
    let at = match p.get("at") {
        None | Some(Value::Null) => Vec2::ZERO,
        Some(v) => point_value(v)
            .filter(|a| a.x.is_finite() && a.y.is_finite() && a.x.abs() < 1e9 && a.y.abs() < 1e9)
            .ok_or_else(|| bad(CMD, "`at` must be a point [x, y]"))?,
    };
    let datum = match p.get("datum") {
        None | Some(Value::Null) => None,
        Some(v) => Some(v.as_f64().filter(|d| d.is_finite() && d.abs() < 1e5).ok_or_else(|| bad(CMD, "`datum` must be a height"))?),
    };
    let fields: Vec<&str> =
        ["title", "place", "address", "number", "part", "surveyed", "drawn", "checked", "date"].iter().map(|k| field(p, k)).collect::<Result<_>>()?;
    let [title, place, address, number, part, surveyed, drawn, checked, date] = fields.as_slice() else {
        return Err(bad(CMD, "title block fields"));
    };
    let binding = p.get("binding").and_then(Value::as_bool).unwrap_or(true);
    // Model units per mm of paper.
    let k = scale / 1000.0;
    let space = s.space();
    ensure_layer(s, LAYER_FRAME, 7)?;
    ensure_layer(s, LAYER_TITLE, 7)?;
    let mut handles = Vec::new();
    let pt = |x: f64, y: f64| Vec2::new(at.x + x * k, at.y + y * k);
    let rect = |x0: f64, y0: f64, x1: f64, y1: f64| LwPolyline {
        vertices: [pt(x0, y0), pt(x1, y0), pt(x1, y1), pt(x0, y1)].into_iter().map(PolyVertex::new).collect(),
        closed: true,
        const_width: 0.0,
        elevation: 0.0,
        plinegen: false,
    };
    // Trimmed sheet (thin) and the drawing border (heavy).
    handles.push(add(s, &space, LAYER_FRAME, EntityKind::LwPolyline(rect(0.0, 0.0, w_mm, h_mm)))?);
    let left = if binding { 20.0 } else { 5.0 };
    let mut border = rect(left, 5.0, w_mm - 5.0, h_mm - 5.0);
    border.const_width = 0.5 * k;
    handles.push(add(s, &space, LAYER_FRAME, EntityKind::LwPolyline(border))?);
    // Fold marks every A4 width along a strip.
    let mut line = |s: &mut Session, layer: &str, a: Vec2, b: Vec2| -> Result<()> {
        handles.push(add(s, &space, layer, EntityKind::Line(Line { a: Vec3::new(a.x, a.y, 0.0), b: Vec3::new(b.x, b.y, 0.0) }))?);
        Ok(())
    };
    for i in 1..strip {
        let x = 210.0 * i as f64;
        line(s, LAYER_FRAME, pt(x, 0.0), pt(x, 5.0))?;
        line(s, LAYER_FRAME, pt(x, h_mm - 5.0), pt(x, h_mm))?;
    }
    // Title block: 110 × 36 mm in the lower right corner of the border.
    let (bw, x1, y0) = (110.0, w_mm - 5.0, 5.0);
    let x0 = x1 - bw;
    let rows = [y0, y0 + 9.0, y0 + 18.0, y0 + 27.0, y0 + 36.0];
    let tb = rect(x0, y0, x1, rows[4]);
    let mut out = Vec::new();
    out.push(add(s, &space, LAYER_TITLE, EntityKind::LwPolyline(tb))?);
    for y in &rows[1..4] {
        line(s, LAYER_TITLE, pt(x0, *y), pt(x1, *y))?;
    }
    // Top row: four columns with labels.
    let col = [x0, x0 + 27.5, x0 + 55.0, x0 + 82.5, x1];
    for x in &col[1..4] {
        line(s, LAYER_TITLE, pt(*x, rows[3]), pt(*x, rows[4]))?;
    }
    line(s, LAYER_TITLE, pt(x0, rows[3] + 4.0), pt(x1, rows[3] + 4.0))?;
    let small = 1.8 * k;
    let normal = 2.5 * k;
    let big = 3.5 * k;
    let format_text = if strip > 0 { format!("{strip} × A4") } else { format.trim().to_ascii_uppercase() };
    for (i, (label, value)) in [("ZAMĚŘIL", *surveyed), ("VYPRACOVAL", *drawn), ("KONTROLOVAL", *checked), ("FORMÁT / DATUM", "")].iter().enumerate()
    {
        let (Some(a), Some(b)) = (col.get(i), col.get(i + 1)) else { continue };
        let cx = (a + b) / 2.0;
        out.push(text(s, &space, LAYER_TITLE, pt(cx, rows[3] + 6.5), small, label.to_string(), 0.0, (HAlign::Center, VAlign::Middle))?);
        let v = if i == 3 { if date.is_empty() { format_text.clone() } else { format!("{format_text}  {date}") } } else { value.to_string() };
        if !v.is_empty() {
            out.push(text(s, &space, LAYER_TITLE, pt(cx, rows[3] + 2.0), small, v, 0.0, (HAlign::Center, VAlign::Middle))?);
        }
    }
    // Place and scale.
    let split = x1 - 32.0;
    line(s, LAYER_TITLE, pt(split, rows[0]), pt(split, rows[3]))?;
    line(s, LAYER_TITLE, pt(split + 16.0, rows[0]), pt(split + 16.0, rows[1]))?;
    if !place.is_empty() {
        out.push(text(s, &space, LAYER_TITLE, pt(x0 + 2.0, rows[2] + 4.5), big, place.to_string(), 0.0, (HAlign::Left, VAlign::Middle))?);
    }
    out.push(text(s, &space, LAYER_TITLE, pt(split + 2.0, rows[2] + 6.5), small, "MĚŘÍTKO".into(), 0.0, (HAlign::Left, VAlign::Middle))?);
    out.push(text(
        s,
        &space,
        LAYER_TITLE,
        pt(split + 16.0, rows[2] + 3.0),
        normal,
        format!("1:{}", scale.round()),
        0.0,
        (HAlign::Center, VAlign::Middle),
    )?);
    if !address.is_empty() {
        out.push(text(s, &space, LAYER_TITLE, pt(x0 + 2.0, rows[1] + 4.5), normal, address.to_string(), 0.0, (HAlign::Left, VAlign::Middle))?);
    }
    out.push(text(s, &space, LAYER_TITLE, pt(split + 2.0, rows[1] + 6.5), small, "ČÁST".into(), 0.0, (HAlign::Left, VAlign::Middle))?);
    if !part.is_empty() {
        out.push(text(s, &space, LAYER_TITLE, pt(split + 16.0, rows[1] + 3.0), normal, part.to_string(), 0.0, (HAlign::Center, VAlign::Middle))?);
    }
    if !title.is_empty() {
        out.push(text(s, &space, LAYER_TITLE, pt(x0 + 2.0, rows[0] + 4.5), normal, title.to_string(), 0.0, (HAlign::Left, VAlign::Middle))?);
    }
    out.push(text(s, &space, LAYER_TITLE, pt(split + 2.0, rows[0] + 6.5), small, "Č. VÝKR.".into(), 0.0, (HAlign::Left, VAlign::Middle))?);
    if !number.is_empty() {
        out.push(text(s, &space, LAYER_TITLE, pt(split + 24.0, rows[0] + 3.0), big, number.to_string(), 0.0, (HAlign::Center, VAlign::Middle))?);
    }
    if let Some(d) = datum {
        out.push(text(
            s,
            &space,
            LAYER_TITLE,
            pt(x1, rows[4] + 3.0),
            normal,
            format!("±0,00 = {}", format!("{d:.2}").replace('.', ",")),
            0.0,
            (HAlign::Right, VAlign::Baseline),
        )?);
    }
    handles.extend(out);
    let (w, h) = (w_mm * k, h_mm * k);
    Ok(json!({
        "width": w, "height": h,
        "handles": handles.iter().map(|h| h.hex()).collect::<Vec<_>>(),
        "message": format!("{format_text} sheet at 1:{} ({w:.2} × {h:.2} m in the drawing)", scale.round()),
    }))
}

#[derive(Default)]
struct RamM {
    format: Option<String>,
    scale: Option<f64>,
    title: Option<String>,
}

impl Interactive for RamM {
    fn name(&self) -> &'static str {
        "GEORAM"
    }
    fn prompt(&self, _s: &Session) -> Prompt {
        match (&self.format, self.scale, &self.title) {
            (None, _, _) => Prompt::new("Format (A4…A0, or 2xA4, 3xA4…)", Accept::TEXT).default("A4"),
            (_, None, _) => Prompt::new("Scale 1:", Accept::NUMBER).default("50"),
            (_, _, None) => Prompt::new("Drawing title", Accept::TEXT),
            _ => Prompt::new("Lower left corner of the sheet", Accept::POINT),
        }
    }
    fn input(&mut self, s: &mut Session, i: Input) -> Result<Step> {
        if matches!(i, Input::Cancel) {
            return Ok(Step::Cancel);
        }
        let typed = match &i {
            Input::Text(t) | Input::Keyword(t) => Some(t.trim().to_string()),
            _ => None,
        };
        if self.format.is_none() {
            let f = match (&i, typed) {
                (Input::Enter, _) => "A4".to_string(),
                (_, Some(t)) => t,
                _ => return Ok(Step::Continue),
            };
            if sheet(&f, None).is_some() {
                self.format = Some(f);
            } else {
                s.echo("Enter A4, A3, A2, A1, A0, or 2xA4, 3xA4…");
            }
            return Ok(Step::Continue);
        }
        if self.scale.is_none() {
            match (&i, typed.as_deref().and_then(typed_number)) {
                (Input::Enter, _) => self.scale = Some(50.0),
                (_, Some(v)) if v >= 1.0 => self.scale = Some(v),
                _ => s.echo("Enter the scale, like 50 for 1:50."),
            }
            return Ok(Step::Continue);
        }
        if self.title.is_none() {
            self.title = Some(typed.unwrap_or_default());
            return Ok(Step::Continue);
        }
        let Input::Point(at) = i else { return Ok(Step::Continue) };
        let r = run_georam(s, &json!({ "format": self.format, "scale": self.scale, "title": self.title, "at": [at.x, at.y], "date": "" }));
        say(s, r);
        Ok(Step::Done)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn formats() {
        assert_eq!(sheet("A4", None), Some((210.0, 297.0, 0)));
        assert_eq!(sheet("a3", None), Some((420.0, 297.0, 0)));
        assert_eq!(sheet("A3", Some(false)), Some((297.0, 420.0, 0)));
        assert_eq!(sheet("3xA4", None), Some((630.0, 297.0, 3)));
        assert_eq!(sheet("4A4", None), Some((840.0, 297.0, 4)));
        assert_eq!(sheet("2 x A4", None), Some((420.0, 297.0, 2)));
        for bad in ["A5", "0xA4", "99xA4", "", "xA4", "A4A4"] {
            assert_eq!(sheet(bad, None), None, "{bad}");
        }
    }

    #[test]
    fn a_sheet_at_scale_with_its_title_block() {
        let mut s = Session::new();
        let r = s
            .execute(
                "georam",
                &json!({ "format": "2xA4", "scale": 50, "at": [100, 0], "title": "Stavební zaměření – řez A", "place": "ŘÍČANY", "number": "4", "datum": 343.5, "surveyed": "M. Šmíd", "date": "12/2019" }),
            )
            .unwrap();
        // 420 × 297 mm at 1:50 is 21 × 14.85 m.
        assert!((r["width"].as_f64().unwrap() - 21.0).abs() < 1e-9 && (r["height"].as_f64().unwrap() - 14.85).abs() < 1e-9, "{r}");
        let d = s.doc().unwrap();
        let texts: Vec<String> =
            d.model.iter().filter_map(|e| if let EntityKind::Text(t) = &e.kind { Some(t.value.clone()) } else { None }).collect();
        for want in ["ŘÍČANY", "1:50", "4", "±0,00 = 343,50", "M. Šmíd", "2 × A4  12/2019", "Stavební zaměření – řez A"] {
            assert!(texts.iter().any(|t| t == want), "{want} missing from {texts:?}");
        }
        // Everything lies on the sheet.
        for e in d.model.iter() {
            let pts: Vec<Vec2> = match &e.kind {
                EntityKind::LwPolyline(p) => p.vertices.iter().map(|v| v.p).collect(),
                EntityKind::Line(l) => vec![l.a.xy(), l.b.xy()],
                _ => continue,
            };
            assert!(pts.iter().all(|p| p.x >= 100.0 - 1e-9 && p.x <= 121.0 + 1e-9 && p.y >= -1e-9 && p.y <= 14.85 + 1e-9), "{pts:?}");
        }
        // One fold mark top and bottom between the two A4s.
        let folds =
            d.model.iter().filter(|e| matches!(&e.kind, EntityKind::Line(l) if (l.a.x - 110.5).abs() < 1e-9 && (l.b.x - 110.5).abs() < 1e-9)).count();
        assert_eq!(folds, 2);
    }

    #[test]
    fn hostile_input_is_refused_not_panicking() {
        let mut s = Session::new();
        for p in [
            json!({ "format": "B5" }),
            json!({ "scale": 0 }),
            json!({ "scale": "x" }),
            json!({ "at": "x" }),
            json!({ "title": 5 }),
            json!({ "title": "x".repeat(500) }),
            json!({ "datum": "x" }),
        ] {
            let r = s.execute("georam", &p);
            assert!(r.is_err() || p.get("scale") == Some(&json!("x")), "{p}");
        }
    }
}
