//! Survey add-on settings (GEONASTAVENI): the drawing scale, the lettering of each kind of
//! label and how points are loaded and labelled.
//!
//! The settings live in the drawing, as the description of the empty block `GEO_NASTAVENI`
//! (JSON), so they travel with the file. Each kind of label has its own text style
//! (`GEO_CISLA` point numbers, `GEO_VYSKY` heights, `GEO_KODY` codes, `GEO_POPIS` other
//! descriptions) holding its font, width factor and slant; its height is given in millimetres
//! on paper and becomes metres in the drawing through the scale (2 mm at 1:500 is 1 m).
//!
//! A template is the same JSON in a file (`save` / `load`), for reuse across drawings.

use std::sync::Arc;

use cadcraft_doc::{Block, TextStyle};
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};

use super::{CommandSpec, bad, str_param};
use crate::{Accept, EngineError, Input, Interactive, Prompt, Result, Session, Step};

const CMD: &str = "geonastaveni";
pub const BLOCK: &str = "GEO_NASTAVENI";
pub const STYLE_NUMBERS: &str = "GEO_CISLA";
pub const STYLE_HEIGHTS: &str = "GEO_VYSKY";
pub const STYLE_CODES: &str = "GEO_KODY";
pub const STYLE_NOTES: &str = "GEO_POPIS";
/// Longest settings text read (a drawing or a template file is untrusted).
const MAX_JSON: usize = 64 << 10;

pub(super) fn specs() -> Vec<CommandSpec> {
    vec![
        CommandSpec::new(CMD, "Survey Settings...", run_geonastaveni)
            .menu(&["Geo", "Settings..."])
            .alias(&["geosettings", "-geonastaveni"])
            .params(
                "{scale?: 1:N, decimals?: height decimals 0-4, fonts?: {numbers|heights|codes|notes: {font?, mm?: height on paper, width?: width factor, slant?: degrees}}, \
                 corner?: where the number sits: \"PH\" right-up | \"PD\" right-down | \"LH\" left-up | \"LD\" left-down, \
                 digits?: heights shown \"full\" | \"tens\" | \"units\", duplicates?: \"keep\" | \"skip\" | \"renumber\" | \"replace\" | \"average\", \
                 ku?: bool (12-digit numbers shown as ZPMZ-number), z3d?: bool (points at their height), symbols?: bool (map symbols from codes on import), \
                 block?: bool (each point a GEO_BOD block with attributes), numbers?, heights?, codes?: bool (which labels), reset?: bool, save?: template path, load?: template path} → {settings, message}",
            )
            .interactive(|_| Ok(Box::new(SettingsM::default()))),
    ]
}

/// The lettering of one kind of label.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct Pismo {
    pub font: String,
    /// Height on paper, millimetres.
    pub mm: f64,
    pub width: f64,
    /// Slant, degrees.
    pub slant: f64,
}

impl Pismo {
    fn new(mm: f64) -> Self {
        Pismo { font: TextStyle::default().font, mm, width: 1.0, slant: 0.0 }
    }
}

impl Default for Pismo {
    fn default() -> Self {
        Pismo::new(2.0)
    }
}

/// Where a point's number sits; its height goes on the other row on the same side.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum Corner {
    #[default]
    #[serde(rename = "PH")]
    RightUp,
    #[serde(rename = "PD")]
    RightDown,
    #[serde(rename = "LH")]
    LeftUp,
    #[serde(rename = "LD")]
    LeftDown,
}

impl Corner {
    fn parse(t: &str) -> Option<Self> {
        Some(match t.trim().to_ascii_lowercase().replace([' ', '_'], "-").as_str() {
            "ph" | "right-up" | "rightup" | "pravo-nahore" | "vpravo-nahore" => Corner::RightUp,
            "pd" | "right-down" | "rightdown" | "pravo-dole" | "vpravo-dole" => Corner::RightDown,
            "lh" | "left-up" | "leftup" | "levo-nahore" | "vlevo-nahore" => Corner::LeftUp,
            "ld" | "left-down" | "leftdown" | "levo-dole" | "vlevo-dole" => Corner::LeftDown,
            _ => return None,
        })
    }
    fn name(self) -> &'static str {
        match self {
            Corner::RightUp => "PH",
            Corner::RightDown => "PD",
            Corner::LeftUp => "LH",
            Corner::LeftDown => "LD",
        }
    }
    pub fn left(self) -> bool {
        matches!(self, Corner::LeftUp | Corner::LeftDown)
    }
    pub fn number_below(self) -> bool {
        matches!(self, Corner::RightDown | Corner::LeftDown)
    }
}

/// How much of a height is written.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Digits {
    #[default]
    Full,
    /// The last two digits before the decimal point (245.37 → 45.37).
    Tens,
    /// The last digit before the decimal point (245.37 → 5.37).
    Units,
}

impl Digits {
    fn parse(t: &str) -> Option<Self> {
        Some(match t.trim().to_ascii_lowercase().as_str() {
            "full" | "f" | "cele" | "celé" | "plna" | "plná" => Digits::Full,
            "tens" | "t" | "desitky" | "desítky" => Digits::Tens,
            "units" | "u" | "jednotky" => Digits::Units,
            _ => return None,
        })
    }
    fn name(self) -> &'static str {
        match self {
            Digits::Full => "full",
            Digits::Tens => "tens",
            Digits::Units => "units",
        }
    }
}

/// What happens to a point whose number is already taken.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Duplicates {
    /// Load it anyway.
    #[default]
    Keep,
    /// Leave it out.
    Skip,
    /// Give it the next free number.
    Renumber,
    /// The later point replaces the earlier one (within the list).
    Replace,
    /// One point at the average of all with that number (within the list).
    Average,
}

impl Duplicates {
    fn parse(t: &str) -> Option<Self> {
        Some(match t.trim().to_ascii_lowercase().as_str() {
            "keep" | "k" | "ponechat" | "nechat" => Duplicates::Keep,
            "skip" | "s" | "preskocit" | "přeskočit" | "vynechat" => Duplicates::Skip,
            "renumber" | "r" | "precislovat" | "přečíslovat" | "nove" | "nové" => Duplicates::Renumber,
            "replace" | "p" | "prepsat" | "přepsat" | "nahradit" => Duplicates::Replace,
            "average" | "a" | "prumer" | "průměr" => Duplicates::Average,
            _ => return None,
        })
    }
    fn name(self) -> &'static str {
        match self {
            Duplicates::Keep => "keep",
            Duplicates::Skip => "skip",
            Duplicates::Renumber => "renumber",
            Duplicates::Replace => "replace",
            Duplicates::Average => "average",
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct Nastaveni {
    pub scale: f64,
    pub decimals: usize,
    pub numbers: Pismo,
    pub heights: Pismo,
    pub codes: Pismo,
    pub notes: Pismo,
    pub corner: Corner,
    pub digits: Digits,
    pub duplicates: Duplicates,
    pub ku: bool,
    pub z3d: bool,
    pub symbols: bool,
    pub block: bool,
    pub show_numbers: bool,
    pub show_heights: bool,
    pub show_codes: bool,
}

impl Default for Nastaveni {
    fn default() -> Self {
        Nastaveni {
            scale: 500.0,
            decimals: 2,
            numbers: Pismo::new(2.0),
            heights: Pismo::new(2.0),
            codes: Pismo::new(1.6),
            notes: Pismo::new(2.5),
            corner: Corner::RightUp,
            digits: Digits::Full,
            duplicates: Duplicates::Keep,
            ku: false,
            z3d: true,
            symbols: false,
            block: false,
            show_numbers: true,
            show_heights: true,
            show_codes: true,
        }
    }
}

impl Nastaveni {
    /// Bring hostile values (from a file) back into range.
    fn sanitize(mut self) -> Self {
        let d = Nastaveni::default();
        if !(self.scale.is_finite() && (1.0..=1_000_000.0).contains(&self.scale)) {
            self.scale = d.scale;
        }
        self.decimals = self.decimals.min(4);
        for (p, dp) in [(&mut self.numbers, d.numbers), (&mut self.heights, d.heights), (&mut self.codes, d.codes), (&mut self.notes, d.notes)] {
            if !(p.mm.is_finite() && (0.1..=50.0).contains(&p.mm)) {
                p.mm = dp.mm;
            }
            if !(p.width.is_finite() && (0.1..=10.0).contains(&p.width)) {
                p.width = 1.0;
            }
            if !(p.slant.is_finite() && (-60.0..=60.0).contains(&p.slant)) {
                p.slant = 0.0;
            }
            p.font = p.font.trim().chars().take(64).collect();
            if p.font.is_empty() {
                p.font = dp.font;
            }
        }
        self
    }

    /// A label's height in the drawing (metres).
    pub fn height(&self, p: &Pismo) -> f64 {
        p.mm / 1000.0 * self.scale
    }

    /// A height as the settings write it. The code "T" always gets the whole height.
    pub fn height_text(&self, z: f64, code: &str) -> String {
        let decimals = self.decimals;
        let shown = if code.trim().eq_ignore_ascii_case("T") {
            z
        } else {
            match self.digits {
                Digits::Full => z,
                Digits::Tens => z.signum() * (z.abs() % 100.0),
                Digits::Units => z.signum() * (z.abs() % 10.0),
            }
        };
        format!("{shown:.decimals$}")
    }

    fn summary(&self) -> String {
        let f = |p: &Pismo| format!("{} {} mm", p.font, p.mm);
        format!(
            "Scale 1:{}, heights {} decimals ({}), numbers {} at {}, heights {}, codes {}, duplicates: {}{}{}{}",
            self.scale,
            self.decimals,
            self.digits.name(),
            f(&self.numbers),
            self.corner.name(),
            f(&self.heights),
            f(&self.codes),
            self.duplicates.name(),
            if self.ku { ", 12-digit numbers shortened" } else { "" },
            if self.symbols { ", symbols from codes" } else { "" },
            if self.block { ", points as blocks" } else { "" },
        )
    }
}

/// The drawing's settings (the defaults when it has none).
pub fn load(s: &Session) -> Nastaveni {
    let Ok(d) = s.doc() else { return Nastaveni::default() };
    d.blocks
        .get(BLOCK)
        .map(|b| b.description.as_str())
        .filter(|t| !t.is_empty() && t.len() <= MAX_JSON)
        .and_then(|t| serde_json::from_str::<Nastaveni>(t).ok())
        .unwrap_or_default()
        .sanitize()
}

/// The drawing's scale (1:N) for labels.
pub fn scale(s: &Session) -> f64 {
    load(s).scale
}

/// Store the settings in the drawing and bring its text styles up to date.
pub fn save(s: &mut Session, n: &Nastaveni) -> Result<()> {
    let text = serde_json::to_string(n).map_err(|e| bad(CMD, e.to_string()))?;
    let d = s.doc_mut()?;
    match d.blocks.get_mut(BLOCK) {
        Some(b) => Arc::make_mut(b).description = text,
        None => {
            let mut b = Block::new(BLOCK);
            b.description = text;
            d.blocks.insert(BLOCK.into(), Arc::new(b));
        }
    }
    for (name, p) in [(STYLE_NUMBERS, &n.numbers), (STYLE_HEIGHTS, &n.heights), (STYLE_CODES, &n.codes), (STYLE_NOTES, &n.notes)] {
        let want = TextStyle { name: name.into(), font: p.font.clone(), width_factor: p.width, oblique: p.slant.to_radians(), ..Default::default() };
        match d.text_styles.iter_mut().find(|t| t.name.eq_ignore_ascii_case(name)) {
            Some(t) => {
                t.font = want.font;
                t.width_factor = want.width_factor;
                t.oblique = want.oblique;
            }
            None => d.text_styles.push(want),
        }
    }
    Ok(())
}

fn set_bool(p: &Value, key: &str, v: &mut bool) -> Result<()> {
    match p.get(key) {
        None => Ok(()),
        Some(Value::Bool(b)) => {
            *v = *b;
            Ok(())
        }
        Some(Value::String(t)) => {
            *v = match t.trim().to_ascii_lowercase().as_str() {
                "yes" | "y" | "true" | "on" | "ano" | "a" | "1" => true,
                "no" | "n" | "false" | "off" | "ne" | "0" => false,
                _ => return Err(bad(CMD, format!("`{key}` must be yes or no"))),
            };
            Ok(())
        }
        Some(_) => Err(bad(CMD, format!("`{key}` must be true or false"))),
    }
}

fn number(v: &Value) -> Option<f64> {
    match v {
        Value::Number(n) => n.as_f64(),
        Value::String(t) => t.trim().rsplit(':').next().unwrap_or("").trim().replace(',', ".").parse().ok(),
        _ => None,
    }
    .filter(|x: &f64| x.is_finite())
}

fn apply_font(p: &mut Pismo, v: &Value, which: &str) -> Result<()> {
    let Some(o) = v.as_object() else { return Err(bad(CMD, format!("`fonts.{which}` must be an object"))) };
    if let Some(f) = o.get("font").and_then(Value::as_str) {
        let f = f.trim();
        if f.is_empty() || f.chars().count() > 64 {
            return Err(bad(CMD, "a font name must be 1 to 64 characters"));
        }
        p.font = f.into();
    }
    if let Some(v) = o.get("mm") {
        p.mm = number(v).filter(|x| (0.1..=50.0).contains(x)).ok_or_else(|| bad(CMD, "a height on paper must be 0.1 to 50 mm"))?;
    }
    if let Some(v) = o.get("width") {
        p.width = number(v).filter(|x| (0.1..=10.0).contains(x)).ok_or_else(|| bad(CMD, "a width factor must be 0.1 to 10"))?;
    }
    if let Some(v) = o.get("slant") {
        p.slant = number(v).filter(|x| (-60.0..=60.0).contains(x)).ok_or_else(|| bad(CMD, "a slant must be -60 to 60 degrees"))?;
    }
    Ok(())
}

/// Apply the parameters of `p` to `n`.
pub(super) fn apply(n: &mut Nastaveni, p: &Value) -> Result<()> {
    if let Some(v) = p.get("scale") {
        n.scale = number(v).filter(|x| (1.0..=1_000_000.0).contains(x)).ok_or_else(|| bad(CMD, "`scale` must be a map scale like 500 or 1000"))?;
    }
    if let Some(v) = p.get("decimals") {
        n.decimals = number(v).filter(|x| (0.0..=4.0).contains(x)).ok_or_else(|| bad(CMD, "`decimals` must be 0 to 4"))?.round() as usize;
    }
    if let Some(f) = p.get("fonts") {
        let o = f.as_object().ok_or_else(|| bad(CMD, "`fonts` must be an object"))?;
        for (k, v) in o {
            let target = match k.as_str() {
                "numbers" | "cisla" => &mut n.numbers,
                "heights" | "vysky" => &mut n.heights,
                "codes" | "kody" => &mut n.codes,
                "notes" | "popis" => &mut n.notes,
                other => return Err(bad(CMD, format!("unknown label kind \"{other}\": use numbers, heights, codes or notes"))),
            };
            apply_font(target, v, k)?;
        }
    }
    if let Some(t) = str_param(p, "corner") {
        n.corner = Corner::parse(t).ok_or_else(|| bad(CMD, "`corner` must be PH, PD, LH or LD"))?;
    }
    if let Some(t) = str_param(p, "digits") {
        n.digits = Digits::parse(t).ok_or_else(|| bad(CMD, "`digits` must be full, tens or units"))?;
    }
    if let Some(t) = str_param(p, "duplicates") {
        n.duplicates = Duplicates::parse(t).ok_or_else(|| bad(CMD, "`duplicates` must be keep, skip, renumber, replace or average"))?;
    }
    set_bool(p, "ku", &mut n.ku)?;
    set_bool(p, "z3d", &mut n.z3d)?;
    set_bool(p, "symbols", &mut n.symbols)?;
    set_bool(p, "block", &mut n.block)?;
    set_bool(p, "numbers", &mut n.show_numbers)?;
    set_bool(p, "heights", &mut n.show_heights)?;
    set_bool(p, "codes", &mut n.show_codes)?;
    Ok(())
}

#[cfg(not(target_arch = "wasm32"))]
fn read_template(path: &str) -> Result<Nastaveni> {
    let len = std::fs::metadata(path).map_err(|e| bad(CMD, format!("{path}: {e}")))?.len();
    if len > MAX_JSON as u64 {
        return Err(bad(CMD, format!("{path} is not a settings template")));
    }
    let t = std::fs::read_to_string(path).map_err(|e| bad(CMD, format!("{path}: {e}")))?;
    serde_json::from_str::<Nastaveni>(&t).map(Nastaveni::sanitize).map_err(|e| bad(CMD, format!("{path} is not a settings template: {e}")))
}

#[cfg(target_arch = "wasm32")]
fn read_template(_path: &str) -> Result<Nastaveni> {
    Err(bad(CMD, "templates are not available on the web"))
}

#[cfg(not(target_arch = "wasm32"))]
fn write_template(path: &str, n: &Nastaveni) -> Result<()> {
    let t = serde_json::to_string_pretty(n).map_err(|e| bad(CMD, e.to_string()))?;
    std::fs::write(path, t).map_err(|e| bad(CMD, format!("{path}: {e}")))
}

#[cfg(target_arch = "wasm32")]
fn write_template(_path: &str, _n: &Nastaveni) -> Result<()> {
    Err(bad(CMD, "templates are not available on the web"))
}

fn run_geonastaveni(s: &mut Session, p: &Value) -> Result<Value> {
    let mut n = if p.get("reset").and_then(Value::as_bool).unwrap_or(false) { Nastaveni::default() } else { load(s) };
    if let Some(path) = str_param(p, "load") {
        n = read_template(path)?;
    }
    apply(&mut n, p)?;
    save(s, &n)?;
    let mut message = n.summary();
    if let Some(path) = str_param(p, "save") {
        write_template(path, &n)?;
        message = format!("Template saved to {path}. {message}");
    }
    Ok(json!({ "settings": serde_json::to_value(&n).unwrap_or(Value::Null), "message": message }))
}

/// What the settings prompt is asking for.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
enum Ask {
    #[default]
    Main,
    Scale,
    Decimals,
    Corner,
    Digits,
    Duplicates,
    Flag(&'static str),
    Template(bool),
    FontKind,
    Font(&'static str, u8),
}

#[derive(Default)]
struct SettingsM {
    ask: Ask,
    shown: bool,
}

fn yes_no(b: bool) -> &'static str {
    if b { "Yes" } else { "No" }
}

fn pismo<'a>(n: &'a Nastaveni, kind: &str) -> &'a Pismo {
    match kind {
        "heights" => &n.heights,
        "codes" => &n.codes,
        "notes" => &n.notes,
        _ => &n.numbers,
    }
}

impl Interactive for SettingsM {
    fn name(&self) -> &'static str {
        "GEONASTAVENI"
    }

    fn prompt(&self, s: &Session) -> Prompt {
        let n = load(s);
        match self.ask {
            Ask::Main => Prompt::new("Survey settings", Accept::TEXT)
                .kw(&["Scale", "Fonts", "Decimals", "Position", "Heights", "dUplicates", "Ku", "3D", "sYmbols", "Block", "saVe", "Load", "eXit"])
                .default("eXit"),
            Ask::Scale => Prompt::new("Drawing scale 1:", Accept::NUMBER).default(format!("{}", n.scale)),
            Ask::Decimals => Prompt::new("Height decimals (0-4)", Accept::NUMBER).default(n.decimals.to_string()),
            Ask::Corner => Prompt::new("Point number position", Accept::TEXT).kw(&["PH", "PD", "LH", "LD"]).default(n.corner.name()),
            Ask::Digits => Prompt::new("Heights shown", Accept::TEXT).kw(&["Full", "Tens", "Units"]).default(n.digits.name()),
            Ask::Duplicates => Prompt::new("Duplicate point numbers", Accept::TEXT)
                .kw(&["Keep", "Skip", "Renumber", "rePlace", "Average"])
                .default(n.duplicates.name()),
            Ask::Flag(key) => {
                let (msg, cur) = match key {
                    "ku" => ("Shorten 12-digit point numbers?", n.ku),
                    "z3d" => ("Points at their height (3D)?", n.z3d),
                    "symbols" => ("Draw map symbols from codes on import?", n.symbols),
                    _ => ("Insert each point as a block with attributes?", n.block),
                };
                Prompt::new(msg, Accept::TEXT).kw(&["Yes", "No"]).default(yes_no(cur))
            }
            Ask::Template(saving) => {
                Prompt::new(if saving { "Save settings template to file" } else { "Load settings template from file" }, Accept::TEXT)
            }
            Ask::FontKind => Prompt::new("Lettering of", Accept::TEXT).kw(&["Numbers", "Heights", "Codes", "nOtes"]).default("Numbers"),
            Ask::Font(kind, step) => {
                let p = pismo(&n, kind);
                match step {
                    0 => Prompt::new("Font name", Accept::TEXT).default(p.font.clone()),
                    1 => Prompt::new("Text height on paper (mm)", Accept::NUMBER).default(format!("{}", p.mm)),
                    2 => Prompt::new("Width factor", Accept::NUMBER).default(format!("{}", p.width)),
                    _ => Prompt::new("Slant angle (degrees)", Accept::NUMBER).default(format!("{}", p.slant)),
                }
            }
        }
    }

    fn input(&mut self, s: &mut Session, i: Input) -> Result<Step> {
        if !self.shown {
            self.shown = true;
            let n = load(s);
            s.echo(n.summary());
        }
        let t = match &i {
            Input::Cancel => return Ok(Step::Cancel),
            Input::Text(t) | Input::Keyword(t) => t.trim().trim_matches('"').to_string(),
            Input::Enter => String::new(),
            _ => return Ok(Step::Continue),
        };
        let prompt = self.prompt(s);
        let kw = prompt.match_keyword(&t);
        let apply_json = |s: &mut Session, p: Value| -> bool {
            match run_geonastaveni(s, &p) {
                Ok(r) => {
                    if let Some(m) = r.get("message").and_then(Value::as_str) {
                        s.echo(m.to_string());
                    }
                    true
                }
                Err(e) => {
                    s.echo(match e {
                        EngineError::BadParams { msg, .. } => msg,
                        other => other.to_string(),
                    });
                    false
                }
            }
        };
        match self.ask {
            Ask::Main => {
                let Some(k) = kw else {
                    if t.is_empty() {
                        return Ok(Step::Done);
                    }
                    s.echo("Choose one of the options.");
                    return Ok(Step::Continue);
                };
                self.ask = match k.as_str() {
                    "Scale" => Ask::Scale,
                    "Fonts" => Ask::FontKind,
                    "Decimals" => Ask::Decimals,
                    "Position" => Ask::Corner,
                    "Heights" => Ask::Digits,
                    "dUplicates" => Ask::Duplicates,
                    "Ku" => Ask::Flag("ku"),
                    "3D" => Ask::Flag("z3d"),
                    "sYmbols" => Ask::Flag("symbols"),
                    "Block" => Ask::Flag("block"),
                    "saVe" => Ask::Template(true),
                    "Load" => Ask::Template(false),
                    _ => return Ok(Step::Done),
                };
            }
            Ask::Scale | Ask::Decimals => {
                if !t.is_empty() {
                    let key = if self.ask == Ask::Scale { "scale" } else { "decimals" };
                    if !apply_json(s, json!({ key: t })) {
                        return Ok(Step::Continue);
                    }
                }
                self.ask = Ask::Main;
            }
            Ask::Corner | Ask::Digits | Ask::Duplicates => {
                if !t.is_empty() {
                    let key = match self.ask {
                        Ask::Corner => "corner",
                        Ask::Digits => "digits",
                        _ => "duplicates",
                    };
                    if !apply_json(s, json!({ key: kw.unwrap_or(t) })) {
                        return Ok(Step::Continue);
                    }
                }
                self.ask = Ask::Main;
            }
            Ask::Flag(key) => {
                if !t.is_empty() {
                    let v = kw.as_deref() == Some("Yes");
                    if kw.is_none() {
                        s.echo("Answer Yes or No.");
                        return Ok(Step::Continue);
                    }
                    apply_json(s, json!({ key: v }));
                }
                self.ask = Ask::Main;
            }
            Ask::Template(saving) => {
                if !t.is_empty() {
                    apply_json(s, if saving { json!({ "save": t }) } else { json!({ "load": t }) });
                }
                self.ask = Ask::Main;
            }
            Ask::FontKind => {
                let k = kw.unwrap_or_else(|| "Numbers".into());
                self.ask = Ask::Font(
                    match k.as_str() {
                        "Heights" => "heights",
                        "Codes" => "codes",
                        "nOtes" => "notes",
                        _ => "numbers",
                    },
                    0,
                );
            }
            Ask::Font(kind, step) => {
                if !t.is_empty() {
                    let field = match step {
                        0 => "font",
                        1 => "mm",
                        2 => "width",
                        _ => "slant",
                    };
                    if !apply_json(s, json!({ "fonts": { kind: { field: t } } })) {
                        return Ok(Step::Continue);
                    }
                }
                self.ask = if step >= 3 { Ask::Main } else { Ask::Font(kind, step + 1) };
            }
        }
        Ok(Step::Continue)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn settings_travel_with_the_drawing() {
        let mut s = Session::new();
        s.execute(CMD, &json!({ "scale": 1000, "corner": "LD", "fonts": { "numbers": { "font": "Arial", "mm": 3, "width": 0.8, "slant": 15 } } }))
            .unwrap();
        let n = load(&s);
        assert_eq!(n.scale, 1000.0);
        assert_eq!(n.corner, Corner::LeftDown);
        assert_eq!(n.numbers.font, "Arial");
        assert!((n.height(&n.numbers) - 3.0).abs() < 1e-9);
        let st = s.doc().unwrap().text_style(STYLE_NUMBERS).cloned().unwrap();
        assert_eq!(st.font, "Arial");
        assert!((st.width_factor - 0.8).abs() < 1e-9);
        assert!((st.oblique - 15f64.to_radians()).abs() < 1e-9);
        // Through a DXF file and back.
        let text = cadcraft_io::write_dxf(s.doc().unwrap());
        let back = cadcraft_io::read_dxf(text.as_bytes()).unwrap();
        let desc = &back.blocks.get(BLOCK).unwrap().description;
        let n2: Nastaveni = serde_json::from_str(desc).unwrap();
        assert_eq!(n2.sanitize(), n);
    }

    #[test]
    fn heights_written_in_part() {
        let mut n = Nastaveni { digits: Digits::Tens, ..Default::default() };
        assert_eq!(n.height_text(245.374, ""), "45.37");
        assert_eq!(n.height_text(245.374, "T"), "245.37");
        n.digits = Digits::Units;
        assert_eq!(n.height_text(245.374, "1"), "5.37");
    }

    #[test]
    fn the_prompt_sets_a_font() {
        let mut s = Session::new();
        s.start(CMD).unwrap();
        for t in ["F", "C", "Courier", "1.5", "", "", ""] {
            s.input(Input::Text(t.into())).unwrap();
        }
        let n = load(&s);
        assert_eq!(n.codes.font, "Courier");
        assert_eq!(n.codes.mm, 1.5);
    }

    #[test]
    fn hostile_settings_are_refused_not_panicking() {
        let mut s = Session::new();
        for p in [
            json!({ "scale": -1 }),
            json!({ "scale": "abc" }),
            json!({ "decimals": 99 }),
            json!({ "fonts": { "x": {} } }),
            json!({ "fonts": { "numbers": { "mm": 1e300 } } }),
            json!({ "fonts": 5 }),
            json!({ "corner": "zz" }),
            json!({ "ku": 7 }),
            json!({ "load": "/nonexistent/geo-template.json" }),
        ] {
            assert!(s.execute(CMD, &p).is_err(), "{p}");
        }
        // A broken block description falls back to the defaults.
        let d = s.doc_mut().unwrap();
        let mut b = Block::new(BLOCK);
        b.description = "{\"scale\": 1e999, \"numbers\": {\"mm\": -4}}".into();
        d.blocks.insert(BLOCK.into(), Arc::new(b));
        assert_eq!(load(&s).scale, 500.0);
    }
}
