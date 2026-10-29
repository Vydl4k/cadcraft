//! PDF → Drawing: the vector content of one PDF page as editable objects (PDFIMPORT).
//!
//! The page is run through craft-pdf's guarded interpreter with a device that keeps what a
//! drawing can use: stroked paths become lines, polylines and (for larger curves) splines on
//! layers named after their colour (`PDF_RRGGBB`); filled areas become closed outlines on
//! `PDF_Fill_RRGGBB` (white fills, which only cover paper, are left out unless asked for);
//! dashed strokes get a `PDF_Dash<n>` linetype of the dash pattern at the import scale; text
//! whose characters the PDF names (through its fonts or a `ToUnicode` map) becomes TEXT on
//! `PDF_Text`, and text whose characters it doesn't name is drawn as glyph outlines instead.
//!
//! Coordinates are the page's in points (1/72 in, `/UserUnit` applied, `/Rotate` and the crop box
//! honoured) with the lower-left corner at the origin and Y up, times
//! [`PdfImport::units_per_point`]: 25.4/72 gives the sheet in millimetres; 25.4/72 × 50 / 1000 a
//! 1:50 sheet in metres.
//!
//! Left out (named in the drawing's read note): raster images, shadings, and glyphs of Type 3
//! fonts. Clipping is not applied: everything the page draws is imported whole.

use std::collections::{BTreeMap, HashMap};

use cadcraft_color::{Color, Rgb};
use cadcraft_doc::{Common, DashElement, Drawing, EntityKind, HAlign, Layer, Line, Linetype, LwPolyline, Space, Text, VAlign};
use cadcraft_geom::{PolyVertex, Spline, Vec2, Vec3};
use craft_pdf::interpret::font::Glyph;
use craft_pdf::interpret::{
    BlendMode, ClipPath, Context, Device, GlyphDrawMode, Image, InterpreterCache, InterpreterSettings, Paint, PathDrawMode, SoftMask, TransformExt,
};
use kurbo::{Affine, BezPath, PathEl, Point, Shape};

use crate::{IoError, Result};

/// Largest PDF accepted.
const MAX_INPUT: usize = 256 << 20;
/// Objects created from one page.
const MAX_OBJECTS: usize = 2_000_000;
/// Vertices kept across the whole page (flattened curves and glyph outlines included).
const MAX_VERTS: usize = 8_000_000;
/// Colour layers and dash linetypes created.
const MAX_LAYERS: usize = 512;
const MAX_LINETYPES: usize = 256;
/// Characters in one TEXT.
const MAX_TEXT_CHARS: usize = 2_000;
/// A curve whose control polygon is longer than this many points becomes a spline; shorter
/// ones are flattened into the polyline they belong to.
const SPLINE_MIN_PT: f64 = 6.0;

/// How a PDF page is imported.
#[derive(Clone, Debug, PartialEq)]
pub struct PdfImport {
    /// The page, from 1.
    pub page: usize,
    /// Drawing units per PDF point (1/72 in on the sheet).
    pub units_per_point: f64,
    /// Import filled areas as closed outlines.
    pub fills: bool,
    /// Import white fills too (they only cover the paper).
    pub white_fills: bool,
    /// Import text as TEXT where the PDF names its characters (else as glyph outlines).
    pub text: bool,
}

impl Default for PdfImport {
    fn default() -> Self {
        PdfImport { page: 1, units_per_point: 25.4 / 72.0, fills: true, white_fills: false, text: true }
    }
}

fn err(msg: impl Into<String>) -> IoError {
    IoError::Format(format!("PDF: {}", msg.into()))
}

fn open(bytes: &[u8]) -> Result<craft_pdf::syntax::Pdf> {
    if bytes.len() > MAX_INPUT {
        return Err(err(format!("the file is larger than {} MB", MAX_INPUT >> 20)));
    }
    craft_pdf::open_slice(bytes, None).map_err(|e| match e {
        craft_pdf::OpenError::Load(craft_pdf::syntax::LoadPdfError::Decryption(_)) => err("the file is password-protected"),
        other => err(other.to_string()),
    })
}

/// How many pages the PDF has.
pub fn page_count(bytes: &[u8]) -> Result<usize> {
    Ok(open(bytes)?.pages().len())
}

/// Read one page as a drawing (model space, the page's lower-left corner at the origin).
pub fn import(bytes: &[u8], opts: &PdfImport) -> Result<Drawing> {
    let k = opts.units_per_point;
    if !k.is_finite() || k <= 0.0 || k > 1.0e6 {
        return Err(err("the scale must be a positive number"));
    }
    let pdf = open(bytes)?;
    let pages = pdf.pages();
    let n = pages.len();
    let page = opts.page.checked_sub(1).and_then(|i| pages.get(i)).ok_or_else(|| {
        err(if n == 1 { format!("page {} doesn't exist: the file has 1 page", opts.page) } else { format!("page {} doesn't exist: the file has {n} pages", opts.page) })
    })?;
    let (w, h) = page.render_dimensions();
    let initial = page.initial_transform(false).to_kurbo();
    let cache = InterpreterCache::new();
    let mut ctx = Context::new(initial, kurbo::Rect::new(0.0, 0.0, f64::from(w), f64::from(h)), &cache, pdf.xref(), InterpreterSettings::default());
    let mut dev = Collector::new(opts.clone());
    craft_pdf::interpret_page(page, &mut ctx, &mut dev).map_err(|e| err(e.to_string()))?;
    dev.flush_text();
    if let Some(e) = dev.error.take() {
        return Err(e);
    }
    let mut d = dev.drawing;
    let mut left = Vec::new();
    if dev.images > 0 {
        left.push(if dev.images == 1 { "1 raster image".to_string() } else { format!("{} raster images", dev.images) });
    }
    if dev.type3 > 0 {
        left.push(format!("{} characters of Type 3 fonts", dev.type3));
    }
    if !left.is_empty() {
        let scanned = dev.objects == 0 && dev.images > 0;
        d.read_notes.push(if scanned {
            "PDF: this page is a scanned image with no lines in it, so there is nothing to import as objects".to_string()
        } else {
            format!("PDF: CADCraft doesn't import these yet, so they were left out: {}", left.join(", "))
        });
    }
    Ok(d)
}

/// What a stroke, fill or text is drawn with.
#[derive(Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
enum Kind {
    Stroke,
    Fill,
}

/// The text being collected: consecutive glyphs on one baseline.
struct Run {
    origin: Point,
    /// Unit vector along the baseline, and the font size, in page points.
    dir: (f64, f64),
    size: f64,
    /// Where the last glyph ends along the baseline (from `origin`).
    end: f64,
    text: String,
    layer: String,
}

struct Collector {
    opts: PdfImport,
    drawing: Drawing,
    layers: HashMap<(Kind, [u8; 3]), String>,
    dashes: BTreeMap<String, String>,
    run: Option<Run>,
    objects: usize,
    verts_left: usize,
    images: usize,
    type3: usize,
    error: Option<IoError>,
}

fn rgb(paint: &Paint<'_>) -> [u8; 3] {
    match paint {
        Paint::Color(c) => {
            let [r, g, b, _] = c.to_rgba().to_rgba8();
            [r, g, b]
        }
        // Patterns and shadings: drawn as black lines.
        Paint::Pattern(_) => [0, 0, 0],
    }
}

fn is_white(c: [u8; 3]) -> bool {
    c.iter().all(|v| *v >= 250)
}

/// How much `t` scales lengths (the square root of its area factor).
fn scale_of(t: Affine) -> f64 {
    let [a, b, c, d, _, _] = t.as_coeffs();
    (a * d - b * c).abs().sqrt()
}

/// Segments to flatten a cubic of control-polygon length `len` (points) into.
fn segments(len: f64) -> usize {
    if len.is_finite() { ((len / 1.5).ceil() as usize).clamp(2, 48) } else { 2 }
}

fn cubic_at(p0: Point, p1: Point, p2: Point, p3: Point, t: f64) -> Point {
    let u = 1.0 - t;
    let (a, b, c, d) = (u * u * u, 3.0 * u * u * t, 3.0 * u * t * t, t * t * t);
    Point::new(a * p0.x + b * p1.x + c * p2.x + d * p3.x, a * p0.y + b * p1.y + c * p2.y + d * p3.y)
}

fn poly_len(pts: &[Point]) -> f64 {
    pts.windows(2).map(|w| match w {
        [a, b] => a.distance(*b),
        _ => 0.0,
    }).sum()
}

impl Collector {
    fn new(opts: PdfImport) -> Self {
        Collector {
            opts,
            drawing: Drawing::new_metric(),
            layers: HashMap::new(),
            dashes: BTreeMap::new(),
            run: None,
            objects: 0,
            verts_left: MAX_VERTS,
            images: 0,
            type3: 0,
            error: None,
        }
    }

    fn fail(&mut self, msg: &str) {
        if self.error.is_none() {
            self.error = Some(err(msg));
        }
    }

    fn v(&self, p: Point) -> Vec2 {
        Vec2::new(p.x * self.opts.units_per_point, p.y * self.opts.units_per_point)
    }

    /// The layer for paths of this kind and colour, created the first time.
    fn layer(&mut self, kind: Kind, c: [u8; 3]) -> String {
        if let Some(l) = self.layers.get(&(kind, c)) {
            return l.clone();
        }
        if self.layers.len() >= MAX_LAYERS {
            return "PDF_Other".into();
        }
        let hex = format!("{:02X}{:02X}{:02X}", c[0], c[1], c[2]);
        let name = match kind {
            Kind::Stroke => format!("PDF_{hex}"),
            Kind::Fill => format!("PDF_Fill_{hex}"),
        };
        self.add_layer(&name, c);
        self.layers.insert((kind, c), name.clone());
        name
    }

    fn add_layer(&mut self, name: &str, c: [u8; 3]) {
        if self.drawing.layers.iter().any(|l| l.name.eq_ignore_ascii_case(name)) {
            return;
        }
        // Black and white become colour 7, which a CAD program draws in the contrast colour of
        // its background.
        let color = if c.iter().all(|v| *v <= 24) || is_white(c) { Color::Index(7) } else { Color::True(Rgb(c[0], c[1], c[2])) };
        self.drawing.layers.push(Layer { name: name.to_string(), color, ..Layer::default() });
    }

    /// The linetype for a dash pattern (in page points), created the first time; `ByLayer` for
    /// solid strokes.
    fn linetype(&mut self, dash: &[f32], scale: f64) -> String {
        let mut pattern: Vec<f64> = dash.iter().map(|d| f64::from(d.abs()) * scale * self.opts.units_per_point).filter(|d| d.is_finite()).collect();
        if pattern.is_empty() || pattern.iter().sum::<f64>() <= 1e-12 {
            return "ByLayer".into();
        }
        if pattern.len() % 2 == 1 {
            pattern.extend_from_within(..);
        }
        let key = pattern.iter().map(|d| format!("{d:.6}")).collect::<Vec<_>>().join(",");
        if let Some(n) = self.dashes.get(&key) {
            return n.clone();
        }
        if self.dashes.len() >= MAX_LINETYPES {
            return "ByLayer".into();
        }
        let name = format!("PDF_Dash{}", self.dashes.len() + 1);
        let elements = pattern.iter().enumerate().map(|(i, d)| DashElement::dash(if i % 2 == 0 { *d } else { -*d })).collect();
        self.drawing.linetypes.push(Linetype { name: name.clone(), description: "Dashes from PDF".into(), pattern: elements });
        self.dashes.insert(key, name.clone());
        name
    }

    fn add(&mut self, layer: &str, linetype: &str, kind: EntityKind) {
        let verts = match &kind {
            EntityKind::LwPolyline(p) => p.vertices.len(),
            EntityKind::Spline(s) => s.control.len(),
            _ => 2,
        };
        self.objects += 1;
        match self.verts_left.checked_sub(verts) {
            Some(v) if self.objects <= MAX_OBJECTS => self.verts_left = v,
            _ => return self.fail("the page has more geometry than CADCraft imports at once"),
        }
        let common = Common { layer: layer.to_string(), linetype: linetype.to_string(), ..Common::default() };
        if self.drawing.add(&Space::Model, common, kind).is_err() {
            self.fail("an object could not be added");
        }
    }

    /// A run of points as a line (two points) or polyline.
    fn polyline(&mut self, layer: &str, lt: &str, pts: &mut Vec<Point>, closed: bool) {
        let mut v: Vec<Vec2> = pts.drain(..).map(|p| self.v(p)).collect();
        v.dedup_by(|a, b| (a.x - b.x).abs() < 1e-12 && (a.y - b.y).abs() < 1e-12);
        if closed
            && v.len() > 2
            && let (Some(f), Some(l)) = (v.first(), v.last())
            && (f.x - l.x).abs() < 1e-12
            && (f.y - l.y).abs() < 1e-12
        {
            v.pop();
        }
        match v.as_slice() {
            [] | [_] => {}
            [a, b] if !closed => self.add(layer, lt, EntityKind::Line(Line { a: Vec3::new(a.x, a.y, 0.0), b: Vec3::new(b.x, b.y, 0.0) })),
            _ => {
                let vertices = v.into_iter().map(PolyVertex::new).collect();
                self.add(layer, lt, EntityKind::LwPolyline(LwPolyline { vertices, closed, const_width: 0.0, elevation: 0.0, plinegen: false }));
            }
        }
    }

    /// Add `path` (already in page points) on `layer`. Strokes keep large curves as splines;
    /// outlines (fills, glyphs) are flattened into closed polylines.
    fn emit(&mut self, path: &BezPath, layer: &str, lt: &str, outline: bool) {
        let mut run: Vec<Point> = Vec::new();
        let mut start = Point::ZERO;
        let mut cur = Point::ZERO;
        for el in path.elements() {
            if self.error.is_some() {
                return;
            }
            match *el {
                PathEl::MoveTo(p) => {
                    self.polyline(layer, lt, &mut run, false);
                    run.push(p);
                    start = p;
                    cur = p;
                }
                PathEl::LineTo(p) => {
                    if run.is_empty() {
                        run.push(cur);
                    }
                    run.push(p);
                    cur = p;
                }
                PathEl::QuadTo(q, p) => {
                    let c1 = cur + (q - cur) * (2.0 / 3.0);
                    let c2 = p + (q - p) * (2.0 / 3.0);
                    self.curve(layer, lt, &mut run, [cur, c1, c2, p], outline);
                    cur = p;
                }
                PathEl::CurveTo(c1, c2, p) => {
                    self.curve(layer, lt, &mut run, [cur, c1, c2, p], outline);
                    cur = p;
                }
                PathEl::ClosePath => {
                    if !run.is_empty() {
                        if run.last().is_some_and(|l| l.distance(start) > 1e-9) && run.len() < 2 {
                            run.push(start);
                        }
                        self.polyline(layer, lt, &mut run, true);
                    } else if cur.distance(start) > 1e-9 {
                        // A closed shape that ended on a spline: close it with a line.
                        let mut seg = vec![cur, start];
                        self.polyline(layer, lt, &mut seg, false);
                    }
                    cur = start;
                }
            }
        }
        self.polyline(layer, lt, &mut run, false);
    }

    fn curve(&mut self, layer: &str, lt: &str, run: &mut Vec<Point>, c: [Point; 4], outline: bool) {
        let [p0, p1, p2, p3] = c;
        let len = poly_len(&c);
        if !outline && len > SPLINE_MIN_PT {
            // A Bézier segment is exactly a clamped cubic spline with these four control points.
            self.polyline(layer, lt, run, false);
            let control = c.iter().map(|p| self.v(*p)).collect();
            self.add(layer, lt, EntityKind::Spline(Spline::from_control(control, 3)));
            return;
        }
        if run.is_empty() {
            run.push(p0);
        }
        let n = segments(len);
        for i in 1..=n {
            run.push(cubic_at(p0, p1, p2, p3, i as f64 / n as f64));
        }
    }

    // --- text ---------------------------------------------------------------

    fn flush_text(&mut self) {
        let Some(r) = self.run.take() else { return };
        let value = r.text.trim_end().to_string();
        if value.trim().is_empty() {
            return;
        }
        let k = self.opts.units_per_point;
        let height = r.size * 0.7 * k;
        if !height.is_finite() || height <= 0.0 {
            return;
        }
        let at = Vec3::new(r.origin.x * k, r.origin.y * k, 0.0);
        let rotation = r.dir.1.atan2(r.dir.0);
        let text = Text {
            insert: at,
            align_pt: None,
            height,
            value,
            rotation,
            width_factor: 1.0,
            oblique: 0.0,
            style: "Standard".into(),
            halign: HAlign::Left,
            valign: VAlign::Baseline,
        };
        let layer = r.layer;
        self.add(&layer, "ByLayer", EntityKind::Text(text));
    }

    /// One glyph with known characters: continue the current run or start a new one.
    fn text_glyph(&mut self, s: &str, full: Affine, width: f64, layer: String) {
        let origin = full * Point::ZERO;
        let x = full * Point::new(1.0, 0.0) - origin;
        let y = full * Point::new(0.0, 1.0) - origin;
        // Glyph space has 1000 units to the em (the interpreter normalises every outline font).
        let size = y.hypot() * 1000.0;
        let along_len = x.hypot();
        if !(size.is_finite() && size > 1e-6 && along_len.is_finite() && along_len > 1e-9) {
            return;
        }
        let dir = (x.x / along_len, x.y / along_len);
        let glyph_end = width * along_len;
        if let Some(r) = &mut self.run {
            let dx = origin.x - r.origin.x;
            let dy = origin.y - r.origin.y;
            let along = dx * r.dir.0 + dy * r.dir.1;
            let perp = -dx * r.dir.1 + dy * r.dir.0;
            let same = (size / r.size - 1.0).abs() < 0.15
                && (dir.0 * r.dir.0 + dir.1 * r.dir.1) > 0.999
                && perp.abs() < 0.3 * r.size
                && r.layer == layer
                && along > r.end - 0.5 * r.size
                && along < r.end + 2.5 * r.size
                && r.text.chars().count() < MAX_TEXT_CHARS;
            if same {
                if along - r.end > 0.25 * r.size && !r.text.ends_with(' ') && !s.starts_with(' ') {
                    r.text.push(' ');
                }
                r.text.push_str(s);
                r.end = r.end.max(along + glyph_end.max(0.25 * size));
                return;
            }
        }
        self.flush_text();
        self.run = Some(Run { origin, dir, size, end: glyph_end.max(0.25 * size), text: s.to_string(), layer });
    }
}

impl<'a> Device<'a> for Collector {
    fn set_soft_mask(&mut self, _mask: Option<SoftMask<'a>>) {}
    fn set_blend_mode(&mut self, _blend_mode: BlendMode) {}

    fn draw_path(&mut self, path: &BezPath, transform: Affine, paint: &Paint<'a>, draw_mode: &PathDrawMode) {
        if self.error.is_some() {
            return;
        }
        let c = rgb(paint);
        let page = transform * path.clone();
        match draw_mode {
            PathDrawMode::Stroke(props) => {
                let layer = self.layer(Kind::Stroke, c);
                let lt = self.linetype(&props.dash_array, scale_of(transform));
                self.emit(&page, &layer, &lt, false);
            }
            PathDrawMode::Fill(_) => {
                if !self.opts.fills || (is_white(c) && !self.opts.white_fills) {
                    return;
                }
                let layer = self.layer(Kind::Fill, c);
                self.emit(&page, &layer, "ByLayer", true);
            }
        }
    }

    fn push_clip_path(&mut self, _clip_path: &ClipPath) {}
    fn push_transparency_group(&mut self, _opacity: f32, _mask: Option<SoftMask<'a>>, _blend_mode: BlendMode) {}

    fn draw_glyph(&mut self, glyph: &Glyph<'a>, transform: Affine, glyph_transform: Affine, paint: &Paint<'a>, draw_mode: &GlyphDrawMode) {
        if self.error.is_some() || matches!(draw_mode, GlyphDrawMode::Invisible | GlyphDrawMode::Clip) {
            return;
        }
        let full = transform * glyph_transform;
        let outline = match glyph {
            Glyph::Outline(g) => g.outline(),
            Glyph::Type3(_) => {
                self.type3 += 1;
                return;
            }
        };
        let c = rgb(paint);
        if self.opts.text
            && let Some(u) = glyph.as_unicode()
        {
            let s = match u {
                craft_pdf::interpret::hayro_cmap::BfString::Char(ch) => ch.to_string(),
                craft_pdf::interpret::hayro_cmap::BfString::String(s) => s,
            };
            if !s.is_empty() && !s.chars().any(char::is_control) {
                let width = outline.bounding_box().x1.max(0.0);
                self.add_layer("PDF_Text", [0, 0, 0]);
                self.text_glyph(&s, full, width, "PDF_Text".into());
                return;
            }
        }
        let path = full * outline;
        if path.elements().is_empty() {
            return;
        }
        self.flush_text();
        let layer = self.layer(Kind::Fill, c);
        self.emit(&path, &layer, "ByLayer", true);
    }

    fn draw_image(&mut self, _image: Image<'a, '_>, _transform: Affine) {
        self.images += 1;
    }

    fn pop_clip_path(&mut self) {}
    fn pop_transparency_group(&mut self) {}
}

/// The page count and the size of page 1 in points, for PDFIMPORT's prompts.
pub fn describe(bytes: &[u8]) -> Result<(usize, f64, f64)> {
    let pdf = open(bytes)?;
    let pages = pdf.pages();
    let (w, h) = pages.first().map(|p| p.render_dimensions()).unwrap_or((0.0, 0.0));
    Ok((pages.len(), f64::from(w), f64::from(h)))
}


#[cfg(test)]
mod tests {
    use super::*;
    use cadcraft_doc::Entity;

    /// A one-page PDF with `content` as its page content stream (uncompressed) and Helvetica as
    /// font /F1; `page_extra` goes into the page dictionary (e.g. `/Rotate 90`).
    fn pdf(content: &str, page_extra: &str) -> Vec<u8> {
        let objs = [
            "<< /Type /Catalog /Pages 2 0 R >>".to_string(),
            "<< /Type /Pages /Kids [3 0 R] /Count 1 >>".to_string(),
            format!("<< /Type /Page /Parent 2 0 R /MediaBox [0 0 595 842] /Resources << /Font << /F1 5 0 R >> >> /Contents 4 0 R {page_extra} >>"),
            format!("<< /Length {} >>\nstream\n{content}\nendstream", content.len()),
            "<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica /Encoding /WinAnsiEncoding >>".to_string(),
        ];
        let mut out = b"%PDF-1.7\n".to_vec();
        let mut offsets = Vec::new();
        for (i, o) in objs.iter().enumerate() {
            offsets.push(out.len());
            out.extend_from_slice(format!("{} 0 obj\n{o}\nendobj\n", i + 1).as_bytes());
        }
        let xref = out.len();
        out.extend_from_slice(format!("xref\n0 {}\n0000000000 65535 f \n", objs.len() + 1).as_bytes());
        for o in offsets {
            out.extend_from_slice(format!("{o:010} 00000 n \n").as_bytes());
        }
        out.extend_from_slice(format!("trailer\n<< /Size {} /Root 1 0 R >>\nstartxref\n{xref}\n%%EOF\n", objs.len() + 1).as_bytes());
        out
    }

    fn at_scale(units_per_point: f64) -> PdfImport {
        PdfImport { units_per_point, ..PdfImport::default() }
    }

    fn model(d: &Drawing) -> Vec<&Entity> {
        d.model.iter().collect()
    }

    #[test]
    fn a_stroked_line_comes_in_at_scale() {
        let bytes = pdf("1 0 0 RG 72 100 m 144 100 l S", "");
        let d = import(&bytes, &at_scale(1.0)).unwrap();
        let ents = model(&d);
        assert_eq!(ents.len(), 1);
        let EntityKind::Line(l) = &ents[0].kind else { panic!("{:?}", ents[0].kind) };
        assert!((l.a.x - 72.0).abs() < 1e-6 && (l.a.y - 100.0).abs() < 1e-6, "{l:?}");
        assert!((l.b.x - 144.0).abs() < 1e-6, "{l:?}");
        assert_eq!(ents[0].common.layer, "PDF_FF0000");
        let layer = d.layers.iter().find(|l| l.name == "PDF_FF0000").unwrap();
        assert_eq!(layer.color, Color::True(Rgb(255, 0, 0)));

        // 1:50 in metres: one inch on the sheet is 1.27 m.
        let d = import(&bytes, &at_scale(25.4 / 72.0 * 50.0 / 1000.0)).unwrap();
        let EntityKind::Line(l) = &model(&d)[0].kind else { panic!() };
        assert!((l.b.x - l.a.x - 1.27).abs() < 1e-9, "{l:?}");
    }

    #[test]
    fn black_layers_take_colour_7() {
        let d = import(&pdf("0 G 0 0 m 10 10 l S", ""), &PdfImport::default()).unwrap();
        assert_eq!(d.layers.iter().find(|l| l.name == "PDF_000000").unwrap().color, Color::Index(7));
    }

    #[test]
    fn rectangles_are_closed_polylines_and_curves_splines() {
        let d = import(&pdf("10 10 100 50 re S 0 0 m 0 100 100 100 100 0 c S", ""), &at_scale(1.0)).unwrap();
        let kinds: Vec<_> = model(&d).iter().map(|e| e.kind.clone()).collect();
        assert!(matches!(&kinds[0], EntityKind::LwPolyline(p) if p.closed && p.vertices.len() == 4), "{kinds:?}");
        assert!(matches!(&kinds[1], EntityKind::Spline(s) if s.control.len() == 4), "{kinds:?}");
    }

    #[test]
    fn dashes_become_a_linetype_at_scale() {
        let d = import(&pdf("[6 3] 0 d 0 0 m 100 0 l S", ""), &at_scale(2.0)).unwrap();
        let e = model(&d)[0];
        assert_eq!(e.common.linetype, "PDF_Dash1");
        let lt = d.linetypes.iter().find(|l| l.name == "PDF_Dash1").unwrap();
        let lens: Vec<f64> = lt.pattern.iter().map(|p| p.length).collect();
        assert_eq!(lens, vec![12.0, -6.0]);
    }

    #[test]
    fn white_fills_are_left_out_unless_asked_for() {
        let bytes = pdf("1 g 0 0 100 100 re f 0.5 g 10 10 20 20 re f", "");
        let d = import(&bytes, &at_scale(1.0)).unwrap();
        assert_eq!(model(&d).len(), 1);
        assert_eq!(model(&d)[0].common.layer, "PDF_Fill_808080");
        let d = import(&bytes, &PdfImport { white_fills: true, ..at_scale(1.0) }).unwrap();
        assert_eq!(model(&d).len(), 2);
        let d = import(&bytes, &PdfImport { fills: false, ..at_scale(1.0) }).unwrap();
        assert!(model(&d).is_empty());
    }

    #[test]
    fn text_becomes_text_on_its_baseline() {
        let d = import(&pdf("BT /F1 20 Tf 100 200 Td (Hello world) Tj ET", ""), &at_scale(1.0)).unwrap();
        let texts: Vec<_> = model(&d).iter().filter_map(|e| if let EntityKind::Text(t) = &e.kind { Some(t.clone()) } else { None }).collect();
        assert_eq!(texts.len(), 1, "{texts:?}");
        let t = &texts[0];
        assert_eq!(t.value, "Hello world");
        assert!((t.insert.x - 100.0).abs() < 1e-6 && (t.insert.y - 200.0).abs() < 1e-6, "{t:?}");
        assert!((t.height - 14.0).abs() < 1e-6, "{t:?}");
        assert!(t.rotation.abs() < 1e-9);

        // Without text, the glyphs are outlines.
        let d = import(&pdf("BT /F1 20 Tf 100 200 Td (Hi) Tj ET", ""), &PdfImport { text: false, ..at_scale(1.0) }).unwrap();
        assert!(!model(&d).is_empty() && model(&d).iter().all(|e| matches!(e.kind, EntityKind::LwPolyline(_))));
    }

    #[test]
    fn a_rotated_page_comes_in_as_shown() {
        // /Rotate 90 shows the 595 × 842 page landscape: 842 wide, 595 high.
        let d = import(&pdf("0 0 m 595 0 l S", "/Rotate 90"), &at_scale(1.0)).unwrap();
        let EntityKind::Line(l) = &model(&d)[0].kind else { panic!() };
        for p in [l.a, l.b] {
            assert!((-1e-6..=842.0 + 1e-6).contains(&p.x) && (-1e-6..=595.0 + 1e-6).contains(&p.y), "{l:?}");
        }
        assert!(((l.a.y - l.b.y).abs() - 595.0).abs() < 1e-6, "the page's bottom edge stands upright: {l:?}");
    }

    #[test]
    fn pages_are_counted_and_checked() {
        let bytes = pdf("0 0 m 1 1 l S", "");
        assert_eq!(page_count(&bytes).unwrap(), 1);
        assert!(import(&bytes, &PdfImport { page: 2, ..PdfImport::default() }).is_err());
        assert!(import(&bytes, &PdfImport { page: 0, ..PdfImport::default() }).is_err());
    }

    #[test]
    fn hostile_pdf_does_not_panic() {
        for bytes in [
            Vec::new(),
            b"%PDF-1.7\n".to_vec(),
            b"not a pdf at all".to_vec(),
            pdf("1e308 1e308 m -1e308 -1e308 l S", ""),
            pdf("q q q q q 0 0 m 1 1 l S", ""),
            pdf("[1e30 -5] 0 d 0 0 m 1 1 l S", ""),
            pdf("0 0 m 1 1 c S 1 2 3 re f* BT /F9 0 Tf (x) Tj ET", ""),
        ] {
            let _ = page_count(&bytes);
            let _ = import(&bytes, &PdfImport::default());
        }
        for k in [0.0, -1.0, f64::NAN, f64::INFINITY, 1e300] {
            assert!(import(&pdf("0 0 m 1 1 l S", ""), &at_scale(k)).is_err(), "{k}");
        }
    }
}
