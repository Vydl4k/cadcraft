//! Survey add-on: pictures as underlays (a scanned map, a photo of a plan, a PDF page).
//!
//! - GEOOBRAZEK places an image (PNG, JPEG, BMP, TIFF) or one page of a PDF as a picture under
//!   the drawing, at its real size: from the map scale (with the image's resolution in dpi, or
//!   the PDF page's paper size), from a given real width, or by measuring a known distance on it
//!   afterwards. A PDF stays a picture: nothing is turned into lines. Its page is drawn to a PNG
//!   file beside the PDF (`plan-str1.png` for page 1 of `plan.pdf`), which the drawing then
//!   refers to like any other image, so it opens again with the drawing.
//! - GEOOBRAZEKMER sets a placed picture's scale by a distance measured on it: pick two points
//!   the picture shows and type their real distance; the picture is scaled about the first.
//!
//! The picture's corners, line ends, bends and crossings snap like objects do (snap/raster.rs).

use std::sync::OnceLock;

use cadcraft_doc::{EntityKind, Handle};
use cadcraft_geom::{Vec2, Vec3};
use serde_json::{Value, json};

use super::geo_calc::{say, typed_number};
use super::geo_nastaveni as nast;
use super::{CommandSpec, bad, point_value, str_param};
use crate::{Accept, Input, Interactive, Prompt, Result, Session, Step};

const CMD: &str = "geoobrazek";
const CMD_MER: &str = "geoobrazekmer";
/// Largest PDF read.
const MAX_PDF_BYTES: u64 = 256 << 20;

/// One PDF page drawn as a picture.
pub struct PdfPicture {
    pub png: Vec<u8>,
    pub width: u32,
    pub height: u32,
    /// The page's paper size, millimetres.
    pub page_mm: [f64; 2],
    /// The resolution used.
    pub dpi: f64,
    pub pages: usize,
}

/// Draws page `page` (from 1) of a PDF at a resolution in dpi; installed by the app (the
/// rasterizer lives in the io layer, below this one).
pub type PdfPictureHook = fn(&[u8], usize, Option<&str>, f64) -> std::result::Result<PdfPicture, String>;

static PDF_PICTURE: OnceLock<PdfPictureHook> = OnceLock::new();

pub fn set_pdf_picture(h: PdfPictureHook) {
    let _ = PDF_PICTURE.set(h);
}

pub(super) fn specs() -> Vec<CommandSpec> {
    vec![
        CommandSpec::new(CMD, "Picture Underlay...", run_obrazek)
            .menu(&["Geo", "Picture Underlay..."])
            .alias(&["geoimage", "geopodklad"])
            .params(
                "{path: image (PNG, JPEG, BMP, TIFF) or PDF file, page?: PDF page (default 1), dpi?: resolution the PDF page is drawn at (default 200), \
                 at?: [x, y] lower-left corner (default 0,0), rotation?: degrees, scale?: map scale 1:N (default the drawing's), imagedpi?: the image's \
                 scan resolution (default 300), width?: real width in drawing units (instead of the scale)} → {handle, path, width, message}",
            )
            .interactive(|_| Ok(Box::new(ObrM::default()))),
        CommandSpec::new(CMD_MER, "Scale Picture by Distance...", run_mer)
            .menu(&["Geo", "Scale Picture by Distance..."])
            .alias(&["geoimagescale"])
            .params("{handle?: the picture (default the last one placed), a: [x, y], b: [x, y] two points on it, distance: their real distance} → {factor, message}")
            .interactive(|_| Ok(Box::new(MerM::default()))),
    ]
}

fn ext(path: &str) -> String {
    std::path::Path::new(path).extension().map(|e| e.to_string_lossy().to_ascii_lowercase()).unwrap_or_default()
}

fn num(p: &Value, key: &str, range: std::ops::RangeInclusive<f64>) -> Result<Option<f64>> {
    match p.get(key) {
        None | Some(Value::Null) => Ok(None),
        Some(Value::String(t)) if t.trim().is_empty() => Ok(None),
        Some(v) => v
            .as_f64()
            .or_else(|| v.as_str().and_then(|t| typed_number(t.rsplit(':').next().unwrap_or(t))))
            .filter(|x| x.is_finite() && range.contains(x))
            .map(Some)
            .ok_or_else(|| bad(CMD, format!("`{key}` must be a number from {} to {}", range.start(), range.end()))),
    }
}

#[cfg(not(target_arch = "wasm32"))]
fn read_pdf(path: &str) -> Result<Vec<u8>> {
    crate::file_access::check_read(path).map_err(|e| bad(CMD, e))?;
    let len = std::fs::metadata(path).map_err(|e| bad(CMD, format!("{path}: {e}")))?.len();
    if len > MAX_PDF_BYTES {
        return Err(bad(CMD, format!("{path} is larger than {} MB", MAX_PDF_BYTES >> 20)));
    }
    std::fs::read(path).map_err(|e| bad(CMD, format!("{path}: {e}")))
}

#[cfg(target_arch = "wasm32")]
fn read_pdf(_path: &str) -> Result<Vec<u8>> {
    Err(bad(CMD, "PDF pictures are not available on the web"))
}

#[cfg(not(target_arch = "wasm32"))]
fn write_png(path: &str, png: &[u8]) -> Result<()> {
    crate::file_access::check_write(path).map_err(|e| bad(CMD, e))?;
    std::fs::write(path, png).map_err(|e| bad(CMD, format!("{path}: {e} (the page's picture is saved beside the PDF)")))
}

#[cfg(target_arch = "wasm32")]
fn write_png(_path: &str, _png: &[u8]) -> Result<()> {
    Err(bad(CMD, "PDF pictures are not available on the web"))
}

/// The PNG drawn from page `page` of `pdf`, beside it.
fn picture_path(pdf: &str, page: usize) -> String {
    let p = std::path::Path::new(pdf);
    let stem = p.file_stem().map(|s| s.to_string_lossy().to_string()).unwrap_or_else(|| "pdf".into());
    let name = format!("{stem}-str{page}.png");
    match p.parent().filter(|d| !d.as_os_str().is_empty()) {
        Some(dir) => dir.join(name).to_string_lossy().to_string(),
        None => name,
    }
}

/// Scale the picture `h` by `k` about `base`.
fn scale_image(s: &mut Session, h: Handle, base: Vec2, k: f64) -> Result<()> {
    s.doc_mut()?
        .modify_entity(h, |e| {
            if let EntityKind::Image(i) = &mut e.kind {
                i.u = Vec3::new(i.u.x * k, i.u.y * k, i.u.z * k);
                i.v = Vec3::new(i.v.x * k, i.v.y * k, i.v.z * k);
                i.insert = Vec3::new(base.x + (i.insert.x - base.x) * k, base.y + (i.insert.y - base.y) * k, i.insert.z);
            }
        })
        .map_err(|e| bad(CMD, e.to_string()))
}

fn image_width(s: &Session, h: Handle) -> Option<f64> {
    let d = s.doc().ok()?;
    let e = d.space(&s.space())?.get(h)?;
    match &e.kind {
        EntityKind::Image(i) => Some(i.u.x.hypot(i.u.y) * i.size.x),
        _ => None,
    }
}

fn run_obrazek(s: &mut Session, p: &Value) -> Result<Value> {
    let path = str_param(p, "path")
        .map(str::trim)
        .filter(|t| !t.is_empty())
        .ok_or_else(|| bad(CMD, "`path`: the image or PDF file is required"))?
        .to_string();
    let at = match p.get("at") {
        None | Some(Value::Null) => Vec2::ZERO,
        Some(v) => point_value(v).filter(|q| q.x.abs() < 1e9 && q.y.abs() < 1e9).ok_or_else(|| bad(CMD, "`at` must be [x, y]"))?,
    };
    let rotation = num(p, "rotation", -360.0..=360.0)?.unwrap_or(0.0);
    let map = num(p, "scale", 1.0..=1_000_000.0)?.unwrap_or_else(|| nast::scale(s));
    let width = num(p, "width", 1e-9..=1e9)?;
    let is_pdf = ext(&path) == "pdf";
    let (image_path, real_width, note) = if is_pdf {
        let hook = PDF_PICTURE.get().ok_or_else(|| bad(CMD, "PDF pictures are not available in this build"))?;
        let page = num(p, "page", 1.0..=100_000.0)?.unwrap_or(1.0).round() as usize;
        let dpi = num(p, "dpi", 10.0..=1200.0)?.unwrap_or(200.0);
        let bytes = read_pdf(&path)?;
        let pic = hook(&bytes, page, str_param(p, "password"), dpi).map_err(|e| bad(CMD, e))?;
        let out = picture_path(&path, page);
        write_png(&out, &pic.png)?;
        let w = width.unwrap_or(pic.page_mm[0] / 1000.0 * map);
        let note = format!(
            "page {page} of {} drawn at {:.0} dpi to {out}{}",
            pic.pages,
            pic.dpi,
            if pic.dpi + 0.5 < dpi { " (lowered so the page fits)" } else { "" }
        );
        (out, w, note)
    } else {
        (path.clone(), f64::NAN, String::new())
    };
    let r = s.execute("imageattach", &json!({ "path": image_path, "at": [at.x, at.y], "rotation": rotation, "width": 1.0 }))?;
    let handle = r.get("handle").and_then(Value::as_str).and_then(Handle::parse_hex).ok_or_else(|| bad(CMD, "the picture was not placed"))?;
    let real_width = if is_pdf {
        real_width
    } else {
        match width {
            Some(w) => w,
            None => {
                let dpi = num(p, "imagedpi", 10.0..=10_000.0)?.unwrap_or(300.0);
                let px = r.get("pixels").and_then(|v| v.get(0)).and_then(Value::as_f64).unwrap_or(0.0);
                px / dpi * 0.0254 * map
            }
        }
    };
    if !(real_width.is_finite() && real_width > 0.0) {
        return Err(bad(CMD, "the picture's size could not be worked out"));
    }
    let now = image_width(s, handle).unwrap_or(1.0).max(1e-12);
    scale_image(s, handle, at, real_width / now)?;
    s.set_selection(vec![handle]);
    let how = if width.is_some() { "at the given width".to_string() } else { format!("at 1:{map}") };
    let name = std::path::Path::new(&path).file_name().map(|n| n.to_string_lossy().to_string()).unwrap_or_default();
    Ok(json!({
        "handle": handle.hex(),
        "path": image_path,
        "width": real_width,
        "message": format!("{name} placed {how}, {real_width:.2} wide{}", if note.is_empty() { String::new() } else { format!("; {note}") }),
    }))
}

/// The last picture in the current space.
fn last_image(s: &Session) -> Option<Handle> {
    let d = s.doc().ok()?;
    d.space(&s.space())?.iter().filter(|e| matches!(e.kind, EntityKind::Image(_))).last().map(|e| e.handle)
}

fn run_mer(s: &mut Session, p: &Value) -> Result<Value> {
    let h = match str_param(p, "handle") {
        Some(t) => Handle::parse_hex(t).ok_or_else(|| bad(CMD_MER, "`handle` must be an object handle"))?,
        None => last_image(s).ok_or_else(|| bad(CMD_MER, "there is no picture in the drawing"))?,
    };
    if image_width(s, h).is_none() {
        return Err(bad(CMD_MER, "that is not a picture"));
    }
    let get = |k: &str| {
        p.get(k)
            .and_then(point_value)
            .filter(|q| q.x.abs() < 1e9 && q.y.abs() < 1e9)
            .ok_or_else(|| bad(CMD_MER, format!("`{k}`: [x, y] is required")))
    };
    let (a, b) = (get("a")?, get("b")?);
    let measured = a.dist(b);
    if measured < 1e-9 {
        return Err(bad(CMD_MER, "the two points are at the same place"));
    }
    let dist = p
        .get("distance")
        .and_then(|v| v.as_f64().or_else(|| v.as_str().and_then(typed_number)))
        .filter(|x| x.is_finite() && *x > 0.0 && *x < 1e9)
        .ok_or_else(|| bad(CMD_MER, "`distance`: the real distance is required"))?;
    let k = dist / measured;
    scale_image(s, h, a, k)?;
    Ok(json!({ "factor": k, "message": format!("Picture scaled ×{k:.6}: {measured:.3} on it is now {dist:.3}") }))
}

#[derive(Default)]
struct ObrM {
    path: Option<String>,
    page: Option<String>,
    at: Option<Vec2>,
    method: Option<String>,
    scale: Option<String>,
    /// The picture placed, while its reference distance is measured.
    placed: Option<Handle>,
    a: Option<Vec2>,
    b: Option<Vec2>,
}

impl ObrM {
    fn pdf(&self) -> bool {
        self.path.as_deref().is_some_and(|p| ext(p) == "pdf")
    }
}

impl Interactive for ObrM {
    fn name(&self) -> &'static str {
        "GEOOBRAZEK"
    }
    fn prompt(&self, s: &Session) -> Prompt {
        if self.path.is_none() {
            return Prompt::new("Image or PDF file", Accept::TEXT);
        }
        if self.pdf() && self.page.is_none() {
            return Prompt::new("PDF page", Accept::NUMBER).default("1");
        }
        if self.at.is_none() {
            return Prompt::new("Pick the picture's lower-left corner", Accept::POINT).default("0,0");
        }
        match (self.method.as_deref(), &self.scale, self.placed) {
            (None, ..) => Prompt::new("Picture size", Accept::TEXT).kw(&["Map", "Width", "Reference"]).default("Map"),
            (Some("Map"), None, _) => Prompt::new("Map scale 1:", Accept::NUMBER).default(format!("{}", nast::scale(s))),
            (Some("Map"), Some(_), _) => Prompt::new("Image resolution (dpi)", Accept::NUMBER).default("300"),
            (Some("Width"), ..) => Prompt::new("Real width of the picture", Accept::NUMBER),
            (_, _, Some(_)) => match (self.a, self.b) {
                (None, _) => Prompt::new("Pick the first point of a known distance on the picture", Accept::POINT),
                (Some(a), None) => Prompt::new("Pick the second point", Accept::POINT).base(a),
                _ => Prompt::new("Real distance between the two points", Accept::NUMBER),
            },
            _ => Prompt::new("", Accept::TEXT),
        }
    }
    fn input(&mut self, s: &mut Session, i: Input) -> Result<Step> {
        if matches!(i, Input::Cancel) {
            return Ok(Step::Cancel);
        }
        let text = match &i {
            Input::Text(t) | Input::Keyword(t) => Some(t.trim().trim_matches('"').to_string()),
            Input::Enter => Some(self.prompt(s).default.unwrap_or_default()),
            _ => None,
        };
        if self.path.is_none() {
            match text.filter(|t| !t.is_empty()) {
                Some(t) => self.path = Some(t),
                None => s.echo("Choose an image or PDF file."),
            }
            return Ok(Step::Continue);
        }
        if self.pdf() && self.page.is_none() {
            self.page = text.filter(|t| !t.is_empty()).or(Some("1".into()));
            return Ok(Step::Continue);
        }
        if self.at.is_none() {
            match i {
                Input::Point(q) => self.at = Some(q),
                Input::Enter => self.at = Some(Vec2::ZERO),
                _ => {}
            }
            return Ok(Step::Continue);
        }
        let at = self.at.unwrap_or(Vec2::ZERO);
        let mut base = json!({ "path": self.path, "at": [at.x, at.y] });
        if let Some(pg) = &self.page {
            base["page"] = json!(pg);
        }
        if self.method.is_none() {
            let k = text.and_then(|t| self.prompt(s).match_keyword(&t)).unwrap_or_else(|| "Map".into());
            if k == "Reference" {
                // Placed at the drawing's scale first, then measured.
                match run_obrazek(s, &base) {
                    Ok(r) => self.placed = r.get("handle").and_then(Value::as_str).and_then(Handle::parse_hex),
                    Err(e) => {
                        say(s, Err(e));
                        return Ok(Step::Done);
                    }
                }
            }
            self.method = Some(k);
            return Ok(Step::Continue);
        }
        match self.method.as_deref() {
            Some("Map") => {
                let Some(t) = text else { return Ok(Step::Continue) };
                if self.scale.is_none() {
                    self.scale = Some(t);
                    if !self.pdf() {
                        return Ok(Step::Continue);
                    }
                    base["scale"] = json!(self.scale);
                } else {
                    base["scale"] = json!(self.scale);
                    base["imagedpi"] = json!(t);
                }
                let r = run_obrazek(s, &base);
                say(s, r);
                Ok(Step::Done)
            }
            Some("Width") => {
                let Some(t) = text.filter(|t| !t.is_empty()) else { return Ok(Step::Continue) };
                base["width"] = json!(t);
                let r = run_obrazek(s, &base);
                say(s, r);
                Ok(Step::Done)
            }
            _ => {
                let Some(h) = self.placed else { return Ok(Step::Done) };
                match (self.a, self.b, i) {
                    (None, _, Input::Point(q)) => self.a = Some(q),
                    (Some(_), None, Input::Point(q)) => self.b = Some(q),
                    (Some(a), Some(b), _) => {
                        let Some(t) = text.filter(|t| !t.is_empty()) else { return Ok(Step::Continue) };
                        let r = run_mer(s, &json!({ "handle": h.hex(), "a": [a.x, a.y], "b": [b.x, b.y], "distance": t }));
                        say(s, r);
                        return Ok(Step::Done);
                    }
                    _ => {}
                }
                Ok(Step::Continue)
            }
        }
    }
}

#[derive(Default)]
struct MerM {
    a: Option<Vec2>,
    b: Option<Vec2>,
}

impl Interactive for MerM {
    fn name(&self) -> &'static str {
        "GEOOBRAZEKMER"
    }
    fn prompt(&self, _s: &Session) -> Prompt {
        match (self.a, self.b) {
            (None, _) => Prompt::new("Pick the first point of a known distance on the picture", Accept::POINT),
            (Some(a), None) => Prompt::new("Pick the second point", Accept::POINT).base(a),
            _ => Prompt::new("Real distance between the two points", Accept::NUMBER),
        }
    }
    fn input(&mut self, s: &mut Session, i: Input) -> Result<Step> {
        match (self.a, self.b, i) {
            (_, _, Input::Cancel) => Ok(Step::Cancel),
            (None, _, Input::Point(q)) => {
                self.a = Some(q);
                Ok(Step::Continue)
            }
            (Some(_), None, Input::Point(q)) => {
                self.b = Some(q);
                Ok(Step::Continue)
            }
            (Some(a), Some(b), Input::Text(t) | Input::Keyword(t)) => {
                // The picture under the first point, else the last one placed.
                let h = picture_at(s, a);
                let mut p = json!({ "a": [a.x, a.y], "b": [b.x, b.y], "distance": t.trim() });
                if let Some(h) = h {
                    p["handle"] = json!(h.hex());
                }
                let r = run_mer(s, &p);
                say(s, r);
                Ok(Step::Done)
            }
            _ => Ok(Step::Continue),
        }
    }
}

/// The picture whose frame holds `q`.
fn picture_at(s: &Session, q: Vec2) -> Option<Handle> {
    let d = s.doc().ok()?;
    d.space(&s.space())?
        .iter()
        .filter_map(|e| match &e.kind {
            EntityKind::Image(i) => {
                let (u, v) = (i.u.xy(), i.v.xy());
                let det = u.x * v.y - u.y * v.x;
                if det.abs() < 1e-30 {
                    return None;
                }
                let (dx, dy) = (q.x - i.insert.x, q.y - i.insert.y);
                let a = (dx * v.y - dy * v.x) / det;
                let b = (u.x * dy - u.y * dx) / det;
                (a >= 0.0 && b >= 0.0 && a <= i.size.x && b <= i.size.y).then_some(e.handle)
            }
            _ => None,
        })
        .last()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::snap::{mode, osnap_all};

    /// A white PNG, 200 × 100 pixels, with a black L: a 3-pixel line along row 50 from column 40
    /// to 160, and one up column 40 from row 50 to row 10.
    fn picture(dir: &std::path::Path) -> String {
        let (w, h) = (200u32, 100u32);
        let mut px = vec![255u8; (w * h * 4) as usize];
        let mut ink = |x: u32, y: u32| {
            let k = ((y * w + x) * 4) as usize;
            px[k..k + 3].copy_from_slice(&[0, 0, 0]);
        };
        for x in 40..=160 {
            for t in 49..=51 {
                ink(x, t);
            }
        }
        for y in 10..=50 {
            for t in 39..=41 {
                ink(t, y);
            }
        }
        let r = cadcraft_doc::Raster::new(w, h, px).unwrap();
        let path = dir.join("mapa.png");
        std::fs::write(&path, cadcraft_io::raster::encode_png(&r).unwrap()).unwrap();
        path.to_string_lossy().to_string()
    }

    fn session(tag: &str) -> (Session, std::path::PathBuf) {
        super::super::file::set_image_decoder(|b| cadcraft_io::raster::decode(b).map_err(|e| e.to_string()));
        let dir = std::env::temp_dir().join(format!("geoobrazek-{tag}-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        (Session::new(), dir)
    }

    #[test]
    fn placed_at_map_scale_and_snaps_to_its_lines() {
        let (mut s, dir) = session("snap");
        let path = picture(&dir);
        // 200 px at 100 dpi is 2 inches = 50.8 mm on paper; at 1:1000 that is 50.8 m.
        let r = s.execute(CMD, &json!({ "path": path, "at": [1000, 2000], "scale": 1000, "imagedpi": 100 })).unwrap();
        assert!((r["width"].as_f64().unwrap() - 50.8).abs() < 1e-9, "{r}");
        // One pixel is 0.254 m. The L's corner (pixel 40, 50) sits 40 px right of the left edge
        // and 50 px above the bottom edge.
        let px = 0.254;
        let corner = Vec2::new(1000.0 + 40.5 * px, 2000.0 + 49.5 * px);
        let space = s.space();
        let d = s.doc().unwrap();
        let hits = osnap_all(d, &space, corner + Vec2::new(0.4, -0.3), 2.0, mode::END | mode::INT, None, false);
        assert!(hits.iter().any(|h| h.point.dist(corner) < 1.0), "{:?}", hits.iter().map(|h| h.point).collect::<Vec<_>>());
        // The far end of the horizontal stroke.
        let end = Vec2::new(1000.0 + 160.0 * px, 2000.0 + 49.5 * px);
        let hits = osnap_all(d, &space, end + Vec2::new(-0.5, 0.2), 2.0, mode::END, None, false);
        assert!(hits.iter().any(|h| h.point.dist(end) < 1.0 && h.mode == mode::END));
        // Plain paper offers nothing.
        let hits = osnap_all(d, &space, Vec2::new(1000.0 + 120.0 * px, 2000.0 + 80.0 * px), 2.0, mode::END | mode::INT | mode::NOD, None, false);
        assert!(hits.iter().all(|h| h.mode != mode::END && h.mode != mode::INT), "{hits:?}");
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn measured_distance_sets_the_scale() {
        let (mut s, dir) = session("mer");
        let path = picture(&dir);
        s.execute(CMD, &json!({ "path": path, "width": 20 })).unwrap();
        // Two points 10 apart on the picture are really 25 apart.
        let r = s.execute(CMD_MER, &json!({ "a": [0, 0], "b": [10, 0], "distance": 25 })).unwrap();
        assert!((r["factor"].as_f64().unwrap() - 2.5).abs() < 1e-12);
        let h = last_image(&s).unwrap();
        assert!((image_width(&s, h).unwrap() - 50.0).abs() < 1e-9);
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn hostile_input_is_refused_not_panicking() {
        let (mut s, dir) = session("bad");
        for p in [
            json!({}),
            json!({ "path": "" }),
            json!({ "path": dir.join("none.png").to_string_lossy() }),
            json!({ "path": dir.join("none.pdf").to_string_lossy() }),
            json!({ "path": "x.png", "scale": -5 }),
            json!({ "path": "x.png", "at": [1e300, 0] }),
        ] {
            assert!(s.execute(CMD, &p).is_err(), "{p}");
        }
        assert!(s.execute(CMD_MER, &json!({ "a": [0, 0], "b": [0, 0], "distance": 1 })).is_err());
        let _ = std::fs::remove_dir_all(&dir);
    }
}
