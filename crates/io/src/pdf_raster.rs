//! One PDF page as a picture (for a PDF used as an image underlay): the page rendered to pixels
//! by craft-pdf's rasterizer, on white, encoded as PNG. Nothing is turned into drawing objects.

use crate::{IoError, Result};

/// Bound the input before the parser copies it.
const MAX_INPUT_BYTES: usize = 256 << 20;
const MAX_PAGES: usize = 10_000;

/// A rendered page.
pub struct PdfPagePng {
    pub png: Vec<u8>,
    pub width: u32,
    pub height: u32,
    /// The page's size on paper, millimetres.
    pub page_mm: [f64; 2],
    /// The resolution actually used (lower than asked when the page would not fit the
    /// rasterizer's limits).
    pub dpi: f64,
    /// Pages in the file.
    pub pages: usize,
}

/// Render page `page` (counted from 1) at `dpi` pixels per inch.
pub fn render_page_png(bytes: &[u8], page: usize, password: Option<&str>, dpi: f64) -> Result<PdfPagePng> {
    if !(dpi.is_finite() && (10.0..=1200.0).contains(&dpi)) {
        return Err(IoError::Format("PDF: the resolution must be 10 to 1200 dpi".into()));
    }
    let options = craft_pdf::OpenOptions { max_input_bytes: MAX_INPUT_BYTES, ..Default::default() };
    let pdf = craft_pdf::open_slice_with_options(bytes, password, &options).map_err(|e| IoError::Format(format!("PDF: {e}")))?;
    let pages = pdf.pages();
    let count = pages.len();
    if count > MAX_PAGES {
        return Err(IoError::Format(format!("PDF: more than {MAX_PAGES} pages")));
    }
    let p = page
        .checked_sub(1)
        .and_then(|i| pages.get(i))
        .ok_or_else(|| IoError::Format(format!("PDF: page {page} doesn't exist; the file has {count} page{}", if count == 1 { "" } else { "s" })))?;
    let (w, h) = p.render_dimensions();
    let (w, h) = (f64::from(w), f64::from(h));
    if !(w.is_finite() && h.is_finite() && w >= 1.0 && h >= 1.0 && w <= 200_000.0 && h <= 200_000.0) {
        return Err(IoError::Format(format!("PDF: page size {w} × {h} points is not usable")));
    }
    // Lower the resolution until the picture fits the rasterizer's side and area limits.
    let side = f64::from(craft_pdf::raster::MAX_SIDE) - 1.0;
    let area = craft_pdf::raster::MAX_PIXELS as f64 * 0.99;
    let mut dpi = dpi;
    let fit = |d: f64| w * d / 72.0 <= side && h * d / 72.0 <= side && (w * d / 72.0) * (h * d / 72.0) <= area;
    if !fit(dpi) {
        let by_side = side * 72.0 / w.max(h);
        let by_area = (area / (w * h)).sqrt() * 72.0;
        dpi = by_side.min(by_area).floor();
    }
    if !(dpi >= 1.0 && fit(dpi)) {
        return Err(IoError::Format("PDF: the page is too large to show as a picture".into()));
    }
    let scale = (dpi / 72.0) as f32;
    let ropts = craft_pdf::raster::Options {
        scale: [scale, scale],
        size: None,
        offset: [0.0, 0.0],
        background: [255, 255, 255, 255],
        color_replacement: None,
        max_coverage: None,
    };
    let cache = craft_pdf::raster::RenderCache::new();
    let settings = craft_pdf::interpret::InterpreterSettings::default();
    let r = craft_pdf::raster::render(p, &cache, &settings, &ropts).map_err(|e| IoError::Format(format!("PDF: the page can't be drawn: {e:?}")))?;
    let raster =
        cadcraft_doc::Raster::new(r.width, r.height, r.bytes().to_vec()).ok_or_else(|| IoError::Format("PDF: the page picture is empty".into()))?;
    let png = crate::raster::encode_png(&raster)?;
    Ok(PdfPagePng { png, width: r.width, height: r.height, page_mm: [w * 25.4 / 72.0, h * 25.4 / 72.0], dpi, pages: count })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_plotted_page_comes_back_as_a_picture() {
        let mut d = cadcraft_doc::Drawing::new_metric();
        let line = cadcraft_doc::EntityKind::Line(cadcraft_doc::Line {
            a: cadcraft_geom::Vec3::new(0.0, 0.0, 0.0),
            b: cadcraft_geom::Vec3::new(100.0, 50.0, 0.0),
        });
        d.add(&cadcraft_doc::Space::Model, cadcraft_doc::Common::default(), line).unwrap();
        let pdf = crate::pdf::plot(&d, &cadcraft_doc::Space::Model, &serde_json::json!({})).unwrap();
        let low = render_page_png(&pdf, 1, None, 50.0).unwrap();
        let high = render_page_png(&pdf, 1, None, 100.0).unwrap();
        assert_eq!(low.pages, 1);
        assert!(high.width > low.width && high.height > low.height);
        let r = crate::raster::decode(&high.png).unwrap();
        // White paper with some ink on it.
        let px: Vec<[u8; 4]> = r.rgba().chunks_exact(4).map(|c| [c[0], c[1], c[2], c[3]]).collect();
        assert!(px.iter().any(|p| p[0] < 128) && px.iter().filter(|p| p[0] > 250).count() * 2 > px.len());
        assert!(render_page_png(&pdf, 2, None, 50.0).is_err());
        // A huge resolution is lowered to what fits, not refused.
        let big = render_page_png(&pdf, 1, None, 1200.0).unwrap();
        assert!(big.dpi <= 1200.0 && big.width <= craft_pdf::raster::MAX_SIDE);
    }

    #[test]
    fn damaged_input_is_refused_not_panicking() {
        for b in [&b""[..], b"%PDF-1.7\n", b"%PDF-1.4\n1 0 obj << /Type /Catalog >> endobj\ntrailer << /Root 1 0 R >>\n%%EOF", &[0xFFu8; 64][..]] {
            assert!(render_page_png(b, 1, None, 150.0).is_err());
        }
        assert!(render_page_png(b"%PDF", 1, None, f64::NAN).is_err());
    }
}
