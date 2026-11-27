//! Survey add-on: joining and exchanging terrain models.
//!
//! - GEOSLOUCIT: merge the design surface (GEO_DMT_navrh) into the existing terrain (GEO_DMT):
//!   inside the design's footprint the design's triangles stand as they are, outside it the
//!   existing triangles do, and between the design's edge and the nearest existing points new
//!   triangles tie the two together. The result goes on GEO_DMT_slouceny (or replaces GEO_DMT).
//! - GEOPOLOZ: drape lines, polylines and points on the terrain model: 2D drawing becomes 3D,
//!   with a vertex wherever a line crosses a triangle edge.
//! - GEOIMPORTXML: points (CgPoints) and TIN surfaces from a LandXML file, the counterpart of
//!   GEOLANDXML.
//! - GEOHECRAS: the river axis and cross sections cut from the terrain model, in the text format
//!   HEC-RAS imports as "GIS Format" geometry (after the HEC-RAS manual's GIS data exchange
//!   appendix).

use std::collections::HashMap;

use cadcraft_doc::{Common, EntityKind, Face3d, Handle, Polyline3d};
use cadcraft_geom::{Vec2, Vec3};
use serde_json::{Value, json};
use spade::{ConstrainedDelaunayTriangulation, HasPosition, Point2, Triangulation};

use super::geo::decode;
use super::geo_calc::{add, along, ensure_layer, one_target, path_of, say, section, tin, tin_on, typed_number};
use super::geo_dmt::{LAYER_DESIGN, LAYER_TIN, clear_layer};
use super::{CommandSpec, bad, f64_or, str_param};
use crate::{Accept, Input, Interactive, Prompt, Result, Session, Step};

pub const LAYER_MERGED: &str = "GEO_DMT_slouceny";
pub const LAYER_DRAPED: &str = "GEO_3D";

/// Vertices or triangles handled at once.
const MAX_ITEMS: usize = 2_000_000;
/// Grid entries of a triangle locator, and line-segment × triangle tests of a drape.
const MAX_WORK: usize = 200_000_000;

pub(super) fn specs() -> Vec<CommandSpec> {
    vec![
        CommandSpec::new("geosloucit", "Merge Terrain Models", run_geosloucit)
            .menu(&["Geo", "Merge Terrain Models"])
            .alias(&["geomerge"])
            .params("{existing?: layer of the existing terrain (default GEO_DMT), design?: layer of the design (default GEO_DMT_navrh), replaceExisting?: bool, put the result on the existing terrain's layer instead of GEO_DMT_slouceny (default false)} → {triangles, design, existing, joining, layer, message}"),
        CommandSpec::new("geopoloz", "Drape on Terrain", run_geopoloz)
            .menu(&["Geo", "Drape on Terrain"])
            .alias(&["geodrape", "geo3d"])
            .params("{handles?: lines, polylines and points (default: the selection), layer?: where the 3D polylines go (default GEO_3D), erase?: bool, remove the 2D originals (default false)} → {lines, points, outside, message}"),
        CommandSpec::new("geoimportxml", "Import LandXML...", run_geoimportxml)
            .menu(&["Geo", "Import LandXML..."])
            .alias(&["geolandxmlin", "-geoimportxml"])
            .params("{path | text: the LandXML file, points?: bool, import CgPoints (default true), surfaces?: bool (default true), scale?: map scale 1:N for point labels (default 500)} → {points, surfaces, triangles, layers, message}")
            .interactive(|_| Ok(Box::new(FileM))),
        CommandSpec::new("geohecras", "Export for HEC-RAS...", run_geohecras)
            .menu(&["Geo", "Export", "HEC-RAS..."])
            .alias(&["georas"])
            .params("{axis?: handle of the river axis, drawn downstream (default: the selected one), step?: distance between cross sections (default 20), width?: to each side (default 30), river?: name (default \"Tok\"), reach?: name (default \"Usek\"), path?: file to write (otherwise the text is returned)} → {sections, length, text?, message}")
            .interactive(|_| Ok(Box::new(RasM::default()))),
    ]
}

// --- triangle locator ----------------------------------------------------------------------------

/// Whether a point lies inside (or on the edge of) a triangle in plan.
fn inside(p: Vec2, t: &[Vec3; 3]) -> bool {
    let c = |a: Vec3, b: Vec3| (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);
    let (d0, d1, d2) = (c(t[0], t[1]), c(t[1], t[2]), c(t[2], t[0]));
    let eps = 1e-9;
    (d0 >= -eps && d1 >= -eps && d2 >= -eps) || (d0 <= eps && d1 <= eps && d2 <= eps)
}

/// A grid over a model's triangles, answering "which triangle is this point in" quickly.
pub struct Locator<'a> {
    tris: &'a [[Vec3; 3]],
    lo: Vec2,
    cell: f64,
    nx: usize,
    ny: usize,
    cells: Vec<Vec<u32>>,
}

impl<'a> Locator<'a> {
    pub fn new(tris: &'a [[Vec3; 3]]) -> Option<Locator<'a>> {
        if tris.is_empty() || tris.len() > u32::MAX as usize {
            return None;
        }
        let (mut lo, mut hi) = (Vec2::new(f64::INFINITY, f64::INFINITY), Vec2::new(f64::NEG_INFINITY, f64::NEG_INFINITY));
        for v in tris.iter().flatten() {
            lo = Vec2::new(lo.x.min(v.x), lo.y.min(v.y));
            hi = Vec2::new(hi.x.max(v.x), hi.y.max(v.y));
        }
        let (w, h) = ((hi.x - lo.x).max(1e-6), (hi.y - lo.y).max(1e-6));
        if !(w.is_finite() && h.is_finite()) {
            return None;
        }
        let cell = ((w * h / tris.len() as f64).sqrt() * 2.0).max(w / 2000.0).max(h / 2000.0).max(1e-6);
        let nx = ((w / cell).ceil() as usize).clamp(1, 2000);
        let ny = ((h / cell).ceil() as usize).clamp(1, 2000);
        let mut g = Locator { tris, lo, cell, nx, ny, cells: vec![Vec::new(); nx * ny] };
        let mut entries = 0usize;
        for (i, t) in tris.iter().enumerate() {
            let (x0, y0) =
                g.cell_of(Vec2::new(t.iter().map(|v| v.x).fold(f64::INFINITY, f64::min), t.iter().map(|v| v.y).fold(f64::INFINITY, f64::min)));
            let (x1, y1) = g
                .cell_of(Vec2::new(t.iter().map(|v| v.x).fold(f64::NEG_INFINITY, f64::max), t.iter().map(|v| v.y).fold(f64::NEG_INFINITY, f64::max)));
            for y in y0..=y1 {
                for x in x0..=x1 {
                    entries += 1;
                    if entries > MAX_WORK {
                        return None;
                    }
                    if let Some(c) = g.cells.get_mut(y * nx + x) {
                        c.push(i as u32);
                    }
                }
            }
        }
        Some(g)
    }

    fn cell_of(&self, p: Vec2) -> (usize, usize) {
        let f = |v: f64, n: usize| ((v / self.cell).floor().max(0.0) as usize).min(n - 1);
        (f(p.x - self.lo.x, self.nx), f(p.y - self.lo.y, self.ny))
    }

    /// The triangle covering `p`.
    pub fn find(&self, p: Vec2) -> Option<&'a [Vec3; 3]> {
        if !(p.x.is_finite() && p.y.is_finite()) || p.x < self.lo.x - 1e-6 || p.y < self.lo.y - 1e-6 {
            return None;
        }
        let (x, y) = self.cell_of(p);
        self.cells.get(y * self.nx + x)?.iter().filter_map(|i| self.tris.get(*i as usize)).find(|t| inside(p, t))
    }

    pub fn covers(&self, p: Vec2) -> bool {
        self.find(p).is_some()
    }

    /// The model's height at `p`.
    pub fn height(&self, p: Vec2) -> Option<f64> {
        let t = self.find(p)?;
        // Relative to the first corner, for precision far from the origin.
        let (a, b, c) = (t[0], t[1], t[2]);
        let (ux, uy, uz) = (b.x - a.x, b.y - a.y, b.z - a.z);
        let (vx, vy, vz) = (c.x - a.x, c.y - a.y, c.z - a.z);
        let den = ux * vy - uy * vx;
        if den.abs() < 1e-18 {
            return None;
        }
        let (px, py) = (p.x - a.x, p.y - a.y);
        let s = (px * vy - py * vx) / den;
        let t2 = (ux * py - uy * px) / den;
        Some(a.z + s * uz + t2 * vz)
    }
}

// --- GEOSLOUCIT ----------------------------------------------------------------------------------

#[derive(Clone, Copy, Debug)]
struct V {
    p: Point2<f64>,
    z: f64,
}

impl HasPosition for V {
    type Scalar = f64;
    fn position(&self) -> Point2<f64> {
        self.p
    }
}

/// The merged model's triangles, and how many came from the design, the existing terrain and the
/// join between them.
pub fn merge(existing: &[[Vec3; 3]], design: &[[Vec3; 3]]) -> std::result::Result<(Vec<[Vec3; 3]>, usize, usize, usize), String> {
    if existing.is_empty() || design.is_empty() {
        return Err("both terrain models need triangles".into());
    }
    if existing.len() > MAX_ITEMS || design.len() > MAX_ITEMS {
        return Err(format!("at most {MAX_ITEMS} triangles in a model"));
    }
    let too_large = || "the terrain models are too large to merge at once".to_string();
    let in_design = Locator::new(design).ok_or_else(too_large)?;
    let in_existing = Locator::new(existing).ok_or_else(too_large)?;
    // Triangulate about the design's first corner: survey coordinates are ~10^6 m.
    let o = design.first().map(|t| t[0].xy()).unwrap_or(Vec2::ZERO);
    let key = |q: Vec3| ((q.x * 1000.0).round() as i64, (q.y * 1000.0).round() as i64);
    let mut cdt: ConstrainedDelaunayTriangulation<V> = ConstrainedDelaunayTriangulation::new();
    let mut index: HashMap<(i64, i64), spade::handles::FixedVertexHandle> = HashMap::new();
    let mut insert = |cdt: &mut ConstrainedDelaunayTriangulation<V>, q: Vec3| -> std::result::Result<(), String> {
        if !(q.x.is_finite() && q.y.is_finite() && q.z.is_finite()) || index.contains_key(&key(q)) {
            return Ok(());
        }
        let h = cdt.insert(V { p: Point2::new(q.x - o.x, q.y - o.y), z: q.z }).map_err(|e| format!("a point could not be triangulated: {e:?}"))?;
        index.insert(key(q), h);
        Ok(())
    };
    // The design first: where a point of both models coincides, the design's height stands.
    for q in design.iter().flatten() {
        insert(&mut cdt, *q)?;
    }
    for q in existing.iter().flatten() {
        if !in_design.covers(q.xy()) {
            insert(&mut cdt, *q)?;
        }
    }
    // Every design edge is kept, then every existing edge that still has both ends and does not
    // run through the design: so both models' own triangles come out as they were.
    for tris in [design, existing] {
        for t in tris {
            for (a, b) in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
                let (Some(&ha), Some(&hb)) = (index.get(&key(a)), index.get(&key(b))) else { continue };
                if ha != hb {
                    // Refused (nothing added) when it would cross an edge that already stands.
                    let _ = cdt.try_add_constraint(ha, hb);
                }
            }
        }
    }
    let mut out = Vec::new();
    let (mut n_design, mut n_existing, mut n_join) = (0, 0, 0);
    for f in cdt.inner_faces() {
        let vs = f.vertices().map(|v| *v.data());
        let t = vs.map(|v| Vec3::new(v.p.x + o.x, v.p.y + o.y, v.z));
        let c = Vec2::new((t[0].x + t[1].x + t[2].x) / 3.0, (t[0].y + t[1].y + t[2].y) / 3.0);
        if in_design.covers(c) {
            n_design += 1;
        } else if in_existing.covers(c) {
            // An existing triangle that survived whole keeps all three of its own corners.
            let own = |v: Vec3| in_existing.height(v.xy()).is_some_and(|z| (z - v.z).abs() < 1e-6);
            if t.iter().all(|v| own(*v)) {
                n_existing += 1;
            } else {
                n_join += 1;
            }
        } else {
            // Outside both models: the hull's filling, not terrain.
            continue;
        }
        out.push(t);
    }
    Ok((out, n_design, n_existing, n_join))
}

fn layer_name(p: &Value, key: &str, default: &str, cmd: &str) -> Result<String> {
    match p.get(key) {
        None | Some(Value::Null) => Ok(default.to_string()),
        Some(Value::String(l)) if !l.is_empty() && l.chars().count() <= 64 && !l.chars().any(|c| "<>/\\\":;?*|=,`".contains(c) || c.is_control()) => {
            Ok(l.clone())
        }
        Some(_) => Err(bad(cmd, format!("`{key}` must be a layer name"))),
    }
}

fn write_tin(s: &mut Session, layer: &str, aci: u8, tris: &[[Vec3; 3]], cmd: &str) -> Result<usize> {
    let space = s.space();
    clear_layer(s, &space, layer)?;
    ensure_layer(s, layer, aci)?;
    let common = Common { layer: layer.into(), ..s.current_common() };
    let d = s.doc_mut()?;
    for t in tris {
        d.add(&space, common.clone(), EntityKind::Face3d(Face3d { corners: [t[0], t[1], t[2], t[2]], hidden_edges: 0 }))
            .map_err(|e| bad(cmd, e.to_string()))?;
    }
    Ok(tris.len())
}

fn run_geosloucit(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "geosloucit";
    let space = s.space();
    let existing_layer = layer_name(p, "existing", LAYER_TIN, CMD)?;
    let design_layer = layer_name(p, "design", LAYER_DESIGN, CMD)?;
    let existing = tin_on(s, &space, &existing_layer)?;
    if existing.is_empty() {
        return Err(bad(CMD, format!("no existing terrain on {existing_layer}: run GEODMT first")));
    }
    let design = tin_on(s, &space, &design_layer)?;
    if design.is_empty() {
        return Err(bad(CMD, format!("no design surface on {design_layer}: select the design's points and lines and run GEONAVRH")));
    }
    let (tris, n_design, n_existing, n_join) = merge(&existing, &design).map_err(|e| bad(CMD, e))?;
    let target = if p.get("replaceExisting").and_then(Value::as_bool).unwrap_or(false) { existing_layer } else { LAYER_MERGED.to_string() };
    let n = write_tin(s, &target, 30, &tris, CMD)?;
    Ok(json!({
        "triangles": n, "design": n_design, "existing": n_existing, "joining": n_join, "layer": target,
        "message": format!("Merged terrain on {target}: {n} triangles ({n_design} of the design, {n_existing} of the existing terrain, {n_join} joining them)"),
    }))
}

// --- GEOPOLOZ ------------------------------------------------------------------------------------

fn run_geopoloz(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "geopoloz";
    let space = s.space();
    let chosen = super::targets(s, p)?;
    if chosen.is_empty() {
        return Err(bad(CMD, "select the lines, polylines and points to drape first"));
    }
    let layer = layer_name(p, "layer", LAYER_DRAPED, CMD)?;
    let erase = p.get("erase").and_then(Value::as_bool).unwrap_or(false);
    let tris = tin(s, &space)?;
    if tris.is_empty() {
        return Err(bad(CMD, "no terrain model: run GEODMT first"));
    }
    let locate = Locator::new(&tris).ok_or_else(|| bad(CMD, "the terrain model is too large"))?;
    // What each object is, read before anything changes.
    enum Item {
        Path(Handle, Vec<Vec2>),
        Point(Handle, Vec2),
    }
    let items: Vec<Item> = {
        let d = s.doc()?;
        let st = d.space(&space).ok_or_else(|| bad(CMD, "no drawing space"))?;
        chosen
            .iter()
            .filter_map(|h| {
                let e = st.get(*h)?;
                match &e.kind {
                    EntityKind::Point(q) => Some(Item::Point(*h, q.p.xy())),
                    EntityKind::Line(_) | EntityKind::LwPolyline(_) | EntityKind::Polyline3d(_) => {
                        let (mut path, closed) = path_of(s, *h, &space)?;
                        if closed && let Some(f) = path.first().copied() {
                            path.push(f);
                        }
                        Some(Item::Path(*h, path))
                    }
                    _ => None,
                }
            })
            .collect()
    };
    let segments: usize = items.iter().map(|i| if let Item::Path(_, v) = i { v.len() } else { 0 }).sum();
    if segments.saturating_mul(tris.len()) > MAX_WORK {
        return Err(bad(CMD, "too much to drape at once: select fewer lines"));
    }
    let (mut n_lines, mut n_points, mut outside) = (0, 0, 0);
    let mut made_layer = false;
    for item in items {
        match item {
            Item::Point(h, q) => match locate.height(q) {
                Some(z) => {
                    s.doc_mut()?
                        .modify_entity(h, |e: &mut cadcraft_doc::Entity| {
                            if let EntityKind::Point(pt) = &mut e.kind {
                                pt.p.z = z;
                            }
                        })
                        .map_err(|e| bad(CMD, e.to_string()))?;
                    n_points += 1;
                }
                None => outside += 1,
            },
            Item::Path(h, path) => {
                let pts: Vec<Vec3> = section(&tris, &path)
                    .into_iter()
                    .map(|(st, z)| {
                        let q = along(&path, st);
                        Vec3::new(q.x, q.y, z)
                    })
                    .collect();
                if pts.len() < 2 {
                    outside += 1;
                    continue;
                }
                if !made_layer {
                    ensure_layer(s, &layer, 6)?;
                    made_layer = true;
                }
                add(s, &space, &layer, EntityKind::Polyline3d(Polyline3d { points: pts, closed: false }))?;
                if erase {
                    s.doc_mut()?.remove_entity(h);
                }
                n_lines += 1;
            }
        }
    }
    if n_lines == 0 && n_points == 0 {
        return Err(bad(CMD, "nothing selected lies on the terrain model"));
    }
    Ok(json!({
        "lines": n_lines, "points": n_points, "outside": outside,
        "message": format!("{n_lines} lines and {n_points} points draped on the terrain{}", if outside > 0 { format!("; {outside} lie outside the model") } else { String::new() }),
    }))
}

// --- GEOIMPORTXML --------------------------------------------------------------------------------

/// One step of an XML text: an opening (or empty) tag with its attributes and the text after
/// it, or a closing tag.
enum Xml<'a> {
    Open { name: &'a str, attrs: &'a str, text: &'a str },
    Close(&'a str),
}

/// The tags of `xml` in order. Comments, declarations and CDATA are skipped; a broken end of
/// file just ends the walk.
fn tags(xml: &str) -> impl Iterator<Item = Xml<'_>> {
    let mut rest = xml;
    std::iter::from_fn(move || {
        loop {
            let at = rest.find('<')?;
            rest = rest.get(at..)?;
            if let Some(r) = rest.strip_prefix("<!--") {
                rest = r.get(r.find("-->")? + 3..)?;
                continue;
            }
            if rest.starts_with("<?") || rest.starts_with("<!") {
                rest = rest.get(rest.find('>')? + 1..)?;
                continue;
            }
            // The tag ends at the first '>' outside quotes.
            let mut quote: Option<char> = None;
            let end = rest.char_indices().find_map(|(i, c)| {
                match (quote, c) {
                    (None, '"' | '\'') => quote = Some(c),
                    (Some(q), c) if c == q => quote = None,
                    (None, '>') => return Some(i),
                    _ => {}
                }
                None
            })?;
            let inner = rest.get(1..end)?;
            rest = rest.get(end + 1..)?;
            let local = |n: &'_ str| -> usize { n.rfind(':').map_or(0, |i| i + 1) };
            if let Some(name) = inner.strip_prefix('/') {
                let name = name.trim();
                return Some(Xml::Close(name.get(local(name)..).unwrap_or(name)));
            }
            let inner = inner.trim_end_matches('/');
            let split = inner.find(|c: char| c.is_whitespace()).unwrap_or(inner.len());
            let (name, attrs) = (inner.get(..split).unwrap_or(inner), inner.get(split..).unwrap_or(""));
            let text = rest.get(..rest.find('<').unwrap_or(rest.len())).unwrap_or("");
            return Some(Xml::Open { name: name.get(local(name)..).unwrap_or(name), attrs, text });
        }
    })
}

/// The value of attribute `key` in a tag's attribute text.
fn attr<'a>(attrs: &'a str, key: &str) -> Option<&'a str> {
    let mut rest = attrs;
    loop {
        let at = rest.find(key)?;
        let before_ok = rest.get(..at)?.chars().next_back().is_none_or(|c| c.is_whitespace());
        let after = rest.get(at + key.len()..)?.trim_start();
        if before_ok && let Some(v) = after.strip_prefix('=') {
            let v = v.trim_start();
            let q = v.chars().next().filter(|c| *c == '"' || *c == '\'')?;
            let v = v.get(1..)?;
            return v.get(..v.find(q)?);
        }
        rest = rest.get(at + key.len()..)?;
    }
}

fn unescape(t: &str) -> String {
    t.replace("&lt;", "<").replace("&gt;", ">").replace("&quot;", "\"").replace("&apos;", "'").replace("&amp;", "&")
}

fn three(t: &str) -> Option<(f64, f64, Option<f64>)> {
    let mut f = t.split_whitespace().map(|v| v.parse::<f64>().ok().filter(|v| v.is_finite() && v.abs() < 1e9));
    let (n, e) = (f.next()??, f.next()??);
    Some((n, e, f.next().flatten()))
}

/// What a LandXML file holds: points as (name, northing, easting, height) and surfaces as
/// (name, triangles in northing/easting/height).
#[derive(Debug, Default)]
pub struct LandXml {
    pub points: Vec<(String, f64, f64, Option<f64>)>,
    pub surfaces: Vec<(String, Vec<[[f64; 3]; 3]>)>,
}

pub fn parse_landxml(xml: &str) -> std::result::Result<LandXml, String> {
    let mut out = LandXml::default();
    let mut surface: Option<String> = None;
    let mut pnts: HashMap<&str, [f64; 3]> = HashMap::new();
    let mut faces: Vec<[[f64; 3]; 3]> = Vec::new();
    let mut seen = 0usize;
    for tag in tags(xml) {
        seen += 1;
        if seen > 50_000_000 {
            return Err("the file has too many elements".into());
        }
        match tag {
            Xml::Open { name: "CgPoint", attrs, text } => {
                let Some((n, e, z)) = three(text) else { continue };
                if out.points.len() >= MAX_ITEMS {
                    return Err(format!("more than {MAX_ITEMS} points"));
                }
                let name = attr(attrs, "name").map(unescape).unwrap_or_default();
                out.points.push((name, n, e, z));
            }
            Xml::Open { name: "Surface", attrs, .. } => {
                surface = Some(attr(attrs, "name").map(unescape).unwrap_or_default());
                pnts.clear();
                faces.clear();
            }
            Xml::Open { name: "P", attrs, text } if surface.is_some() => {
                let (Some(id), Some((n, e, Some(z)))) = (attr(attrs, "id"), three(text)) else { continue };
                if pnts.len() >= MAX_ITEMS {
                    return Err(format!("more than {MAX_ITEMS} surface points"));
                }
                pnts.insert(id, [n, e, z]);
            }
            Xml::Open { name: "F", attrs, text } if surface.is_some() => {
                // i="1" marks a face that is not part of the surface.
                if attr(attrs, "i") == Some("1") {
                    continue;
                }
                let mut ids = text.split_whitespace().map(|id| pnts.get(id).copied());
                let (Some(Some(a)), Some(Some(b)), Some(Some(c))) = (ids.next(), ids.next(), ids.next()) else { continue };
                if faces.len() >= MAX_ITEMS {
                    return Err(format!("more than {MAX_ITEMS} triangles"));
                }
                faces.push([a, b, c]);
            }
            Xml::Close("Surface") => {
                if let Some(name) = surface.take()
                    && !faces.is_empty()
                {
                    out.surfaces.push((name, std::mem::take(&mut faces)));
                }
                pnts.clear();
            }
            _ => {}
        }
    }
    Ok(out)
}

fn run_geoimportxml(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "geoimportxml";
    let xml = match (str_param(p, "text"), str_param(p, "path")) {
        (Some(t), _) => t.to_string(),
        (None, Some(path)) => {
            #[cfg(not(target_arch = "wasm32"))]
            {
                let meta = std::fs::metadata(path).map_err(|e| bad(CMD, format!("{path}: {e}")))?;
                if meta.len() > 1024 * 1024 * 1024 {
                    return Err(bad(CMD, format!("{path}: larger than 1 GB")));
                }
                decode(&std::fs::read(path).map_err(|e| bad(CMD, format!("{path}: {e}")))?)
            }
            #[cfg(target_arch = "wasm32")]
            return Err(bad(CMD, format!("{path}: files can't be read on the web; pass `text`")));
        }
        (None, None) => return Err(bad(CMD, "`path` (the LandXML file) or `text` is required")),
    };
    let scale = f64_or(p, "scale", super::geo_nastaveni::scale(s));
    if !(scale.is_finite() && (1.0..=1e6).contains(&scale)) {
        return Err(bad(CMD, "`scale` must be a map scale like 500"));
    }
    let mut land = parse_landxml(&xml).map_err(|e| bad(CMD, e))?;
    if !p.get("points").and_then(Value::as_bool).unwrap_or(true) {
        land.points.clear();
    }
    if !p.get("surfaces").and_then(Value::as_bool).unwrap_or(true) {
        land.surfaces.clear();
    }
    if land.points.is_empty() && land.surfaces.is_empty() {
        return Err(bad(CMD, "no points (CgPoint) or TIN surfaces found: is this a LandXML file?"));
    }
    // Positive S-JTSK (northing = X, easting = Y) is drawn at x = −Y, y = −X; anything else as
    // easting, northing.
    let all = land.points.iter().map(|(_, n, e, _)| (*n, *e)).chain(land.surfaces.iter().flat_map(|(_, f)| f.iter().flatten().map(|v| (v[0], v[1]))));
    let sjtsk = all.clone().next().is_some() && all.clone().all(|(n, e)| n > 900_000.0 && e > 300_000.0);
    let place = |n: f64, e: f64| if sjtsk { Vec2::new(-e, -n) } else { Vec2::new(e, n) };
    let mut n_points = 0;
    if !land.points.is_empty() {
        let mut list = String::new();
        for (i, (name, n, e, z)) in land.points.iter().enumerate() {
            let name: String = name.split_whitespace().collect::<Vec<_>>().join("_");
            let name = if name.is_empty() { format!("{}", i + 1) } else { name };
            match z {
                Some(z) => list.push_str(&format!("{name} {e} {n} {z}\n")),
                None => list.push_str(&format!("{name} {e} {n}\n")),
            }
        }
        let r = s.execute("geobody", &json!({ "text": list, "columns": "PYXZ", "sjtsk": sjtsk, "scale": scale }))?;
        n_points = r.get("points").and_then(Value::as_u64).unwrap_or(land.points.len() as u64);
    }
    let (mut n_tris, mut layers) = (0, Vec::new());
    for (i, (name, faces)) in land.surfaces.iter().enumerate() {
        let layer = if name.eq_ignore_ascii_case(LAYER_DESIGN) {
            LAYER_DESIGN.to_string()
        } else if !layers.iter().any(|l: &String| l == LAYER_TIN) {
            LAYER_TIN.to_string()
        } else {
            let clean: String = name.chars().filter(|c| c.is_alphanumeric() || *c == '_' || *c == '-').take(40).collect();
            format!("GEO_DMT_{}", if clean.is_empty() { format!("{}", i + 1) } else { clean })
        };
        let tris: Vec<[Vec3; 3]> = faces
            .iter()
            .map(|f| {
                f.map(|v| {
                    let q = place(v[0], v[1]);
                    Vec3::new(q.x, q.y, v[2])
                })
            })
            .collect();
        n_tris += write_tin(s, &layer, if layer == LAYER_DESIGN { 3 } else { 8 }, &tris, CMD)?;
        layers.push(layer);
    }
    Ok(json!({
        "points": n_points, "surfaces": layers.len(), "triangles": n_tris, "layers": layers,
        "message": format!("LandXML: {n_points} points, {} surfaces with {n_tris} triangles{}", layers.len(), if layers.is_empty() { String::new() } else { format!(" on {}", layers.join(", ")) }),
    }))
}

// --- GEOHECRAS -----------------------------------------------------------------------------------

/// The point `st` along `path` and the direction there.
fn frame_at(path: &[Vec2], st: f64) -> Option<(Vec2, Vec2)> {
    let mut left = st;
    let mut last = None;
    for w in path.windows(2) {
        let [a, b] = w else { continue };
        let len = (b.x - a.x).hypot(b.y - a.y);
        if len < 1e-12 {
            continue;
        }
        let u = Vec2::new((b.x - a.x) / len, (b.y - a.y) / len);
        if left <= len + 1e-9 {
            let t = (left / len).clamp(0.0, 1.0);
            return Some((Vec2::new(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)), u));
        }
        left -= len;
        last = Some((*b, u));
    }
    last
}

fn ras_name(p: &Value, key: &str, default: &str, cmd: &str) -> Result<String> {
    match p.get(key) {
        None | Some(Value::Null) => Ok(default.to_string()),
        // HEC-RAS keeps 16 characters; commas and line breaks would break the file.
        Some(Value::String(t)) if !t.trim().is_empty() && t.chars().count() <= 16 && !t.chars().any(|c| c == ',' || c == ':' || c.is_control()) => {
            Ok(t.trim().to_string())
        }
        Some(_) => Err(bad(cmd, format!("`{key}` must be a name of at most 16 characters without commas or colons"))),
    }
}

fn run_geohecras(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "geohecras";
    let space = s.space();
    let h = one_target(s, p, "axis", CMD)?;
    let (axis, _) = path_of(s, h, &space).ok_or_else(|| bad(CMD, "the axis must be a line or polyline"))?;
    let step = f64_or(p, "step", 20.0);
    let width = f64_or(p, "width", 30.0);
    if !(step.is_finite() && step > 0.0 && width.is_finite() && width > 0.0 && width < 1e5) {
        return Err(bad(CMD, "`step` and `width` must be positive"));
    }
    let river = ras_name(p, "river", "Tok", CMD)?;
    let reach = ras_name(p, "reach", "Usek", CMD)?;
    let length: f64 = axis.windows(2).map(|w| if let [a, b] = w { (b.x - a.x).hypot(b.y - a.y) } else { 0.0 }).sum();
    if !(length.is_finite() && length > 0.0) {
        return Err(bad(CMD, "the axis has no length"));
    }
    if length / step > 5_000.0 {
        return Err(bad(CMD, "too many cross sections: use a larger `step`"));
    }
    let tris = tin(s, &space)?;
    if tris.is_empty() {
        return Err(bad(CMD, "no terrain model: run GEODMT first"));
    }
    let locate = Locator::new(&tris).ok_or_else(|| bad(CMD, "the terrain model is too large"))?;
    // Sections from the upstream end down, each as its cut line and the terrain along it.
    let mut marks: Vec<f64> = (0..=((length / step).floor() as usize)).map(|i| i as f64 * step).collect();
    if marks.last().is_some_and(|l| length - l > 1e-6) {
        marks.push(length);
    }
    let mut sections: Vec<(f64, [Vec2; 2], Vec<Vec3>)> = Vec::new();
    for d in &marks {
        let Some((c, u)) = frame_at(&axis, *d) else { continue };
        // Left bank first, looking downstream.
        let n = Vec2::new(-u.y, u.x);
        let cut = [Vec2::new(c.x + n.x * width, c.y + n.y * width), Vec2::new(c.x - n.x * width, c.y - n.y * width)];
        let line: Vec<Vec3> = section(&tris, &cut)
            .into_iter()
            .map(|(st, z)| {
                let q = along(&cut, st);
                Vec3::new(q.x, q.y, z)
            })
            .collect();
        if line.len() >= 2 {
            sections.push((*d, cut, line));
        }
    }
    if sections.is_empty() {
        return Err(bad(CMD, "no cross section crosses the terrain model"));
    }
    let z_axis = |q: Vec2| locate.height(q).unwrap_or(0.0);
    let (Some(first), Some(last)) = (axis.first().copied(), axis.last().copied()) else { return Err(bad(CMD, "the axis has no vertices")) };
    let mut out = String::new();
    let (lo, hi) =
        tris.iter().flatten().fold((Vec2::new(f64::INFINITY, f64::INFINITY), Vec2::new(f64::NEG_INFINITY, f64::NEG_INFINITY)), |(l, h), v| {
            (Vec2::new(l.x.min(v.x), l.y.min(v.y)), Vec2::new(h.x.max(v.x), h.y.max(v.y)))
        });
    out.push_str("BEGIN HEADER:\n");
    out.push_str("  UNITS: METRIC\n  DTM TYPE: TIN\n  DTM: GEO_DMT\n  STREAM LAYER: osa\n");
    out.push_str("  NUMBER OF REACHES: 1\n  CROSS-SECTION LAYER: pricne_rezy\n");
    out.push_str(&format!("  NUMBER OF CROSS-SECTIONS: {}\n", sections.len()));
    out.push_str("  MAP PROJECTION: \n  PROJECTION ZONE: \n  DATUM: \n  VERTICAL DATUM: \n");
    out.push_str(&format!(
        "  BEGIN SPATIAL EXTENT:\n    XMIN: {:.3}\n    YMIN: {:.3}\n    XMAX: {:.3}\n    YMAX: {:.3}\n  END SPATIAL EXTENT:\n",
        lo.x, lo.y, hi.x, hi.y
    ));
    out.push_str("END HEADER:\n\nBEGIN STREAM NETWORK:\n");
    out.push_str(&format!("  ENDPOINT: {:.3}, {:.3}, {:.3}, 1\n", first.x, first.y, z_axis(first)));
    out.push_str(&format!("  ENDPOINT: {:.3}, {:.3}, {:.3}, 2\n", last.x, last.y, z_axis(last)));
    out.push_str(&format!("  REACH:\n    STREAM ID: {river}\n    REACH ID: {reach}\n    FROM POINT: 1\n    TO POINT: 2\n    CENTERLINE:\n"));
    // River stations count up from the downstream end.
    let mut run = 0.0;
    let mut prev: Option<Vec2> = None;
    for q in &axis {
        if let Some(a) = prev {
            run += (q.x - a.x).hypot(q.y - a.y);
        }
        prev = Some(*q);
        out.push_str(&format!("      {:.3}, {:.3}, {:.3}, {:.3}\n", q.x, q.y, z_axis(*q), length - run));
    }
    out.push_str("  END:\nEND STREAM NETWORK:\n\nBEGIN CROSS-SECTIONS:\n");
    for (i, (d, cut, line)) in sections.iter().enumerate() {
        let down = sections.get(i + 1).map_or(0.0, |n| n.0 - d);
        out.push_str(&format!("  CROSS-SECTION:\n    STREAM ID: {river}\n    REACH ID: {reach}\n    STATION: {:.3}\n    NODE NAME: \n", length - d));
        out.push_str("    BANK POSITIONS: 0.45, 0.55\n");
        out.push_str(&format!("    REACH LENGTHS: {down:.3}, {down:.3}, {down:.3}\n"));
        out.push_str("    NVALUES:\n    LEVEE POSITIONS:\n    INEFFECTIVE POSITIONS:\n    BLOCKED POSITIONS:\n    CUT LINE:\n");
        for q in cut {
            out.push_str(&format!("      {:.3}, {:.3}\n", q.x, q.y));
        }
        out.push_str("    SURFACE LINE:\n");
        for q in line {
            out.push_str(&format!("      {:.3}, {:.3}, {:.3}\n", q.x, q.y, q.z));
        }
        out.push_str("  END:\n");
    }
    out.push_str("END CROSS-SECTIONS:\n");
    let n = sections.len();
    let msg = format!("HEC-RAS geometry: {n} cross sections along {length:.1} m (bank positions are placeholders: set them in HEC-RAS)");
    match str_param(p, "path") {
        Some(path) => {
            #[cfg(not(target_arch = "wasm32"))]
            {
                std::fs::write(path, out.as_bytes()).map_err(|e| bad(CMD, format!("{path}: {e}")))?;
                Ok(json!({ "sections": n, "length": length, "message": format!("{msg}, written to {path}") }))
            }
            #[cfg(target_arch = "wasm32")]
            {
                let _ = (out, msg);
                Err(bad(CMD, format!("{path}: files can't be written on the web; leave out `path`")))
            }
        }
        None => Ok(json!({ "sections": n, "length": length, "text": out, "message": msg })),
    }
}

// --- prompts -------------------------------------------------------------------------------------

struct FileM;

impl Interactive for FileM {
    fn name(&self) -> &'static str {
        "GEOIMPORTXML"
    }
    fn prompt(&self, _s: &Session) -> Prompt {
        Prompt::new("LandXML file", Accept::TEXT)
    }
    fn input(&mut self, s: &mut Session, i: Input) -> Result<Step> {
        match i {
            Input::Cancel | Input::Enter => Ok(Step::Done),
            Input::Text(t) | Input::Keyword(t) => {
                let r = run_geoimportxml(s, &json!({ "path": t.trim().trim_matches('"') }));
                say(s, r);
                Ok(Step::Done)
            }
            _ => Ok(Step::Continue),
        }
    }
}

#[derive(Default)]
struct RasM {
    axis: Option<Handle>,
    step: Option<f64>,
    width: Option<f64>,
}

impl Interactive for RasM {
    fn name(&self) -> &'static str {
        "GEOHECRAS"
    }
    fn prompt(&self, _s: &Session) -> Prompt {
        match (self.axis, self.step, self.width) {
            (None, ..) => Prompt::new("Select the river axis (drawn downstream)", Accept::SELECT),
            (_, None, _) => Prompt::new("Distance between cross sections (m)", Accept::NUMBER).default("20"),
            (_, _, None) => Prompt::new("Width to each side (m)", Accept::NUMBER).default("30"),
            _ => Prompt::new("File to write (.geo)", Accept::TEXT),
        }
    }
    fn input(&mut self, s: &mut Session, i: Input) -> Result<Step> {
        if matches!(i, Input::Cancel) {
            return Ok(Step::Cancel);
        }
        let Some(axis) = self.axis else {
            if let Input::Pick(hs) = i {
                self.axis = hs.first().copied();
            }
            return Ok(Step::Continue);
        };
        let number = match &i {
            Input::Text(t) | Input::Keyword(t) => typed_number(t).filter(|v| *v > 0.0),
            _ => None,
        };
        if self.step.is_none() || self.width.is_none() {
            let default = if self.step.is_none() { 20.0 } else { 30.0 };
            let v = match (&i, number) {
                (Input::Enter, _) => default,
                (_, Some(v)) => v,
                _ => {
                    s.echo("Enter a distance in metres.");
                    return Ok(Step::Continue);
                }
            };
            if self.step.is_none() {
                self.step = Some(v)
            } else {
                self.width = Some(v)
            }
            return Ok(Step::Continue);
        }
        let file = match i {
            Input::Text(t) | Input::Keyword(t) if !t.trim().is_empty() => t.trim().trim_matches('"').to_string(),
            Input::Enter => return Ok(Step::Done),
            _ => return Ok(Step::Continue),
        };
        let r = run_geohecras(s, &json!({ "axis": axis.hex(), "step": self.step, "width": self.width, "path": file }));
        say(s, r);
        Ok(Step::Done)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cadcraft_doc::{LwPolyline, Point};
    use cadcraft_geom::PolyVertex;

    /// An `n` × `n` grid of cells of `size` from (x0, y0), as triangles of z = f(x, y).
    fn grid(x0: f64, y0: f64, size: f64, n: usize, f: impl Fn(f64, f64) -> f64) -> Vec<[Vec3; 3]> {
        let v = |i: usize, j: usize| {
            let (x, y) = (x0 + i as f64 * size, y0 + j as f64 * size);
            Vec3::new(x, y, f(x, y))
        };
        let mut out = Vec::new();
        for i in 0..n {
            for j in 0..n {
                let (a, b, c, d) = (v(i, j), v(i + 1, j), v(i + 1, j + 1), v(i, j + 1));
                out.push([a, b, c]);
                out.push([a, c, d]);
            }
        }
        out
    }

    fn faces(s: &mut Session, layer: &str, tris: &[[Vec3; 3]]) {
        let space = s.space();
        for t in tris {
            add(s, &space, layer, EntityKind::Face3d(Face3d { corners: [t[0], t[1], t[2], t[2]], hidden_edges: 0 })).unwrap();
        }
    }

    fn area(tris: &[[Vec3; 3]]) -> f64 {
        tris.iter().map(|t| ((t[1].x - t[0].x) * (t[2].y - t[0].y) - (t[1].y - t[0].y) * (t[2].x - t[0].x)).abs() / 2.0).sum()
    }

    #[test]
    fn a_platform_merges_into_the_ground() {
        for shift in [Vec2::ZERO, Vec2::new(-742_000.0, -1_043_000.0)] {
            // Ground: 100 × 100 m at 300 m; a 20 × 20 m platform at 305 m in the middle.
            let ground = grid(shift.x, shift.y, 10.0, 10, |_, _| 300.0);
            let platform = grid(shift.x + 40.0, shift.y + 40.0, 10.0, 2, |_, _| 305.0);
            let (tris, n_design, n_existing, n_join) = merge(&ground, &platform).unwrap();
            assert_eq!(n_design, 8, "{n_design} {n_existing} {n_join}");
            assert!(n_join > 0 && n_existing > 100, "{n_design} {n_existing} {n_join}");
            // The merged model covers the ground exactly once.
            assert!((area(&tris) - 10_000.0).abs() < 1e-6, "{}", area(&tris));
            let m = Locator::new(&tris).unwrap();
            let h = |x: f64, y: f64| m.height(Vec2::new(shift.x + x, shift.y + y)).unwrap();
            assert!((h(50.0, 50.0) - 305.0).abs() < 1e-9 && (h(45.0, 57.0) - 305.0).abs() < 1e-9);
            assert!((h(5.0, 5.0) - 300.0).abs() < 1e-9 && (h(95.0, 20.0) - 300.0).abs() < 1e-9);
            // Between the platform's edge (x = 60) and the next ground points (x = 70): a slope.
            let mid = h(65.0, 50.0);
            assert!(mid > 300.0 && mid < 305.0, "{mid}");
        }
    }

    #[test]
    fn the_merge_command_writes_its_layer() {
        let mut s = Session::new();
        assert!(s.execute("geosloucit", &json!({})).is_err());
        faces(&mut s, LAYER_TIN, &grid(0.0, 0.0, 10.0, 6, |x, _| 300.0 + x / 10.0));
        assert!(s.execute("geosloucit", &json!({})).is_err());
        faces(&mut s, LAYER_DESIGN, &grid(20.0, 20.0, 10.0, 2, |_, _| 310.0));
        let r = s.execute("geosloucit", &json!({})).unwrap();
        assert_eq!(r["layer"], LAYER_MERGED);
        assert_eq!(r["design"], 8, "{r}");
        let n = r["triangles"].as_u64().unwrap() as usize;
        assert_eq!(s.doc().unwrap().model.iter().filter(|e| e.common.layer == LAYER_MERGED).count(), n);
        // Again: replaced, not doubled; and into the existing layer on request.
        s.execute("geosloucit", &json!({})).unwrap();
        assert_eq!(s.doc().unwrap().model.iter().filter(|e| e.common.layer == LAYER_MERGED).count(), n);
        let r = s.execute("geosloucit", &json!({ "replaceExisting": true })).unwrap();
        assert_eq!(r["layer"], LAYER_TIN);
        assert_eq!(s.doc().unwrap().model.iter().filter(|e| e.common.layer == LAYER_TIN).count(), r["triangles"].as_u64().unwrap() as usize);
    }

    #[test]
    fn lines_and_points_are_draped() {
        let mut s = Session::new();
        faces(&mut s, LAYER_TIN, &grid(0.0, 0.0, 10.0, 4, |x, y| 100.0 + x / 10.0 + y / 20.0));
        let space = s.space();
        let pl = LwPolyline {
            vertices: [(5.0, 5.0), (35.0, 5.0), (35.0, 25.0)].iter().map(|(x, y)| PolyVertex::new(Vec2::new(*x, *y))).collect(),
            closed: false,
            const_width: 0.0,
            elevation: 0.0,
            plinegen: false,
        };
        let line = add(&mut s, &space, "0", EntityKind::LwPolyline(pl)).unwrap();
        let pt = add(&mut s, &space, "0", EntityKind::Point(Point { p: Vec3::new(20.0, 20.0, 0.0), angle: 0.0 })).unwrap();
        let far = add(&mut s, &space, "0", EntityKind::Point(Point { p: Vec3::new(200.0, 20.0, 0.0), angle: 0.0 })).unwrap();
        let r = s.execute("geopoloz", &json!({ "handles": [line.hex(), pt.hex(), far.hex()] })).unwrap();
        assert_eq!((r["lines"].as_u64(), r["points"].as_u64(), r["outside"].as_u64()), (Some(1), Some(1), Some(1)), "{r}");
        let d = s.doc().unwrap();
        let EntityKind::Point(q) = &d.model.get(pt).unwrap().kind else { panic!("point") };
        assert!((q.p.z - 103.0).abs() < 1e-9, "{}", q.p.z);
        let draped = d
            .model
            .iter()
            .find_map(
                |e| if let (EntityKind::Polyline3d(p), true) = (&e.kind, e.common.layer == LAYER_DRAPED) { Some(p.points.clone()) } else { None },
            )
            .unwrap();
        // More vertices than the 2D line had, each at the plane's height, ends in place.
        assert!(draped.len() > 3, "{draped:?}");
        assert!(draped.iter().all(|v| (v.z - (100.0 + v.x / 10.0 + v.y / 20.0)).abs() < 1e-9), "{draped:?}");
        assert!((draped[0].x - 5.0).abs() < 1e-9 && (draped.last().unwrap().y - 25.0).abs() < 1e-9);
        // The original stays unless asked.
        assert!(d.model.get(line).is_some());
        assert!(s.execute("geopoloz", &json!({ "handles": [far.hex()] })).is_err());
    }

    #[test]
    fn landxml_round_trip() {
        let mut s = Session::new();
        s.execute("geobody", &json!({ "text": "1 743000 1043000 300\n2 743010 1043000 301\n3 743000 1043010 302\n4 743010 1043010 303\n" })).unwrap();
        s.execute("geodmt", &json!({})).unwrap();
        let xml = s.execute("geolandxml", &json!({})).unwrap()["text"].as_str().unwrap().to_string();
        let land = parse_landxml(&xml).unwrap();
        assert_eq!(land.points.len(), 4);
        assert_eq!(land.surfaces.len(), 1);
        assert_eq!(land.surfaces[0].1.len(), 2);
        let mut t = Session::new();
        let r = t.execute("geoimportxml", &json!({ "text": xml })).unwrap();
        assert_eq!((r["points"].as_u64(), r["surfaces"].as_u64(), r["triangles"].as_u64()), (Some(4), Some(1), Some(2)), "{r}");
        // The same points at the same places, under their numbers.
        let space = t.space();
        let pts = super::super::geo_pohled::numbered_points(&t, &space, None).unwrap();
        let p3 = pts.iter().find(|(n, _)| n == "3").unwrap().1;
        assert!((p3.x + 743000.0).abs() < 1e-6 && (p3.y + 1043010.0).abs() < 1e-6 && (p3.z - 302.0).abs() < 1e-9, "{p3:?}");
        assert_eq!(super::super::geo_calc::tin(&t, &space).unwrap().len(), 2);
    }

    #[test]
    fn landxml_from_other_programs() {
        // Namespaces, attributes in single quotes, an invisible face, a comment, local coordinates.
        let xml = "<?xml version='1.0'?><!-- made <elsewhere> --><lx:LandXML xmlns:lx='x'><lx:CgPoints><lx:CgPoint name='A &amp; B' code=\"x>y\">20 10 5</lx:CgPoint><lx:CgPoint name='noz'>21 11</lx:CgPoint><lx:CgPoint pntRef='A'/></lx:CgPoints>\
<lx:Surfaces><lx:Surface name='Navrh 1'><lx:Definition surfType='TIN'><lx:Pnts><lx:P id='1'>0 0 1</lx:P><lx:P id='2'>0 10 1</lx:P><lx:P id='3'>10 10 2</lx:P><lx:P id='4'>10 0 2</lx:P></lx:Pnts>\
<lx:Faces><lx:F>1 2 3</lx:F><lx:F i='1'>1 3 4</lx:F><lx:F n='0 1 0'>1 3 4</lx:F><lx:F>1 2 99</lx:F></lx:Faces></lx:Definition></lx:Surface></lx:Surfaces></lx:LandXML>";
        let land = parse_landxml(xml).unwrap();
        assert_eq!(land.points.len(), 2);
        assert_eq!(land.points[0], ("A & B".to_string(), 20.0, 10.0, Some(5.0)));
        assert_eq!(land.surfaces[0].0, "Navrh 1");
        assert_eq!(land.surfaces[0].1.len(), 2);
        let mut s = Session::new();
        let r = s.execute("geoimportxml", &json!({ "text": xml })).unwrap();
        assert_eq!(r["layers"], json!([LAYER_TIN]));
        // Local coordinates: easting is x, northing is y.
        let space = s.space();
        let t = super::super::geo_calc::tin(&s, &space).unwrap();
        assert!(t.iter().flatten().any(|v| (v.x - 10.0).abs() < 1e-9 && v.y.abs() < 1e-9 && (v.z - 1.0).abs() < 1e-9), "{t:?}");
    }

    #[test]
    fn hec_ras_geometry() {
        let mut s = Session::new();
        // A valley along x: the bed falls 1 m per 100 m, the banks rise 1:5 to either side.
        faces(&mut s, LAYER_TIN, &grid(0.0, -50.0, 10.0, 10, |x, y| 200.0 - x / 100.0 + y.abs() / 5.0));
        let space = s.space();
        let pl = LwPolyline {
            vertices: [(10.0, 0.0), (90.0, 0.0)].iter().map(|(x, y)| PolyVertex::new(Vec2::new(*x, *y))).collect(),
            closed: false,
            const_width: 0.0,
            elevation: 0.0,
            plinegen: false,
        };
        let axis = add(&mut s, &space, "0", EntityKind::LwPolyline(pl)).unwrap();
        let r = s.execute("geohecras", &json!({ "axis": axis.hex(), "step": 40, "width": 20, "river": "Botič" })).unwrap();
        assert_eq!(r["sections"], 3, "{r}");
        let t = r["text"].as_str().unwrap();
        assert!(t.starts_with("BEGIN HEADER:") && t.contains("NUMBER OF CROSS-SECTIONS: 3") && t.contains("STREAM ID: Botič"), "{t}");
        // Stations count from the downstream end: 80, 40, 0; the first lies 40 m above the next.
        let stations: Vec<&str> = t.lines().filter_map(|l| l.trim().strip_prefix("STATION: ")).collect();
        assert_eq!(stations, ["80.000", "40.000", "0.000"]);
        assert!(t.contains("REACH LENGTHS: 40.000, 40.000, 40.000") && t.contains("REACH LENGTHS: 0.000, 0.000, 0.000"));
        // The first cut line runs from the left bank (north, looking east) to the right.
        assert!(t.contains("CUT LINE:\n      10.000, 20.000\n      10.000, -20.000\n"), "{t}");
        // Its surface line starts 4 m above the bed at x = 10 (199.9).
        assert!(t.contains("SURFACE LINE:\n      10.000, 20.000, 203.900\n"), "{t}");
        assert_eq!(t.matches("CROSS-SECTION:").count(), 3);
        assert_eq!(t.lines().filter(|l| l.trim() == "END:").count(), 4);
    }

    #[test]
    fn hostile_input_is_refused_not_panicking() {
        let mut s = Session::new();
        for (c, p) in [
            ("geopoloz", json!({})),
            ("geoimportxml", json!({})),
            ("geoimportxml", json!({ "text": "" })),
            ("geoimportxml", json!({ "text": "<LandXML><CgPoints><CgPoint>x y</CgPoint></CgPoints></LandXML>" })),
            ("geoimportxml", json!({ "text": "<a b='" })),
            ("geoimportxml", json!({ "path": "/nonexistent.xml" })),
            ("geohecras", json!({})),
            ("geohecras", json!({ "axis": "zz" })),
            ("geosloucit", json!({ "existing": "a/b" })),
        ] {
            assert!(s.execute(c, &p).is_err(), "{c} {p}");
        }
        for junk in
            ["<", "<<<>>>", "<P id=>1 2 3</P>", "<Surface><P id='1'>1 2 3</P><F>1 1 1</F>", "<!-- never closed", "<a 'b>c'>", "</>", "<F/><P/>"]
        {
            let _ = parse_landxml(junk);
        }
        assert!(attr(" name='a' id=\"7\"", "id") == Some("7") && attr(" pid='1'", "id").is_none() && attr(" id", "id").is_none());
        let nan = [[Vec3::new(f64::NAN, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0)]];
        let ok = grid(0.0, 0.0, 1.0, 1, |_, _| 0.0);
        let _ = merge(&nan, &ok);
        let _ = merge(&ok, &nan);
        assert!(merge(&ok, &[]).is_err());
        let _ = Locator::new(&nan);
        let _ = Locator::new(&ok).map(|l| l.height(Vec2::new(f64::NAN, 0.0)));
    }
}
