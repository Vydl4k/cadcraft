//! Object snaps on raster images: points the picture itself shows. Around the cursor the image's
//! pixels are split into ink and paper, the ink is thinned to one-pixel lines, and the ends of
//! those lines (Endpoint), their bends (Endpoint) and the places where they meet or cross
//! (Intersection) become snap points; with only Node on, all of them are Nodes. Only a small
//! window round the cursor is looked at (sampled more coarsely the further out the view is
//! zoomed), so a scan of any size snaps as fast as a small one, and nothing is kept between
//! queries.

use cadcraft_doc::{Drawing, Image};
use cadcraft_geom::Vec2;

use super::mode;

/// Cells per side of the window examined (after sampling).
const WINDOW: usize = 160;
/// Thinning passes, at most.
const MAX_PASSES: usize = 64;
/// Images looked at per query.
pub(super) const MAX_IMAGES: usize = 16;
/// Steps traced along one line.
const MAX_TRACE: usize = 4 * WINDOW * WINDOW;

/// Snap candidates `img` offers within `aperture` of `cursor`.
pub(super) fn nodes(d: &Drawing, img: &Image, cursor: Vec2, aperture: f64, osmode: u32) -> Vec<(u32, Vec2)> {
    if osmode & (mode::END | mode::INT | mode::NOD) == 0 || img.display & 1 == 0 {
        return Vec::new();
    }
    let Some(raster) = d.rasters.get(&img.path) else { return Vec::new() };
    let (rw, rh) = (raster.width() as usize, raster.height() as usize);
    let (iw, ih) = (img.size.x, img.size.y);
    let (u, v) = (img.u.xy(), img.v.xy());
    let det = u.x * v.y - u.y * v.x;
    if rw == 0 || rh == 0 || !(iw.is_finite() && ih.is_finite() && iw >= 1.0 && ih >= 1.0) || !det.is_finite() || det.abs() < 1e-30 {
        return Vec::new();
    }
    // Raster pixels per image pixel (a loaded picture may be a reduced copy).
    let (sx, sy) = (rw as f64 / iw, rh as f64 / ih);
    let o = img.insert.xy();
    let to_image = |p: Vec2| {
        let (dx, dy) = (p.x - o.x, p.y - o.y);
        let a = (dx * v.y - dy * v.x) / det;
        let b = (u.x * dy - u.y * dx) / det;
        (a, ih - b)
    };
    let to_world = |px: f64, py: f64| {
        let b = ih - py;
        Vec2::new(o.x + u.x * px + v.x * b, o.y + u.y * px + v.y * b)
    };
    let (cx, cy) = to_image(cursor);
    let pixel = u.x.hypot(u.y).min(v.x.hypot(v.y));
    if !(pixel.is_finite() && pixel > 0.0) {
        return Vec::new();
    }
    // The window: twice the aperture each way, in raster pixels.
    let reach = (2.0 * aperture / pixel).max(6.0);
    let (rx, ry) = (cx * sx, cy * sy);
    let reach_r = reach * sx.max(sy);
    if !(rx.is_finite() && ry.is_finite() && reach_r.is_finite())
        || rx < -reach_r
        || ry < -reach_r
        || rx > rw as f64 + reach_r
        || ry > rh as f64 + reach_r
    {
        return Vec::new();
    }
    let x0 = (rx - reach_r).floor().max(0.0) as usize;
    let y0 = (ry - reach_r).floor().max(0.0) as usize;
    let x1 = ((rx + reach_r).ceil().max(0.0) as usize).min(rw);
    let y1 = ((ry + reach_r).ceil().max(0.0) as usize).min(rh);
    if x1 <= x0 + 2 || y1 <= y0 + 2 {
        return Vec::new();
    }
    let step = (x1 - x0).max(y1 - y0).div_ceil(WINDOW).max(1);
    let (w, h) = ((x1 - x0).div_ceil(step), (y1 - y0).div_ceil(step));
    // Each cell's darkest pixel, so thin lines survive the sampling.
    let mut lum = vec![255u8; w * h];
    let rgba = raster.rgba();
    for cyi in 0..h {
        for cxi in 0..w {
            let mut darkest = 255u8;
            for yy in (y0 + cyi * step)..(y0 + (cyi + 1) * step).min(y1) {
                for xx in (x0 + cxi * step)..(x0 + (cxi + 1) * step).min(x1) {
                    let k = (yy * rw + xx) * 4;
                    if let Some(px) = rgba.get(k..k + 4) {
                        let (r, g, b, a) = (u32::from(px[0]), u32::from(px[1]), u32::from(px[2]), u32::from(px[3]));
                        // Over white paper (premultiplied: add what alpha leaves).
                        let l = (r * 299 + g * 587 + b * 114) / 1000 + (255 - a);
                        darkest = darkest.min(l.min(255) as u8);
                    }
                }
            }
            if let Some(c) = lum.get_mut(cyi * w + cxi) {
                *c = darkest;
            }
        }
    }
    let Some(ink) = binarize(&lum) else { return Vec::new() };
    let mut bits: Vec<bool> = lum.iter().map(|l| *l < ink).collect();
    // Mostly ink is a filled area or a dark photo: nothing to snap to.
    if bits.iter().filter(|b| **b).count() * 5 > bits.len() * 3 {
        return Vec::new();
    }
    thin(&mut bits, w, h);
    let found = features(&bits, w, h);
    // Points at the window's cut edges are where the window ends, not the drawing.
    let cut = |x: usize, y: usize| (x <= 1 && x0 > 0) || (y <= 1 && y0 > 0) || (x + 2 >= w && x1 < rw) || (y + 2 >= h && y1 < rh);
    let mut out = Vec::new();
    for (x, y, kind) in found {
        if cut(x, y) {
            continue;
        }
        let px = (x0 as f64 + (x as f64 + 0.5) * step as f64) / sx;
        let py = (y0 as f64 + (y as f64 + 0.5) * step as f64) / sy;
        let p = to_world(px, py);
        if !p.is_finite() || p.dist(cursor) > aperture {
            continue;
        }
        let m = match kind {
            Feature::Junction if osmode & mode::INT != 0 => mode::INT,
            _ if osmode & mode::END != 0 => mode::END,
            _ if osmode & mode::NOD != 0 => mode::NOD,
            _ => continue,
        };
        out.push((m, p));
    }
    out
}

/// The ink threshold (Otsu's), or `None` for a window with no clear ink on paper.
fn binarize(lum: &[u8]) -> Option<u8> {
    let mut hist = [0u64; 256];
    for l in lum {
        if let Some(c) = hist.get_mut(usize::from(*l)) {
            *c += 1;
        }
    }
    let (lo, hi) = (lum.iter().copied().min()?, lum.iter().copied().max()?);
    if hi.saturating_sub(lo) < 60 {
        return None;
    }
    let total = lum.len() as f64;
    let sum: f64 = hist.iter().enumerate().map(|(i, c)| i as f64 * *c as f64).sum();
    let (mut wb, mut sb, mut best, mut at) = (0.0, 0.0, -1.0, 128usize);
    for (t, c) in hist.iter().enumerate() {
        wb += *c as f64;
        if wb == 0.0 {
            continue;
        }
        let wf = total - wb;
        if wf == 0.0 {
            break;
        }
        sb += t as f64 * *c as f64;
        let (mb, mf) = (sb / wb, (sum - sb) / wf);
        let between = wb * wf * (mb - mf) * (mb - mf);
        if between > best {
            best = between;
            at = t;
        }
    }
    u8::try_from(at + 1).ok()
}

fn get(bits: &[bool], w: usize, h: usize, x: isize, y: isize) -> bool {
    x >= 0 && y >= 0 && (x as usize) < w && (y as usize) < h && bits.get(y as usize * w + x as usize).copied().unwrap_or(false)
}

/// The 8 neighbours clockwise from north: P2..P9 of the Zhang–Suen thinning.
const RING: [(isize, isize); 8] = [(0, -1), (1, -1), (1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1)];

fn ring(bits: &[bool], w: usize, h: usize, x: usize, y: usize) -> [bool; 8] {
    let mut r = [false; 8];
    for (k, (dx, dy)) in RING.iter().enumerate() {
        if let Some(c) = r.get_mut(k) {
            *c = get(bits, w, h, x as isize + dx, y as isize + dy);
        }
    }
    r
}

/// Ink to paper changes going round a pixel.
fn crossings(r: &[bool; 8]) -> usize {
    (0..8).filter(|k| !r[*k] && r[(k + 1) % 8]).count()
}

/// Zhang–Suen thinning: ink becomes lines one pixel wide.
fn thin(bits: &mut [bool], w: usize, h: usize) {
    for _ in 0..MAX_PASSES {
        let mut changed = false;
        for sub in 0..2 {
            let mut gone = Vec::new();
            for y in 0..h {
                for x in 0..w {
                    if !bits.get(y * w + x).copied().unwrap_or(false) {
                        continue;
                    }
                    let r = ring(bits, w, h, x, y);
                    let n = r.iter().filter(|b| **b).count();
                    if !(2..=6).contains(&n) || crossings(&r) != 1 {
                        continue;
                    }
                    let [p2, _, p4, _, p6, _, p8, _] = r;
                    let ok = if sub == 0 { !(p2 && p4 && p6) && !(p4 && p6 && p8) } else { !(p2 && p4 && p8) && !(p2 && p6 && p8) };
                    if ok {
                        gone.push(y * w + x);
                    }
                }
            }
            for k in &gone {
                if let Some(b) = bits.get_mut(*k) {
                    *b = false;
                }
            }
            changed |= !gone.is_empty();
        }
        if !changed {
            break;
        }
    }
}

#[derive(Clone, Copy, PartialEq, Debug)]
enum Feature {
    End,
    Corner,
    Junction,
}

/// Line ends, junctions and bends of a thinned picture, as cells.
fn features(bits: &[bool], w: usize, h: usize) -> Vec<(usize, usize, Feature)> {
    let mut raw: Vec<(f64, f64, Feature)> = Vec::new();
    let mut node = vec![false; w * h];
    for y in 0..h {
        for x in 0..w {
            if !bits.get(y * w + x).copied().unwrap_or(false) {
                continue;
            }
            let r = ring(bits, w, h, x, y);
            let n = r.iter().filter(|b| **b).count();
            let c = crossings(&r);
            let kind = if n == 1 || (c == 1 && n <= 2) {
                Some(Feature::End)
            } else if c >= 3 {
                Some(Feature::Junction)
            } else {
                None
            };
            if let Some(k) = kind {
                raw.push((x as f64, y as f64, k));
                if let Some(m) = node.get_mut(y * w + x) {
                    *m = true;
                }
            }
        }
    }
    raw.extend(corners(bits, &node, w, h).into_iter().map(|(x, y)| (x as f64, y as f64, Feature::Corner)));
    // A junction is often a few pixels: one point each (the kinds' strongest wins).
    let mut out: Vec<(f64, f64, Feature, f64)> = Vec::new();
    for (x, y, k) in raw {
        match out.iter_mut().find(|o| (o.0 / o.3 - x).abs() <= 2.5 && (o.1 / o.3 - y).abs() <= 2.5) {
            Some(o) => {
                o.0 += x;
                o.1 += y;
                o.3 += 1.0;
                if k == Feature::Junction || (k == Feature::End && o.2 == Feature::Corner) {
                    o.2 = k;
                }
            }
            None => out.push((x, y, k, 1.0)),
        }
        if out.len() > 4096 {
            break;
        }
    }
    out.into_iter().map(|(x, y, k, n)| ((x / n).round().max(0.0) as usize, (y / n).round().max(0.0) as usize, k)).collect()
}

/// Bends along the thinned lines: each line between ends and junctions (or each closed ring) is
/// simplified, and a vertex where it turns by more than 30° is a corner.
fn corners(bits: &[bool], node: &[bool], w: usize, h: usize) -> Vec<(usize, usize)> {
    let mut seen = vec![false; w * h];
    let mut out = Vec::new();
    let mut steps = 0usize;
    let ink = |x: isize, y: isize| get(bits, w, h, x, y);
    let is_node = |x: isize, y: isize| {
        x >= 0 && y >= 0 && (x as usize) < w && (y as usize) < h && node.get(y as usize * w + x as usize).copied().unwrap_or(false)
    };
    for sy in 0..h {
        for sx in 0..w {
            let k = sy * w + sx;
            if !bits.get(k).copied().unwrap_or(false) || seen.get(k).copied().unwrap_or(true) || node.get(k).copied().unwrap_or(true) {
                continue;
            }
            // Walk both ways from here to the line's ends (or round the ring).
            let mut halves: Vec<Vec<(isize, isize)>> = Vec::new();
            if let Some(s) = seen.get_mut(k) {
                *s = true;
            }
            let start = (sx as isize, sy as isize);
            let firsts: Vec<(isize, isize)> = RING.iter().map(|(dx, dy)| (start.0 + dx, start.1 + dy)).filter(|p| ink(p.0, p.1)).take(2).collect();
            for f in firsts {
                let mut path = Vec::new();
                let mut cur = f;
                let mut prev = start;
                loop {
                    steps += 1;
                    if steps > MAX_TRACE {
                        return out;
                    }
                    path.push(cur);
                    let ck = cur.1 as usize * w + cur.0 as usize;
                    if is_node(cur.0, cur.1) || seen.get(ck).copied().unwrap_or(true) {
                        break;
                    }
                    if let Some(s) = seen.get_mut(ck) {
                        *s = true;
                    }
                    let next = RING.iter().map(|(dx, dy)| (cur.0 + dx, cur.1 + dy)).filter(|p| *p != prev && ink(p.0, p.1)).find(|p| {
                        let pk = p.1 as usize * w + p.0 as usize;
                        is_node(p.0, p.1) || !seen.get(pk).copied().unwrap_or(true)
                    });
                    match next {
                        Some(n) => {
                            prev = cur;
                            cur = n;
                        }
                        None => break,
                    }
                }
                halves.push(path);
            }
            let mut line: Vec<(isize, isize)> = halves.first().map(|p| p.iter().rev().copied().collect()).unwrap_or_default();
            line.push(start);
            if let Some(p) = halves.get(1) {
                line.extend(p.iter().copied());
            }
            if line.len() < 7 {
                continue;
            }
            let pts: Vec<Vec2> = line.iter().map(|(x, y)| Vec2::new(*x as f64, *y as f64)).collect();
            let keep = simplify(&pts, 1.2);
            for win in keep.windows(3) {
                let (Some(&a), Some(&b), Some(&c)) = (win.first(), win.get(1), win.get(2)) else { continue };
                let (pa, pb, pc) = (pts[a], pts[b], pts[c]);
                let (d1, d2) = (pb - pa, pc - pb);
                if d1.len() < 4.0 || d2.len() < 4.0 {
                    continue;
                }
                let cos = (d1.x * d2.x + d1.y * d2.y) / (d1.len() * d2.len());
                if cos < 30f64.to_radians().cos() {
                    out.push((pb.x.max(0.0) as usize, pb.y.max(0.0) as usize));
                }
            }
        }
    }
    out
}

/// Douglas–Peucker: the indices of `pts` kept within `eps`.
fn simplify(pts: &[Vec2], eps: f64) -> Vec<usize> {
    let n = pts.len();
    if n < 3 {
        return (0..n).collect();
    }
    let mut keep = vec![false; n];
    keep[0] = true;
    keep[n - 1] = true;
    let mut stack = vec![(0usize, n - 1)];
    while let Some((a, b)) = stack.pop() {
        if b <= a + 1 {
            continue;
        }
        let (pa, pb) = (pts[a], pts[b]);
        let d = pb - pa;
        let l = d.len();
        let (mut far, mut at) = (0.0, a);
        for (i, p) in pts.iter().enumerate().take(b).skip(a + 1) {
            let dist = if l < 1e-9 { p.dist(pa) } else { ((p.x - pa.x) * d.y - (p.y - pa.y) * d.x).abs() / l };
            if dist > far {
                far = dist;
                at = i;
            }
        }
        if far > eps {
            keep[at] = true;
            stack.push((a, at));
            stack.push((at, b));
        }
    }
    (0..n).filter(|i| keep[*i]).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A 100 × 100 white picture with an L of 3-pixel black lines: (20..80, 20) and (20, 20..80).
    fn ell() -> (Vec<bool>, usize, usize) {
        let (w, h) = (100, 100);
        let mut b = vec![false; w * h];
        for i in 20..80 {
            for t in 19..22 {
                b[t * w + i] = true;
                b[i * w + t] = true;
            }
        }
        (b, w, h)
    }

    #[test]
    fn an_l_has_two_ends_and_a_corner() {
        let (mut b, w, h) = ell();
        thin(&mut b, w, h);
        let f = features(&b, w, h);
        let near = |x: f64, y: f64| f.iter().any(|(fx, fy, _)| (*fx as f64 - x).abs() <= 3.0 && (*fy as f64 - y).abs() <= 3.0);
        assert!(near(79.0, 20.0) && near(20.0, 79.0), "ends: {f:?}");
        assert!(near(20.0, 20.0), "corner: {f:?}");
        assert!(f.len() <= 4, "{f:?}");
    }

    #[test]
    fn a_cross_has_a_junction() {
        let (w, h) = (60, 60);
        let mut b = vec![false; w * h];
        for i in 5..55 {
            b[30 * w + i] = true;
            b[i * w + 30] = true;
        }
        thin(&mut b, w, h);
        let f = features(&b, w, h);
        assert!(f.iter().any(|(x, y, k)| *k == Feature::Junction && x.abs_diff(30) <= 2 && y.abs_diff(30) <= 2), "{f:?}");
    }

    #[test]
    fn blank_and_flat_windows_offer_nothing() {
        assert!(binarize(&[200u8; 400]).is_none());
        assert!(binarize(&[]).is_none());
        let mut b = vec![false; 0];
        thin(&mut b, 0, 0);
        assert!(features(&b, 0, 0).is_empty());
    }
}
