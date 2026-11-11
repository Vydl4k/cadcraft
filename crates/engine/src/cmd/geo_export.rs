//! Survey add-on: exchange formats.
//!
//! - GEOLANDXML: the GEO_body points (with their numbers) and the terrain models (GEO_DMT, and
//!   GEO_DMT_navrh when there is one) as LandXML 1.2: CgPoints and TIN surfaces.
//! - GEOKML: the points, and optionally selected lines, for Google Earth (KML, WGS 84). S-JTSK
//!   coordinates are turned into latitude and longitude by the Křovák projection's inverse
//!   (after the EPSG guidance for method 9819) on the Bessel ellipsoid and a three-parameter
//!   datum shift to WGS 84, good to a few metres, which is what a KML overview needs.

use cadcraft_doc::EntityKind;
use cadcraft_geom::{Vec2, Vec3};
use serde_json::{Value, json};

use super::geo_calc::tin_on;
use super::geo_dmt::{LAYER_DESIGN, LAYER_TIN};
use super::geo_pohled::numbered_points;
use super::{CommandSpec, bad, str_param};
use crate::{Result, Session};

/// Output text produced at once (bytes).
const MAX_TEXT: usize = 512 * 1024 * 1024;

pub(super) fn specs() -> Vec<CommandSpec> {
    vec![
        CommandSpec::new("geolandxml", "Export LandXML...", run_geolandxml)
            .menu(&["Geo", "Export", "LandXML..."])
            .alias(&["geoxml"])
            .params("{path?: file to write (otherwise the text is returned)} → {points, surfaces, text?, message}"),
        CommandSpec::new("geokml", "Export KML (Google Earth)...", run_geokml)
            .menu(&["Geo", "Export", "KML (Google Earth)..."])
            .alias(&["geogoogle"])
            .params("{path?: file to write (otherwise the text is returned), handles?: lines and polylines to include as paths (default: the selection)} → {points, paths, text?, message}"),
    ]
}

fn esc(t: &str) -> String {
    t.replace('&', "&amp;").replace('<', "&lt;").replace('>', "&gt;").replace('"', "&quot;")
}

/// Geodetic Y, X of drawing points: S-JTSK drawings (x = −Y, y = −X) are recognised by their
/// coordinates.
fn is_sjtsk(pts: &[Vec2]) -> bool {
    !pts.is_empty() && pts.iter().all(|q| q.x < -300_000.0 && q.y < -900_000.0)
}

fn write_or_return(path: Option<&str>, text: String, cmd: &str, what: Value) -> Result<Value> {
    let mut what = what;
    match path {
        Some(path) => {
            #[cfg(not(target_arch = "wasm32"))]
            {
                std::fs::write(path, text.as_bytes()).map_err(|e| bad(cmd, format!("{path}: {e}")))?;
                if let Some(o) = what.as_object_mut() {
                    let m = o.get("message").and_then(Value::as_str).unwrap_or("").to_string();
                    o.insert("message".into(), json!(format!("{m}, written to {path}")));
                }
                Ok(what)
            }
            #[cfg(target_arch = "wasm32")]
            {
                let _ = (text, what);
                Err(bad(cmd, format!("{path}: files can't be written on the web; leave out `path`")))
            }
        }
        None => {
            if let Some(o) = what.as_object_mut() {
                o.insert("text".into(), json!(text));
            }
            Ok(what)
        }
    }
}

fn run_geolandxml(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "geolandxml";
    let space = s.space();
    let pts = numbered_points(s, &space, None)?;
    let surfaces: Vec<(&str, Vec<[Vec3; 3]>)> =
        [LAYER_TIN, LAYER_DESIGN].into_iter().map(|l| tin_on(s, &space, l).map(|t| (l, t))).collect::<Result<_>>()?;
    if pts.is_empty() && surfaces.iter().all(|(_, t)| t.is_empty()) {
        return Err(bad(CMD, "nothing to export: no points on GEO_body and no terrain model"));
    }
    let plan: Vec<Vec2> = pts.iter().map(|(_, q)| q.xy()).chain(surfaces.iter().flat_map(|(_, t)| t.iter().flatten().map(|v| v.xy()))).collect();
    let sj = is_sjtsk(&plan);
    // LandXML writes northing easting elevation: X and Y for S-JTSK, y and x otherwise.
    let ne = |q: Vec3| if sj { (-q.y, -q.x) } else { (q.y, q.x) };
    let mut out =
        String::from("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<LandXML xmlns=\"http://www.landxml.org/schema/LandXML-1.2\" version=\"1.2\">\n");
    out.push_str("  <Units><Metric linearUnit=\"meter\" areaUnit=\"squareMeter\" volumeUnit=\"cubicMeter\" angularUnit=\"decimal degrees\" directionUnit=\"decimal degrees\"/></Units>\n");
    if sj {
        out.push_str("  <CoordinateSystem name=\"S-JTSK\" epsgCode=\"5514\"/>\n");
    }
    out.push_str("  <Application name=\"CADCraft\" manufacturer=\"CADCraft\"/>\n");
    if !pts.is_empty() {
        out.push_str("  <CgPoints>\n");
        for (num, q) in &pts {
            let (n, e) = ne(*q);
            out.push_str(&format!("    <CgPoint name=\"{}\">{n:.3} {e:.3} {:.3}</CgPoint>\n", esc(num), q.z));
        }
        out.push_str("  </CgPoints>\n");
    }
    let mut count = 0;
    let with_tris: Vec<_> = surfaces.iter().filter(|(_, t)| !t.is_empty()).collect();
    if !with_tris.is_empty() {
        out.push_str("  <Surfaces>\n");
        for (name, tris) in with_tris {
            count += 1;
            // Shared vertices are written once.
            let mut ids: std::collections::HashMap<(i64, i64, i64), usize> = std::collections::HashMap::new();
            let mut verts: Vec<Vec3> = Vec::new();
            let mut faces: Vec<[usize; 3]> = Vec::new();
            let key = |v: Vec3| ((v.x * 1000.0).round() as i64, (v.y * 1000.0).round() as i64, (v.z * 1000.0).round() as i64);
            for t in tris {
                let mut f = [0usize; 3];
                for (slot, v) in f.iter_mut().zip(t.iter()) {
                    *slot = *ids.entry(key(*v)).or_insert_with(|| {
                        verts.push(*v);
                        verts.len()
                    });
                }
                faces.push(f);
            }
            out.push_str(&format!("    <Surface name=\"{}\">\n      <Definition surfType=\"TIN\">\n        <Pnts>\n", esc(name)));
            for (i, v) in verts.iter().enumerate() {
                let (n, e) = ne(*v);
                out.push_str(&format!("          <P id=\"{}\">{n:.3} {e:.3} {:.3}</P>\n", i + 1, v.z));
            }
            out.push_str("        </Pnts>\n        <Faces>\n");
            for f in &faces {
                out.push_str(&format!("          <F>{} {} {}</F>\n", f[0], f[1], f[2]));
            }
            out.push_str("        </Faces>\n      </Definition>\n    </Surface>\n");
            if out.len() > MAX_TEXT {
                return Err(bad(CMD, "the export would be larger than 512 MB"));
            }
        }
        out.push_str("  </Surfaces>\n");
    }
    out.push_str("</LandXML>\n");
    let what = json!({ "points": pts.len(), "surfaces": count, "message": format!("LandXML: {} points, {count} surfaces", pts.len()) });
    write_or_return(str_param(p, "path"), out, CMD, what)
}

// --- S-JTSK → WGS 84 ----------------------------------------------------------------------------

const BESSEL_A: f64 = 6_377_397.155;
const BESSEL_INV_F: f64 = 299.152_812_8;
const WGS_A: f64 = 6_378_137.0;
const WGS_INV_F: f64 = 298.257_223_563;
fn dms(d: f64, m: f64, s: f64) -> f64 {
    (d + m / 60.0 + s / 3600.0).to_radians()
}

/// Latitude and longitude (radians, Greenwich) on the Bessel ellipsoid of the S-JTSK point
/// (Y westing, X southing, both positive), by the inverse Křovák projection. The longitude of
/// origin 24°50′ is Greenwich's (42°30′ east of Ferro).
pub fn krovak_inverse(y: f64, x: f64) -> Option<(f64, f64)> {
    let f = 1.0 / BESSEL_INV_F;
    let e2 = f * (2.0 - f);
    let e = e2.sqrt();
    let phi_c = dms(49.0, 30.0, 0.0);
    let lambda_o = dms(24.0, 50.0, 0.0);
    let alpha_c = dms(30.0, 17.0, 17.3031);
    let phi_p = dms(78.0, 30.0, 0.0);
    let k_p = 0.9999;
    let a_big = BESSEL_A * (1.0 - e2).sqrt() / (1.0 - e2 * phi_c.sin().powi(2));
    let b = (1.0 + e2 * phi_c.cos().powi(4) / (1.0 - e2)).sqrt();
    let gamma0 = (phi_c.sin() / b).asin();
    let t0 = (std::f64::consts::FRAC_PI_4 + gamma0 / 2.0).tan() * ((1.0 + e * phi_c.sin()) / (1.0 - e * phi_c.sin())).powf(e * b / 2.0)
        / (std::f64::consts::FRAC_PI_4 + phi_c / 2.0).tan().powf(b);
    let n = phi_p.sin();
    let r0 = k_p * a_big / phi_p.tan();
    let r = x.hypot(y);
    if !(r.is_finite() && r > 0.0) {
        return None;
    }
    let theta = y.atan2(x);
    let d = theta / n;
    let t = 2.0 * (((r0 / r).powf(1.0 / n) * (std::f64::consts::FRAC_PI_4 + phi_p / 2.0).tan()).atan() - std::f64::consts::FRAC_PI_4);
    let u = (alpha_c.cos() * t.sin() - alpha_c.sin() * t.cos() * d.cos()).asin();
    let v = (t.cos() * d.sin() / u.cos()).asin();
    let mut phi = u;
    for _ in 0..20 {
        let next = 2.0
            * ((t0.powf(-1.0 / b)
                * (u / 2.0 + std::f64::consts::FRAC_PI_4).tan().powf(1.0 / b)
                * ((1.0 + e * phi.sin()) / (1.0 - e * phi.sin())).powf(e / 2.0))
            .atan()
                - std::f64::consts::FRAC_PI_4);
        let done = (next - phi).abs() < 1e-14;
        phi = next;
        if done {
            break;
        }
    }
    let lambda = lambda_o - v / b;
    (phi.is_finite() && lambda.is_finite()).then_some((phi, lambda))
}

fn to_ecef(phi: f64, lam: f64, h: f64, a: f64, inv_f: f64) -> (f64, f64, f64) {
    let f = 1.0 / inv_f;
    let e2 = f * (2.0 - f);
    let n = a / (1.0 - e2 * phi.sin().powi(2)).sqrt();
    ((n + h) * phi.cos() * lam.cos(), (n + h) * phi.cos() * lam.sin(), (n * (1.0 - e2) + h) * phi.sin())
}

fn from_ecef(x: f64, y: f64, z: f64, a: f64, inv_f: f64) -> (f64, f64) {
    let f = 1.0 / inv_f;
    let e2 = f * (2.0 - f);
    let lam = y.atan2(x);
    let p = x.hypot(y);
    let mut phi = z.atan2(p * (1.0 - e2));
    for _ in 0..10 {
        let n = a / (1.0 - e2 * phi.sin().powi(2)).sqrt();
        let h = p / phi.cos() - n;
        phi = z.atan2(p * (1.0 - e2 * n / (n + h)));
    }
    (phi, lam)
}

/// WGS 84 latitude and longitude in degrees of an S-JTSK point (Y, X positive).
pub fn sjtsk_to_wgs84(y: f64, x: f64, h: f64) -> Option<(f64, f64)> {
    let (phi, lam) = krovak_inverse(y, x)?;
    let (cx, cy, cz) = to_ecef(phi, lam, h, BESSEL_A, BESSEL_INV_F);
    // Three-parameter shift Bessel (S-JTSK) → WGS 84.
    let (wx, wy, wz) = (cx + 589.0, cy + 76.0, cz + 480.0);
    let (lat, lon) = from_ecef(wx, wy, wz, WGS_A, WGS_INV_F);
    let (lat, lon) = (lat.to_degrees(), lon.to_degrees());
    (lat.is_finite() && lon.is_finite()).then_some((lat, lon))
}

fn run_geokml(s: &mut Session, p: &Value) -> Result<Value> {
    const CMD: &str = "geokml";
    let space = s.space();
    let pts = numbered_points(s, &space, None)?;
    let chosen = super::targets(s, p)?;
    let paths: Vec<Vec<Vec3>> = {
        let d = s.doc()?;
        let st = d.space(&space).ok_or_else(|| bad(CMD, "no drawing space"))?;
        chosen
            .iter()
            .filter_map(|h| st.get(*h))
            .filter_map(|e| match &e.kind {
                EntityKind::Line(l) => Some(vec![l.a, l.b]),
                EntityKind::LwPolyline(pl) => {
                    let mut v: Vec<Vec3> = pl.vertices.iter().map(|q| Vec3::new(q.p.x, q.p.y, pl.elevation)).collect();
                    if pl.closed
                        && let Some(f) = v.first().copied()
                    {
                        v.push(f);
                    }
                    Some(v)
                }
                EntityKind::Polyline3d(pl) => Some(pl.points.clone()),
                _ => None,
            })
            .collect()
    };
    if pts.is_empty() && paths.is_empty() {
        return Err(bad(CMD, "nothing to export: no points on GEO_body and no lines selected"));
    }
    let plan: Vec<Vec2> = pts.iter().map(|(_, q)| q.xy()).chain(paths.iter().flatten().map(|q| q.xy())).collect();
    if !is_sjtsk(&plan) {
        return Err(bad(CMD, "KML needs S-JTSK coordinates (a local system can't be placed on the globe)"));
    }
    let ll = |q: Vec3| sjtsk_to_wgs84(-q.x, -q.y, q.z);
    let mut out = String::from(
        "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<kml xmlns=\"http://www.opengis.net/kml/2.2\">\n<Document>\n  <name>CADCraft</name>\n",
    );
    let mut n_pts = 0;
    for (num, q) in &pts {
        let Some((lat, lon)) = ll(*q) else { continue };
        n_pts += 1;
        out.push_str(&format!(
            "  <Placemark><name>{}</name><description>{:.2} m</description><Point><coordinates>{lon:.8},{lat:.8},0</coordinates></Point></Placemark>\n",
            esc(num),
            q.z
        ));
    }
    let mut n_paths = 0;
    for path in &paths {
        let coords: Vec<String> = path.iter().filter_map(|q| ll(*q)).map(|(lat, lon)| format!("{lon:.8},{lat:.8},0")).collect();
        if coords.len() >= 2 {
            n_paths += 1;
            out.push_str(&format!(
                "  <Placemark><LineString><tessellate>1</tessellate><coordinates>{}</coordinates></LineString></Placemark>\n",
                coords.join(" ")
            ));
        }
        if out.len() > MAX_TEXT {
            return Err(bad(CMD, "the export would be larger than 512 MB"));
        }
    }
    out.push_str("</Document>\n</kml>\n");
    let what = json!({ "points": n_pts, "paths": n_paths, "message": format!("KML: {n_pts} points, {n_paths} paths") });
    write_or_return(str_param(p, "path"), out, CMD, what)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn krovak_matches_the_epsg_example() {
        // EPSG guidance note 7-2, method 9819: Y 568 990.997, X 1 050 538.63 is
        // 50°12′32.442″ N, 16°50′59.179″ E on the Bessel ellipsoid.
        let (phi, lam) = krovak_inverse(568_990.997, 1_050_538.63).unwrap();
        assert!((phi.to_degrees() - (50.0 + 12.0 / 60.0 + 32.442 / 3600.0)).abs() < 2e-6, "{}", phi.to_degrees());
        assert!((lam.to_degrees() - (16.0 + 50.0 / 60.0 + 59.179 / 3600.0)).abs() < 2e-6, "{}", lam.to_degrees());
    }

    #[test]
    fn prague_lands_in_prague() {
        // About Prague's Old Town in S-JTSK.
        let (lat, lon) = sjtsk_to_wgs84(743_000.0, 1_043_000.0, 300.0).unwrap();
        assert!((lat - 50.08).abs() < 0.05 && (lon - 14.42).abs() < 0.08, "{lat} {lon}");
        assert!(sjtsk_to_wgs84(0.0, 0.0, 0.0).is_none());
    }

    #[test]
    fn landxml_and_kml_from_a_survey() {
        let mut s = Session::new();
        assert!(s.execute("geolandxml", &json!({})).is_err());
        s.execute("geobody", &json!({ "text": "1 743000 1043000 300\n2 743010 1043000 301\n3 743000 1043010 302\n" })).unwrap();
        s.execute("geodmt", &json!({})).unwrap();
        let r = s.execute("geolandxml", &json!({})).unwrap();
        let t = r["text"].as_str().unwrap();
        assert!(t.contains("<CgPoint name=\"2\">1043000.000 743010.000 301.000</CgPoint>"), "{t}");
        assert!(t.contains("<F>") && t.contains("epsgCode=\"5514\""), "{t}");
        assert_eq!(r["surfaces"], 1);
        let r = s.execute("geokml", &json!({})).unwrap();
        let k = r["text"].as_str().unwrap();
        assert_eq!(r["points"], 3, "{r}");
        assert!(k.contains("<name>3</name>") && k.contains(",50.0"), "{k}");
        let mut local = Session::new();
        local.execute("geobody", &json!({ "text": "1 10 20 3\n", "sjtsk": false })).unwrap();
        assert!(local.execute("geokml", &json!({})).is_err());
        assert!(local.execute("geolandxml", &json!({})).is_ok());
        assert_eq!(esc("a<&\"b"), "a&lt;&amp;&quot;b");
    }
}
