//! Spherical Mercator projection and the fit-to-rect viewport built on it.
//!
//! The projection is used in two roles:
//!
//! * geographic bounding box -> [`GeoBounds`]
//! * projection viewport -> [`MercatorView`]
//!
//! This replaces the hand-rolled equirectangular projection the overlay used to
//! carry.  Mercator is conformal, so a junction drawn at a right angle really
//! is one, and — unlike a single `cos(lat)` correction taken at the centre of
//! the track — it stays exact however far north the drive runs or however large
//! its bounding box grows.

use std::f64::consts::PI;

/// Zoom ceiling used for degenerate bounding boxes.
const MAX_ZOOM: f64 = 262_144.0;

/// Geographic bounding box, in degrees.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GeoBounds {
    pub north: f64,
    pub south: f64,
    pub east: f64,
    pub west: f64,
}

impl GeoBounds {
    /// Create a degenerate box around a single point.
    pub fn from_point(lat: f64, lon: f64) -> Self {
        GeoBounds {
            north: lat,
            south: lat,
            east: lon,
            west: lon,
        }
    }

    /// Extend the bounds to include a finite point.
    pub fn extend(&mut self, lat: f64, lon: f64) {
        if !(lat.is_finite() && lon.is_finite()) {
            return;
        }
        if self.south > lat {
            self.south = lat;
        } else if self.north < lat {
            self.north = lat;
        }
        if self.west > lon {
            self.west = lon;
        } else if self.east < lon {
            self.east = lon;
        }
    }

    /// Build bounds from the first point and extend them with the rest.
    ///
    /// Returns `None` when the iterator is empty. Non-finite points after the
    /// first are ignored by [`Self::extend`].
    pub fn from_points<I: IntoIterator<Item = (f64, f64)>>(points: I) -> Option<Self> {
        let mut it = points.into_iter();
        let (lat0, lon0) = it.next()?;
        let mut b = GeoBounds::from_point(lat0, lon0);
        for (lat, lon) in it {
            b.extend(lat, lon);
        }
        Some(b)
    }

    #[inline]
    /// Midpoint of the latitude span in degree space.
    ///
    /// This is the arithmetic centre of the box, not a Mercator-projected
    /// midpoint; the projection itself is handled by [`MercatorView::fit`].
    pub fn mid_lat(&self) -> f64 {
        self.south / 2.0 + self.north / 2.0
    }

    #[inline]
    /// Midpoint of the longitude span in degree space.
    pub fn mid_lon(&self) -> f64 {
        self.west / 2.0 + self.east / 2.0
    }

    #[inline]
    /// Whether all four stored edges are finite.
    pub fn is_valid(&self) -> bool {
        self.north.is_finite()
            && self.south.is_finite()
            && self.east.is_finite()
            && self.west.is_finite()
    }
}

/// A Mercator viewport: `scale` is pixels per radian and `origin_*` is the
/// top-left corner of the visible rect in projected pixel space.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MercatorView {
    /// Reference size used by the zoom clamp.
    base: f64,
    /// Pixels per radian.
    scale: f64,
    /// Projected point drawn at (0, 0).
    origin_x: f64,
    origin_y: f64,
}

impl MercatorView {
    /// Create a viewport whose default scale corresponds to a 1048576 px world.
    pub fn new(base: u32) -> Self {
        let base = base as f64;
        MercatorView {
            base,
            scale: (base / (2.0 * PI)) * 1_048_576.0,
            origin_x: 0.0,
            origin_y: 0.0,
        }
    }

    /// Convert latitude in radians to Mercator y.  Clamping `sin(phi)` to
    /// ±0.9999 keeps the poles finite.
    #[inline]
    fn mercator_y(phi_rad: f64, scale: f64) -> f64 {
        let s = phi_rad.sin().clamp(-0.9999, 0.9999);
        (PI - (((s + 1.0) / (1.0 - s)).ln() * 0.5)) * scale
    }

    /// Convert longitude in degrees to an x pixel.
    #[inline]
    pub fn project_x(&self, lon_deg: f64) -> f32 {
        (((lon_deg.to_radians() + PI) * self.scale) - self.origin_x) as f32
    }

    /// Convert latitude in degrees to a y pixel.
    #[inline]
    pub fn project_y(&self, lat_deg: f64) -> f32 {
        (Self::mercator_y(lat_deg.to_radians(), self.scale) - self.origin_y) as f32
    }

    /// Fit `bounds` into a `width` × `height` rectangle with `margin` pixels of
    /// padding on every side, then centre it.
    pub fn fit(&mut self, bounds: &GeoBounds, margin: f32, width: f32, height: f32) {
        let pad = margin * 2.0;

        // Vertical fit: the Mercator y span of the box is its latitude span
        // stretched by sec(mid latitude).
        let lat_span = (bounds.north - bounds.south).to_radians();
        let sec_mid = 1.0 / bounds.mid_lat().to_radians().cos();
        let scale_from_height = (height - pad) as f64 / (lat_span * sec_mid);
        let scale_from_width = (width - pad) as f64 / (bounds.east - bounds.west).to_radians();

        self.scale = scale_from_height
            .min(scale_from_width)
            .min((self.base / (2.0 * PI)) * MAX_ZOOM);

        self.origin_x = ((bounds.mid_lon().to_radians() + PI) * self.scale) - (width / 2.0) as f64;
        self.origin_y =
            Self::mercator_y(bounds.mid_lat().to_radians(), self.scale) - (height / 2.0) as f64;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn taipei_box() -> GeoBounds {
        GeoBounds::from_points([(25.0, 121.0), (25.01, 121.02), (25.005, 121.004)]).unwrap()
    }

    #[test]
    fn bounds_grow_to_cover_every_point() {
        let b = taipei_box();
        assert!((b.south - 25.0).abs() < 1e-12 && (b.north - 25.01).abs() < 1e-12);
        assert!((b.west - 121.0).abs() < 1e-12 && (b.east - 121.02).abs() < 1e-12);
        // A dropout contributes no position and must not poison the box.
        let mut b = b;
        b.extend(f64::NAN, f64::NAN);
        assert!(b.is_valid());
        assert!(GeoBounds::from_points(std::iter::empty()).is_none());
    }

    #[test]
    fn fit_centres_the_box_inside_the_margins() {
        let b = taipei_box();
        let mut v = MercatorView::new(480);
        v.fit(&b, 24.0, 480.0, 480.0);

        let (cx, cy) = (v.project_x(b.mid_lon()), v.project_y(b.mid_lat()));
        assert!((cx - 240.0).abs() < 0.01, "cx = {cx}");
        assert!((cy - 240.0).abs() < 0.01, "cy = {cy}");

        // The whole box lands inside the rect, touching one pair of margins.
        let (x0, x1) = (v.project_x(b.west), v.project_x(b.east));
        let (y0, y1) = (v.project_y(b.north), v.project_y(b.south));
        assert!(x0 >= 23.99 && x1 <= 456.01, "x {x0}..{x1}");
        assert!(y0 >= 23.99 && y1 <= 456.01, "y {y0}..{y1}");
        assert!((x0 - 24.0).abs() < 0.01 || (y0 - 24.0).abs() < 0.01);
        // North is up.
        assert!(y0 < y1 && x0 < x1);
    }

    #[test]
    fn a_parked_car_still_projects_to_a_finite_point() {
        let b = GeoBounds::from_point(25.0, 121.0);
        let mut v = MercatorView::new(480);
        v.fit(&b, 24.0, 480.0, 480.0);
        let (x, y) = (v.project_x(121.0), v.project_y(25.0));
        assert!(x.is_finite() && y.is_finite(), "({x}, {y})");
        assert!((x - 240.0).abs() < 0.01 && (y - 240.0).abs() < 0.01);
    }

    #[test]
    fn the_projection_keeps_its_aspect_ratio() {
        // One degree of latitude is 111 km; at 25°N one degree of longitude is
        // 111 × cos(25°) = 101 km. The drawn box must be that much wider.
        let b = GeoBounds::from_points([(25.0, 121.0), (25.1, 121.1)]).unwrap();
        let mut v = MercatorView::new(480);
        v.fit(&b, 0.0, 480.0, 480.0);
        let w = v.project_x(b.east) - v.project_x(b.west);
        let h = v.project_y(b.south) - v.project_y(b.north);
        let ratio = (w / h) as f64;
        assert!(
            (ratio - 25.05f64.to_radians().cos()).abs() < 1e-3,
            "{ratio}"
        );
    }
}
