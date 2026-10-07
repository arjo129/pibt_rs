use std::{num::NonZeroU16, ops::DerefMut};

use ::rand::{SeedableRng, rngs::StdRng, seq::SliceRandom};
use macroquad::{miniquad::gl::WGL_CONTEXT_FORWARD_COMPATIBLE_BIT_ARB, prelude::*};
use serde::{Deserialize, Serialize};

/// Geometry utility: computes the centroid of a 2D polygon from its vertices.
/// Uses the standard polygon signed area formula; falls back to the vertex average
/// for degenerate or collinear polygons.
pub fn compute_polygon_centroid(vertices: &[(f32, f32)]) -> (f32, f32) {
    let n = vertices.len();
    if n == 0 {
        return (0.0, 0.0);
    }
    if n == 1 {
        return vertices[0];
    }
    if n == 2 {
        return (
            (vertices[0].0 + vertices[1].0) * 0.5,
            (vertices[0].1 + vertices[1].1) * 0.5,
        );
    }

    let mut signed_area = 0.0f32;
    let mut cx = 0.0f32;
    let mut cy = 0.0f32;

    for i in 0..n {
        let (x0, y0) = vertices[i];
        let (x1, y1) = vertices[(i + 1) % n];
        let cross = x0 * y1 - x1 * y0;
        signed_area += cross;
        cx += (x0 + x1) * cross;
        cy += (y0 + y1) * cross;
    }

    signed_area *= 0.5;
    if signed_area.abs() > 1e-5 {
        let factor = 1.0 / (6.0 * signed_area);
        (cx * factor, cy * factor)
    } else {
        let sum_x: f32 = vertices.iter().map(|v| v.0).sum();
        let sum_y: f32 = vertices.iter().map(|v| v.1).sum();
        (sum_x / n as f32, sum_y / n as f32)
    }
}

/// Squared Euclidean distance from a point `p` to a line segment `ab`.
pub fn point_to_segment_distance_sq(p: (f32, f32), a: (f32, f32), b: (f32, f32)) -> f32 {
    let dx = b.0 - a.0;
    let dy = b.1 - a.1;
    let len_sq = dx * dx + dy * dy;
    if len_sq < 1e-8 {
        let px = p.0 - a.0;
        let py = p.1 - a.1;
        return px * px + py * py;
    }
    let t = (((p.0 - a.0) * dx + (p.1 - a.1) * dy) / len_sq).clamp(0.0, 1.0);
    let proj_x = a.0 + t * dx;
    let proj_y = a.1 + t * dy;
    let rx = p.0 - proj_x;
    let ry = p.1 - proj_y;
    rx * rx + ry * ry
}

/// Liang-Barsky segment clipping test against an Axis-Aligned Bounding Box (AABB).
/// Returns true if the line segment `p1` to `p2` intersects the AABB [min_x, max_x] x [min_y, max_y].
pub fn segment_intersects_aabb(
    p1: (f32, f32),
    p2: (f32, f32),
    min_x: f32,
    min_y: f32,
    max_x: f32,
    max_y: f32,
) -> bool {
    let dx = p2.0 - p1.0;
    let dy = p2.1 - p1.1;

    let mut t0 = 0.0f32;
    let mut t1 = 1.0f32;

    let p = [-dx, dx, -dy, dy];
    let q = [p1.0 - min_x, max_x - p1.0, p1.1 - min_y, max_y - p1.1];

    for i in 0..4 {
        if p[i].abs() < 1e-8 {
            if q[i] < 0.0 {
                return false;
            }
        } else {
            let t = q[i] / p[i];
            if p[i] < 0.0 {
                if t > t1 {
                    return false;
                }
                if t > t0 {
                    t0 = t;
                }
            } else {
                if t < t0 {
                    return false;
                }
                if t < t1 {
                    t1 = t;
                }
            }
        }
    }
    t0 <= t1
}

/// Robust intersection test between an AABB grid cell and a 2D polygon.
/// Returns true if the cell touches or overlaps the polygon.
pub fn cell_intersects_polygon(
    min_x: f32,
    min_y: f32,
    max_x: f32,
    max_y: f32,
    polygon: &Polygon,
) -> bool {
    let boundary = polygon.boundary_vertices();
    if boundary.is_empty() {
        return false;
    }

    // 1. Any polygon vertex inside the cell?
    for &(vx, vy) in &boundary {
        if vx >= min_x && vx <= max_x && vy >= min_y && vy <= max_y {
            return true;
        }
    }

    // 2. Cell center inside polygon?
    let center = ((min_x + max_x) * 0.5, (min_y + max_y) * 0.5);
    if polygon.contains_point(center) {
        return true;
    }

    // 3. Any polygon edge intersects the cell AABB?
    let n = boundary.len();
    for i in 0..n {
        let p1 = boundary[i];
        let p2 = boundary[(i + 1) % n];
        if segment_intersects_aabb(p1, p2, min_x, min_y, max_x, max_y) {
            return true;
        }
    }

    false
}

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
pub struct Polygon {
    pub vertices: Vec<(f32, f32)>,
    pub draw_index: Vec<usize>,
    pub centroid: (f32, f32),
}

impl Polygon {
    pub fn new(vertices: Vec<(f32, f32)>, draw_index: Vec<usize>, centroid: (f32, f32)) -> Self {
        Self {
            vertices,
            draw_index,
            centroid,
        }
    }

    /// Construct a polygon from vertices given in perimeter order.
    /// Centroid and closed draw loop are calculated automatically.
    pub fn from_vertices(vertices: Vec<(f32, f32)>) -> Self {
        let n = vertices.len();
        assert!(n >= 3, "A polygon must have at least 3 vertices");
        let mut draw_index: Vec<usize> = (0..n).collect();
        draw_index.push(0);
        let centroid = compute_polygon_centroid(&vertices);
        Self {
            vertices,
            draw_index,
            centroid,
        }
    }

    /// Construct an axis-aligned rectangle polygon.
    pub fn rectangle(min_x: f32, min_y: f32, width: f32, height: f32) -> Self {
        let centroid = (min_x + width * 0.5, min_y + height * 0.5);
        Self {
            centroid,
            vertices: vec![
                (min_x, min_y),
                (min_x, min_y + height),
                (min_x + width, min_y + height),
                (min_x + width, min_y),
            ],
            draw_index: vec![0, 1, 2, 3, 0],
        }
    }

    /// Returns the ordered perimeter vertices of the polygon.
    pub fn boundary_vertices(&self) -> Vec<(f32, f32)> {
        if self.draw_index.is_empty() {
            return self.vertices.clone();
        }
        let mut indices = self.draw_index.as_slice();
        if indices.len() > 1 && indices.first() == indices.last() {
            indices = &indices[..indices.len() - 1];
        }
        indices
            .iter()
            .filter_map(|&i| self.vertices.get(i).copied())
            .collect()
    }

    /// Returns the axis-aligned bounding box ((min_x, min_y), (max_x, max_y)).
    pub fn aabb(&self) -> ((f32, f32), (f32, f32)) {
        let boundary = self.boundary_vertices();
        if boundary.is_empty() {
            return (self.centroid, self.centroid);
        }
        let mut min_x = f32::MAX;
        let mut min_y = f32::MAX;
        let mut max_x = f32::MIN;
        let mut max_y = f32::MIN;
        for &(x, y) in &boundary {
            if x < min_x {
                min_x = x;
            }
            if y < min_y {
                min_y = y;
            }
            if x > max_x {
                max_x = x;
            }
            if y > max_y {
                max_y = y;
            }
        }
        ((min_x, min_y), (max_x, max_y))
    }

    /// Point-in-polygon test using ray casting (even-odd rule).
    pub fn contains_point(&self, p: (f32, f32)) -> bool {
        let ((min_x, min_y), (max_x, max_y)) = self.aabb();
        if p.0 < min_x - 1e-4 || p.0 > max_x + 1e-4 || p.1 < min_y - 1e-4 || p.1 > max_y + 1e-4 {
            return false;
        }

        let b = self.boundary_vertices();
        if b.len() < 3 {
            return false;
        }
        let mut inside = false;
        let n = b.len();
        let mut j = n - 1;
        for i in 0..n {
            let (xi, yi) = b[i];
            let (xj, yj) = b[j];
            let intersect =
                ((yi > p.1) != (yj > p.1)) && (p.0 < (xj - xi) * (p.1 - yi) / (yj - yi) + xi);
            if intersect {
                inside = !inside;
            }
            j = i;
        }
        inside
    }

    /// Distance from point `p` to the polygon perimeter.
    pub fn distance_to_boundary(&self, p: (f32, f32)) -> f32 {
        let b = self.boundary_vertices();
        if b.is_empty() {
            return f32::MAX;
        }
        if b.len() == 1 {
            let dx = p.0 - b[0].0;
            let dy = p.1 - b[0].1;
            return (dx * dx + dy * dy).sqrt();
        }
        let mut min_d_sq = f32::MAX;
        let n = b.len();
        for i in 0..n {
            let j = (i + 1) % n;
            let d_sq = point_to_segment_distance_sq(p, b[i], b[j]);
            if d_sq < min_d_sq {
                min_d_sq = d_sq;
            }
        }
        min_d_sq.sqrt()
    }

    pub fn draw(&self) {
        self.draw_styled(RED, 1.0);
    }

    pub fn draw_styled(&self, color: Color, thickness: f32) {
        for i in 0..self.draw_index.len().saturating_sub(1) {
            let start = self.vertices[self.draw_index[i]];
            let end = self.vertices[self.draw_index[i + 1]];
            draw_line(start.0, start.1, end.0, end.1, thickness, color);
        }
    }

    pub fn draw_filled(&self, color: Color) {
        let b = self.boundary_vertices();
        if b.len() < 3 {
            return;
        }
        for i in 1..b.len() - 1 {
            draw_triangle(
                Vec2::new(b[0].0, b[0].1),
                Vec2::new(b[i].0, b[i].1),
                Vec2::new(b[i + 1].0, b[i + 1].1),
                color,
            );
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum GuidePreference {
    FORWARD,
    BACKWARD,
    NONE,
}

impl GuidePreference {
    pub fn opposite(&self) -> Self {
        match self {
            Self::FORWARD => Self::BACKWARD,
            Self::BACKWARD => Self::FORWARD,
            Self::NONE => Self::NONE,
        }
    }
}

#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct NavGraph {
    pub polygons: Vec<Polygon>,
    pub connections: Vec<Vec<(usize, GuidePreference)>>,
}

impl NavGraph {
    pub fn add_node(&mut self, polygon: Polygon) -> usize {
        self.polygons.push(polygon);
        self.connections.push(vec![]);
        self.polygons.len() - 1
    }

    pub fn add_rectangle(&mut self, min_x: f32, min_y: f32, width: f32, height: f32) -> usize {
        self.add_node(Polygon::rectangle(min_x, min_y, width, height))
    }

    pub fn add_polygon_from_vertices(&mut self, vertices: Vec<(f32, f32)>) -> usize {
        self.add_node(Polygon::from_vertices(vertices))
    }

    /// Note: We DO NOT check for duplicates
    pub fn add_edge(&mut self, n1: usize, n2: usize, guide: GuidePreference) {
        self.connections[n1].push((n2, guide));
        self.connections[n2].push((n1, guide.opposite()));
    }

    /// Going back: maps a specific polygon index to its centroid (x, y) location.
    pub fn polygon_centroid(&self, polygon_id: usize) -> Option<(f32, f32)> {
        self.polygons.get(polygon_id).map(|p| p.centroid)
    }

    /// Alias for going back from polygon to (x, y) location.
    pub fn polygon_to_point(&self, polygon_id: usize) -> Option<(f32, f32)> {
        self.polygon_centroid(polygon_id)
    }

    /// Returns the combined bounding box of all polygons in the NavGraph.
    pub fn bounds(&self) -> Option<((f32, f32), (f32, f32))> {
        if self.polygons.is_empty() {
            return None;
        }
        let mut min_x = f32::MAX;
        let mut min_y = f32::MAX;
        let mut max_x = f32::MIN;
        let mut max_y = f32::MIN;

        for p in &self.polygons {
            let ((p_min_x, p_min_y), (p_max_x, p_max_y)) = p.aabb();
            if p_min_x < min_x {
                min_x = p_min_x;
            }
            if p_min_y < min_y {
                min_y = p_min_y;
            }
            if p_max_x > max_x {
                max_x = p_max_x;
            }
            if p_max_y > max_y {
                max_y = p_max_y;
            }
        }

        Some(((min_x, min_y), (max_x, max_y)))
    }

    /// Builds a fixed-resolution spatial bitmap grid rasterizing all polygons in this NavGraph.
    pub fn build_bitmap_grid(&self, resolution: f32) -> PolygonBitmapGrid {
        PolygonBitmapGrid::from_nav_graph(self, resolution)
    }

    pub fn draw(&self) {
        self.draw_styled(RED, 1.0, false);
    }

    pub fn draw_styled(&self, wire_color: Color, thickness: f32, draw_connections: bool) {
        if draw_connections {
            for (u, neighbors) in self.connections.iter().enumerate() {
                let cu = self.polygons[u].centroid;
                for &(v, _) in neighbors {
                    if u < v {
                        let cv = self.polygons[v].centroid;
                        draw_line(cu.0, cu.1, cv.0, cv.1, 1.5, DARKGRAY);
                    }
                }
            }
        }
        for polygon in &self.polygons {
            polygon.draw_styled(wire_color, thickness);
        }
    }
}

/// A fixed-resolution bitmap spatial grid that rasterizes polygons from a NavGraph.
///
/// Provides O(1) lookup to map arbitrary (x, y) continuous locations on the NavGraph
/// to specific polygon IDs, and maps polygon IDs back to (x, y) centroids.
#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
pub struct PolygonBitmapGrid {
    /// World coordinate origin corresponding to grid cell (0, 0).
    pub origin: (f32, f32),
    /// World-space width and height of each discrete grid cell (fixed resolution).
    pub resolution: f32,
    /// Number of grid columns along the X axis.
    pub width: usize,
    /// Number of grid rows along the Y axis.
    pub height: usize,
    /// Flat 2D array of cells (index = `gy * width + gx`).
    /// Each cell stores the IDs of all polygons that intersect/overlap that cell.
    pub cells: Vec<Vec<usize>>,
}

impl PolygonBitmapGrid {
    /// Creates an empty bitmap grid with the given origin, resolution, and dimensions.
    pub fn new(origin: (f32, f32), width: usize, height: usize, resolution: f32) -> Self {
        assert!(resolution > 0.0, "Resolution must be positive");
        let total_cells = width * height;
        Self {
            origin,
            resolution,
            width,
            height,
            cells: vec![Vec::new(); total_cells],
        }
    }

    /// Automatically constructs and rasterizes a bitmap grid sized to cover the entire `NavGraph`.
    pub fn from_nav_graph(nav: &NavGraph, resolution: f32) -> Self {
        Self::from_nav_graph_with_padding(nav, resolution, 0.0)
    }

    /// Constructs and rasterizes a bitmap grid covering the `NavGraph` with optional world padding.
    pub fn from_nav_graph_with_padding(nav: &NavGraph, resolution: f32, padding: f32) -> Self {
        assert!(resolution > 0.0, "Resolution must be positive");
        let Some(((min_x, min_y), (max_x, max_y))) = nav.bounds() else {
            return Self::new((0.0, 0.0), 0, 0, resolution);
        };

        let origin_x = min_x - padding;
        let origin_y = min_y - padding;
        let total_w = (max_x + padding) - origin_x;
        let total_h = (max_y + padding) - origin_y;

        // +1 margin ensures points lying exactly on the upper boundary fall within valid indices
        let width = ((total_w / resolution).ceil() as usize + 1).max(1);
        let height = ((total_h / resolution).ceil() as usize + 1).max(1);

        let mut grid = Self::new((origin_x, origin_y), width, height, resolution);
        grid.rasterize_nav_graph(nav);
        grid
    }

    /// Clears all rasterized polygon references from all grid cells.
    pub fn clear(&mut self) {
        for cell in &mut self.cells {
            cell.clear();
        }
    }

    /// Rasterizes all polygons of a NavGraph into this bitmap grid.
    pub fn rasterize_nav_graph(&mut self, nav: &NavGraph) {
        for (poly_id, polygon) in nav.polygons.iter().enumerate() {
            self.rasterize_polygon(poly_id, polygon);
        }
    }

    /// Rasterizes a single polygon into the bitmap grid cells it overlaps.
    pub fn rasterize_polygon(&mut self, poly_id: usize, polygon: &Polygon) {
        if self.width == 0 || self.height == 0 {
            return;
        }

        let ((min_x, min_y), (max_x, max_y)) = polygon.aabb();

        // Convert world-space AABB to grid coordinate bounds, clamped to grid dimensions
        let min_gx = (((min_x - self.origin.0) / self.resolution).floor().max(0.0) as usize)
            .min(self.width.saturating_sub(1));
        let max_gx = (((max_x - self.origin.0) / self.resolution).floor().max(0.0) as usize)
            .min(self.width.saturating_sub(1));
        let min_gy = (((min_y - self.origin.1) / self.resolution).floor().max(0.0) as usize)
            .min(self.height.saturating_sub(1));
        let max_gy = (((max_y - self.origin.1) / self.resolution).floor().max(0.0) as usize)
            .min(self.height.saturating_sub(1));

        for gy in min_gy..=max_gy {
            let cell_min_y = self.origin.1 + gy as f32 * self.resolution;
            let cell_max_y = cell_min_y + self.resolution;
            for gx in min_gx..=max_gx {
                let cell_min_x = self.origin.0 + gx as f32 * self.resolution;
                let cell_max_x = cell_min_x + self.resolution;

                if cell_intersects_polygon(cell_min_x, cell_min_y, cell_max_x, cell_max_y, polygon)
                {
                    let idx = gy * self.width + gx;
                    if !self.cells[idx].contains(&poly_id) {
                        self.cells[idx].push(poly_id);
                    }
                }
            }
        }
    }

    /// Converts world coordinates (x, y) to discrete grid cell indices (gx, gy).
    pub fn world_to_grid(&self, x: f32, y: f32) -> Option<(usize, usize)> {
        if self.width == 0 || self.height == 0 {
            return None;
        }
        let gx_f = (x - self.origin.0) / self.resolution;
        let gy_f = (y - self.origin.1) / self.resolution;
        if gx_f < 0.0 || gy_f < 0.0 {
            return None;
        }
        let gx = gx_f.floor() as usize;
        let gy = gy_f.floor() as usize;
        if gx < self.width && gy < self.height {
            Some((gx, gy))
        } else {
            None
        }
    }

    /// Returns the world-space center coordinate of grid cell (gx, gy).
    pub fn grid_to_world_center(&self, gx: usize, gy: usize) -> (f32, f32) {
        (
            self.origin.0 + (gx as f32 + 0.5) * self.resolution,
            self.origin.1 + (gy as f32 + 0.5) * self.resolution,
        )
    }

    /// Returns the bounding box ((min_x, min_y), (max_x, max_y)) of cell (gx, gy).
    pub fn grid_to_cell_bounds(&self, gx: usize, gy: usize) -> ((f32, f32), (f32, f32)) {
        let min_x = self.origin.0 + gx as f32 * self.resolution;
        let min_y = self.origin.1 + gy as f32 * self.resolution;
        (
            (min_x, min_y),
            (min_x + self.resolution, min_y + self.resolution),
        )
    }

    /// Returns the slice of rasterized polygon IDs at grid cell (gx, gy).
    pub fn get_cell(&self, gx: usize, gy: usize) -> &[usize] {
        if gx < self.width && gy < self.height {
            &self.cells[gy * self.width + gx]
        } else {
            &[]
        }
    }

    /// Returns the slice of rasterized polygon candidate IDs at arbitrary world coordinates (x, y).
    pub fn get_raster_polygons_at(&self, x: f32, y: f32) -> &[usize] {
        match self.world_to_grid(x, y) {
            Some((gx, gy)) => self.get_cell(gx, gy),
            None => &[],
        }
    }

    /// Core forward mapping: goes from an arbitrary (x, y) location on the NavGraph
    /// to the specific polygon ID containing that location.
    ///
    /// Runs in O(1) time using the raster bitmap cell to retrieve candidate polygons,
    /// then performs an exact point-in-polygon test among candidates.
    pub fn get_polygon_at(&self, x: f32, y: f32, nav: &NavGraph) -> Option<usize> {
        let (gx, gy) = self.world_to_grid(x, y)?;
        let idx = gy * self.width + gx;
        let candidates = &self.cells[idx];
        if candidates.is_empty() {
            return None;
        }

        // Fast path: single candidate covering this cell
        if candidates.len() == 1 {
            let poly_id = candidates[0];
            if let Some(poly) = nav.polygons.get(poly_id) {
                if poly.contains_point((x, y)) || poly.distance_to_boundary((x, y)) < 1e-3 {
                    return Some(poly_id);
                }
            }
            return None;
        }

        // Multiple candidates (boundary cell): check strict containment
        for &poly_id in candidates {
            if let Some(poly) = nav.polygons.get(poly_id) {
                if poly.contains_point((x, y)) {
                    return Some(poly_id);
                }
            }
        }

        // Floating-point edge boundary fallback: select the candidate whose perimeter
        // is closest to the query point within tolerance.
        let mut best_candidate = None;
        let mut best_dist = f32::MAX;
        for &poly_id in candidates {
            if let Some(poly) = nav.polygons.get(poly_id) {
                let dist = poly.distance_to_boundary((x, y));
                if dist < best_dist {
                    best_dist = dist;
                    best_candidate = Some(poly_id);
                }
            }
        }

        if best_dist < 1e-2 {
            best_candidate
        } else {
            None
        }
    }

    /// Core reverse mapping: goes from a specific polygon ID back to its (x, y) centroid.
    pub fn polygon_to_point(&self, nav: &NavGraph, polygon_id: usize) -> Option<(f32, f32)> {
        nav.polygon_centroid(polygon_id)
    }

    /// Draws wireframe grid lines of the bitmap raster cells.
    pub fn draw_debug_grid(&self, color: Color) {
        if self.width == 0 || self.height == 0 {
            return;
        }
        let max_x = self.origin.0 + self.width as f32 * self.resolution;
        let max_y = self.origin.1 + self.height as f32 * self.resolution;

        for gx in 0..=self.width {
            let x = self.origin.0 + gx as f32 * self.resolution;
            draw_line(x, self.origin.1, x, max_y, 0.5, color);
        }
        for gy in 0..=self.height {
            let y = self.origin.1 + gy as f32 * self.resolution;
            draw_line(self.origin.0, y, max_x, y, 0.5, color);
        }
    }

    /// Fills a specific raster cell with color.
    pub fn draw_cell(&self, gx: usize, gy: usize, fill_color: Color) {
        if gx < self.width && gy < self.height {
            let x = self.origin.0 + gx as f32 * self.resolution;
            let y = self.origin.1 + gy as f32 * self.resolution;
            draw_rectangle(x, y, self.resolution, self.resolution, fill_color);
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DistanceMatrix {
    pub matrix: Vec<Vec<i32>>,
}

impl DistanceMatrix {
    pub fn from(nav_graph: &NavGraph) -> Self {
        let n = nav_graph.polygons.len();
        let mut matrix = vec![vec![-1; n]; n];
        let mut q = Vec::with_capacity(n);

        for i in 0..n {
            let row = &mut matrix[i];
            row[i] = 0;
            q.clear();
            q.push(i);
            let mut head = 0;

            while head < q.len() {
                let node = q[head];
                head += 1;
                let dist = row[node];

                for &(other, _) in &nav_graph.connections[node] {
                    if row[other] == -1 {
                        row[other] = dist + 1;
                        q.push(other);
                    }
                }
            }
        }

        Self { matrix }
    }
}

impl From<&NavGraph> for DistanceMatrix {
    fn from(nav_graph: &NavGraph) -> Self {
        Self::from(nav_graph)
    }
}

pub fn simple_square(centroid: (f32, f32), size: f32) -> Polygon {
    Polygon {
        centroid,
        vertices: vec![
            (centroid.0 - size / 2.0, centroid.1 - size / 2.0),
            (centroid.0 - size / 2.0, centroid.1 + size / 2.0),
            (centroid.0 + size / 2.0, centroid.1 - size / 2.0),
            (centroid.0 + size / 2.0, centroid.1 + size / 2.0),
        ],
        draw_index: vec![0, 1, 3, 2, 0],
    }
}

pub fn generate_grid(start: (f32, f32), grid_size: f32, width: usize, height: usize) -> NavGraph {
    let mut nav = NavGraph::default();
    if width == 0 || height == 0 {
        return nav;
    }
    let (start_x, start_y) = start;

    for i in 0..width {
        for j in 0..height {
            let centroid = (
                start_x + (i as f32 + 0.5) * grid_size,
                start_y + (j as f32 + 0.5) * grid_size,
            );
            nav.add_node(simple_square(centroid, grid_size));
        }
    }

    let node_id = |i: usize, j: usize| i * height + j;

    for i in 0..width {
        for j in 0..height {
            let current = node_id(i, j);
            if i + 1 < width {
                nav.add_edge(current, node_id(i + 1, j), GuidePreference::NONE);
            }
            if j + 1 < height {
                nav.add_edge(current, node_id(i, j + 1), GuidePreference::NONE);
            }
        }
    }
    nav
}

/// Generates a realistic non-uniform nav-mesh graph with variable-sized rooms,
/// narrow corridors, and an angled triangular foyer.
pub fn generate_non_uniform_navmesh() -> NavGraph {
    let mut nav = NavGraph::default();

    // Node 0: Large Central Hall (120 x 80)
    let n0 = nav.add_rectangle(180.0, 140.0, 140.0, 90.0);

    // Node 1: West Room (70 x 70)
    let n1 = nav.add_rectangle(60.0, 150.0, 70.0, 70.0);

    // Node 2: West Connecting Corridor (50 x 30)
    let n2 = nav.add_rectangle(130.0, 170.0, 50.0, 30.0);

    // Node 3: North Office (90 x 50)
    let n3 = nav.add_rectangle(205.0, 50.0, 90.0, 50.0);

    // Node 4: North Corridor (30 x 40)
    let n4 = nav.add_rectangle(235.0, 100.0, 30.0, 40.0);

    // Node 5: East Corridor (60 x 30)
    let n5 = nav.add_rectangle(320.0, 170.0, 60.0, 30.0);

    // Node 6: East Wing (80 x 110)
    let n6 = nav.add_rectangle(380.0, 130.0, 80.0, 110.0);

    // Node 7: South Triangular Atrium / Foyer
    let n7 = nav.add_polygon_from_vertices(vec![(210.0, 230.0), (290.0, 230.0), (250.0, 310.0)]);

    // Connect the non-uniform navmesh nodes
    nav.add_edge(n1, n2, GuidePreference::NONE);
    nav.add_edge(n2, n0, GuidePreference::NONE);
    nav.add_edge(n3, n4, GuidePreference::NONE);
    nav.add_edge(n4, n0, GuidePreference::NONE);
    nav.add_edge(n0, n5, GuidePreference::NONE);
    nav.add_edge(n5, n6, GuidePreference::NONE);
    nav.add_edge(n0, n7, GuidePreference::NONE);

    nav
}

struct PIBTOverNavGraph {
    nav_graph: NavGraph,
    q: Vec<Vec<Option<usize>>>,
    // TODO(arjoc): Only compute distance matrix for agents
    distance_matrix: DistanceMatrix,
    rng: StdRng,
    // TODO(arjoc): Remove this
    ends: Vec<usize>,
    occupied_now: Vec<Option<usize>>,
    occupied_nxt: Vec<Option<usize>>,
}

impl PIBTOverNavGraph {
    pub fn init(nav_graph: NavGraph) -> Self {
        let distance_matrix = DistanceMatrix::from(&nav_graph);
        let nav_graph_len = nav_graph.polygons.len();
        Self {
            nav_graph,
            distance_matrix,
            q: vec![],
            rng: StdRng::seed_from_u64(42),
            ends: vec![], //hacky remove this once we fix the Distance Matrix API
            occupied_now: vec![None; nav_graph_len],
            occupied_nxt: vec![None; nav_graph_len],
        }
    }

    fn pibt(&mut self, agent: usize, time: usize) -> bool {
        let Some(q_from) = self.q[time][agent] else {
            // SAFETY: pibt is a private API that should only be called from within this file
            // Before we call it we already populate all positions for the agent at timestap t.
            panic!("Accessed an agent wuth no position");
        };
        let mut neighbors = self.nav_graph.connections[q_from].clone();
        neighbors.shuffle(&mut self.rng);

        let goal_node = self.ends[agent];

        neighbors.sort_by(|pos1, pos2| {
            // Get agent's goal
            let dist1 = self.distance_matrix.matrix[pos1.0][goal_node];
            let dist2 = self.distance_matrix.matrix[pos2.0][goal_node];
            dist1.cmp(&dist2)
        });

        for (node_id, _) in neighbors {
            // vertex conflict
            if self.occupied_nxt[node_id] != None {
                continue;
            }

            let agent_to_move_out = self.occupied_now[node_id];

            //swap conflicts
            if let Some(agent_to_move_out) = agent_to_move_out {
                if self.q[time][agent] == self.q[time + 1][agent_to_move_out] {
                    continue;
                }
            }

            // Reserve next location
            self.occupied_nxt[node_id] = Some(agent);
            self.q[time + 1][agent] = Some(node_id);

            if let Some(agent_to_move_out) = agent_to_move_out {
                if self.q[time + 1][agent_to_move_out] == None {
                    if !self.pibt(agent_to_move_out, time) {
                        continue;
                    }
                }
            }

            return true;
        }

        self.occupied_nxt[self.q[time][agent].unwrap()] = Some(agent);
        self.q[time + 1][agent] = self.q[time][agent];
        false
    }

    fn is_solution(&self, t: usize) -> bool {
        if self.q[t].len() != self.ends.len() {
            return false;
        }

        for item in 0..self.q[t].len() {
            let Some(val) = self.q[t][item] else {
                return false;
            };

            if val != self.ends[item] {
                return false;
            }
        }

        true
    }

    pub fn solve(
        &mut self,
        starts: Vec<(f32, f32)>,
        ends: Vec<(f32, f32)>,
        max_time: usize,
    ) -> Vec<Vec<(f32, f32)>> {
        let mut agents: Vec<_> = (0..starts.len()).collect();
        let mut final_trajectory = vec![];

        let bitmap = self.nav_graph.build_bitmap_grid(1.0);

        let initial_position: Vec<_> = starts
            .iter()
            .map(|(x, y)| bitmap.get_polygon_at(*x, *y, &self.nav_graph))
            .collect();
        let final_position: Vec<_> = ends
            .iter()
            .map(|(x, y)| bitmap.get_polygon_at(*x, *y, &self.nav_graph))
            .collect();

        // TODO(arjoc): return an error
        self.ends = final_position.iter().map(|p| p.unwrap()).collect();

        for (agent, node) in initial_position.iter().enumerate() {
            let Some(node) = node else {
                continue;
            };
            self.occupied_now[*node] = Some(agent);
        }

        self.q.push(initial_position.clone());
        self.q
            .extend((1..max_time).map(|_| vec![None; agents.len()]));

        let mut priorities: Vec<_> = (0..starts.len())
            .map(|agent| {
                self.distance_matrix.matrix[initial_position[agent].unwrap()]
                    [final_position[agent].unwrap()]
            })
            .collect();
        for t in 1..max_time - 1 {
            agents.sort_by(|p, q| priorities[*p].cmp(&priorities[*q]));
            for agent in &agents {
                if self.q[t][*agent] != None {
                    continue;
                }

                self.pibt(*agent, t - 1);
            }

            self.occupied_now = self.occupied_nxt.clone();
            self.occupied_nxt = vec![None; self.occupied_nxt.len()];

            if self.is_solution(t) {
                for i in 0..=t {
                    final_trajectory.push(
                        self.q[i]
                            .iter()
                            .map(|p| {
                                if let Some(p) = p {
                                    self.nav_graph.polygon_centroid(*p).unwrap()
                                } else {
                                    (-1.0, -1.0)
                                }
                            })
                            .collect(),
                    )
                }
                break;
            }
        }

        final_trajectory
    }
}

#[macroquad::main("gpibt nav-mesh demo")]
async fn main() {
    let nav = generate_grid((100.0, 100.0), 60.0, 5, 5);

    // Spawn 4 robots at the 4 corners of the 5x5 grid targeting the opposite corners
    let starts = vec![
        nav.polygons[0].centroid,  // (0, 0)
        nav.polygons[4].centroid,  // (0, 4)
        nav.polygons[20].centroid, // (4, 0)
        nav.polygons[24].centroid, // (4, 4)
    ];
    let ends = vec![
        nav.polygons[24].centroid, // (4, 4)
        nav.polygons[20].centroid, // (4, 0)
        nav.polygons[4].centroid,  // (0, 4)
        nav.polygons[0].centroid,  // (0, 0)
    ];

    let mut solver = PIBTOverNavGraph::init(nav.clone());
    let trajectories = solver.solve(starts, ends.clone(), 50);

    let agent_colors = [
        Color::new(0.95, 0.30, 0.30, 1.0), // Red
        Color::new(0.30, 0.85, 0.45, 1.0), // Green
        Color::new(0.30, 0.60, 0.95, 1.0), // Blue
        Color::new(0.95, 0.80, 0.25, 1.0), // Yellow
    ];

    let mut last_update = std::time::Instant::now();
    let mut time = 0usize;

    loop {
        clear_background(Color::new(0.08, 0.08, 0.1, 1.0));

        // Draw 5x5 navigation grid and connections
        nav.draw_styled(Color::new(0.7, 0.7, 0.8, 1.0), 2.0, true);

        // Draw goal markers for each agent
        for (agent, &(gx, gy)) in ends.iter().enumerate() {
            draw_circle_lines(gx, gy, 18.0, 2.0, agent_colors[agent]);
        }

        // Draw agents at current timestep
        if !trajectories.is_empty() {
            for (agent, &(x, y)) in trajectories[time].iter().enumerate() {
                draw_circle(x, y, 14.0, agent_colors[agent]);
            }

            if last_update.elapsed().as_secs_f32() >= 0.5 {
                time = (time + 1) % trajectories.len();
                last_update = std::time::Instant::now();
            }
        }

        next_frame().await;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_generate_grid_empty() {
        let nav0 = generate_grid((0.0, 0.0), 20.0, 0, 0);
        assert_eq!(nav0.polygons.len(), 0);
        let nav1 = generate_grid((0.0, 0.0), 20.0, 5, 0);
        assert_eq!(nav1.polygons.len(), 0);
        let nav2 = generate_grid((0.0, 0.0), 20.0, 0, 5);
        assert_eq!(nav2.polygons.len(), 0);
    }

    #[test]
    fn test_generate_grid_structure() {
        let width = 3;
        let height = 4;
        let grid_size = 10.0;
        let nav = generate_grid((0.0, 0.0), grid_size, width, height);
        assert_eq!(nav.polygons.len(), width * height);

        for i in 0..width {
            for j in 0..height {
                let id = i * height + j;
                let expected_centroid =
                    ((i as f32 + 0.5) * grid_size, (j as f32 + 0.5) * grid_size);
                assert!((nav.polygons[id].centroid.0 - expected_centroid.0).abs() < 1e-5);
                assert!((nav.polygons[id].centroid.1 - expected_centroid.1).abs() < 1e-5);

                let mut expected_neighbors = 0;
                if i > 0 {
                    expected_neighbors += 1;
                }
                if i + 1 < width {
                    expected_neighbors += 1;
                }
                if j > 0 {
                    expected_neighbors += 1;
                }
                if j + 1 < height {
                    expected_neighbors += 1;
                }
                assert_eq!(nav.connections[id].len(), expected_neighbors);
            }
        }
    }

    #[test]
    fn test_distance_matrix_correctness() {
        let width = 5;
        let height = 5;
        let nav = generate_grid((0.0, 0.0), 10.0, width, height);
        let dm = DistanceMatrix::from(&nav);

        for i1 in 0..width {
            for j1 in 0..height {
                let u = i1 * height + j1;
                for i2 in 0..width {
                    for j2 in 0..height {
                        let v = i2 * height + j2;
                        let expected = (i1.abs_diff(i2) + j1.abs_diff(j2)) as i32;
                        assert_eq!(
                            dm.matrix[u][v], expected,
                            "Distance between ({i1},{j1}) and ({i2},{j2}) should match Manhattan distance"
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn test_distance_matrix_performance() {
        let width = 30;
        let height = 30;
        let nav = generate_grid((0.0, 0.0), 10.0, width, height);
        let start = std::time::Instant::now();
        let dm = DistanceMatrix::from(&nav);
        let elapsed = start.elapsed();
        println!("DistanceMatrix for 30x30 (900 nodes) took: {:?}", elapsed);
        assert_eq!(dm.matrix.len(), width * height);
        assert_eq!(
            dm.matrix[0][width * height - 1],
            (width - 1 + height - 1) as i32
        );
        assert!(elapsed.as_millis() < 100);
    }

    #[test]
    fn test_polygon_contains_point_and_boundary() {
        let rect = Polygon::rectangle(10.0, 20.0, 30.0, 40.0);
        // Inside
        assert!(rect.contains_point((20.0, 35.0)));
        assert!(rect.contains_point((11.0, 21.0)));
        assert!(rect.contains_point((39.0, 59.0)));

        // Outside
        assert!(!rect.contains_point((5.0, 35.0)));
        assert!(!rect.contains_point((45.0, 35.0)));
        assert!(!rect.contains_point((20.0, 15.0)));
        assert!(!rect.contains_point((20.0, 65.0)));

        // Distance to boundary
        assert!(rect.distance_to_boundary((10.0, 35.0)) < 1e-4);
        assert!((rect.distance_to_boundary((20.0, 35.0)) - 10.0).abs() < 1e-4);
    }

    #[test]
    fn test_polygon_centroid_computation() {
        // Triangle with known centroid at (10, 10)
        let tri = Polygon::from_vertices(vec![(0.0, 0.0), (30.0, 0.0), (0.0, 30.0)]);
        assert!((tri.centroid.0 - 10.0).abs() < 1e-4);
        assert!((tri.centroid.1 - 10.0).abs() < 1e-4);

        // Rectangle with centroid at (25, 40)
        let rect = Polygon::rectangle(10.0, 20.0, 30.0, 40.0);
        assert!((rect.centroid.0 - 25.0).abs() < 1e-4);
        assert!((rect.centroid.1 - 40.0).abs() < 1e-4);
    }

    #[test]
    fn test_segment_intersects_aabb() {
        // AABB [10, 20] x [10, 20]
        assert!(segment_intersects_aabb(
            (0.0, 15.0),
            (30.0, 15.0),
            10.0,
            10.0,
            20.0,
            20.0
        ));
        assert!(segment_intersects_aabb(
            (12.0, 12.0),
            (18.0, 18.0),
            10.0,
            10.0,
            20.0,
            20.0
        ));
        assert!(!segment_intersects_aabb(
            (0.0, 5.0),
            (30.0, 5.0),
            10.0,
            10.0,
            20.0,
            20.0
        ));
        assert!(!segment_intersects_aabb(
            (0.0, 0.0),
            (5.0, 5.0),
            10.0,
            10.0,
            20.0,
            20.0
        ));
    }

    #[test]
    fn test_bitmap_grid_point_to_polygon_and_back_uniform() {
        let width = 4;
        let height = 4;
        let grid_size = 20.0;
        let nav = generate_grid((0.0, 0.0), grid_size, width, height);

        // Fixed resolution of 4.0 world units per cell
        let bitmap = nav.build_bitmap_grid(4.0);

        // For each node in the grid, test points inside and test centroid round-trip
        for i in 0..width {
            for j in 0..height {
                let id = i * height + j;
                let centroid = nav.polygons[id].centroid;

                // 1. Back: polygon_id -> (x, y) location (centroid)
                let pt_back = bitmap.polygon_to_point(&nav, id);
                assert_eq!(pt_back, Some(centroid));

                // 2. Going: (x, y) centroid -> polygon_id
                let mapped_poly = bitmap.get_polygon_at(centroid.0, centroid.1, &nav);
                assert_eq!(mapped_poly, Some(id));

                // 3. Test arbitrary points inside this polygon
                let test_pts = [
                    (centroid.0 - 5.0, centroid.1 - 5.0),
                    (centroid.0 + 7.0, centroid.1 - 3.0),
                    (centroid.0 - 2.0, centroid.1 + 8.0),
                ];
                for pt in test_pts {
                    let res = bitmap.get_polygon_at(pt.0, pt.1, &nav);
                    assert_eq!(
                        res,
                        Some(id),
                        "Point ({}, {}) should map to polygon {}",
                        pt.0,
                        pt.1,
                        id
                    );
                }
            }
        }

        // Test points outside the grid bounds
        assert_eq!(bitmap.get_polygon_at(-10.0, 10.0, &nav), None);
        assert_eq!(bitmap.get_polygon_at(10.0, -10.0, &nav), None);
        assert_eq!(bitmap.get_polygon_at(200.0, 200.0, &nav), None);
    }

    #[test]
    fn test_bitmap_grid_non_uniform_navmesh() {
        let nav = generate_non_uniform_navmesh();
        let resolution = 2.0;
        let bitmap = nav.build_bitmap_grid(resolution);

        // Verify each non-uniform polygon can be reached from internal points and round-tripped
        for (id, poly) in nav.polygons.iter().enumerate() {
            // Centroid maps to this polygon
            let mapped_centroid = bitmap.get_polygon_at(poly.centroid.0, poly.centroid.1, &nav);
            assert_eq!(
                mapped_centroid,
                Some(id),
                "Centroid of polygon {} should map to itself",
                id
            );

            // Centroid lookup from polygon ID
            let centroid_back = bitmap.polygon_to_point(&nav, id);
            assert_eq!(centroid_back, Some(poly.centroid));
        }

        // Specific point checks:
        // Inside Node 0 (Large Central Hall: 180..320, 140..230)
        assert_eq!(bitmap.get_polygon_at(220.0, 180.0, &nav), Some(0));

        // Inside Node 1 (West Room: 60..130, 150..220)
        assert_eq!(bitmap.get_polygon_at(80.0, 170.0, &nav), Some(1));

        // Inside Node 2 (West Corridor: 130..180, 170..200)
        assert_eq!(bitmap.get_polygon_at(150.0, 185.0, &nav), Some(2));

        // Inside Node 3 (North Office: 205..295, 50..100)
        assert_eq!(bitmap.get_polygon_at(250.0, 75.0, &nav), Some(3));

        // Inside Node 7 (South Triangle: centroid around (250, 256.7))
        assert_eq!(bitmap.get_polygon_at(250.0, 250.0, &nav), Some(7));

        // Unoccupied space outside the navmesh
        assert_eq!(bitmap.get_polygon_at(10.0, 10.0, &nav), None);
        assert_eq!(bitmap.get_polygon_at(500.0, 500.0, &nav), None);
        assert_eq!(bitmap.get_polygon_at(100.0, 100.0, &nav), None);
    }

    #[test]
    fn test_bitmap_grid_performance() {
        let width = 25;
        let height = 25;
        let grid_size = 20.0;
        let nav = generate_grid((0.0, 0.0), grid_size, width, height);

        let build_start = std::time::Instant::now();
        let bitmap = nav.build_bitmap_grid(2.0);
        let build_time = build_start.elapsed();
        println!(
            "Bitmap grid build ({}x{} cells, 625 polygons) took: {:?}",
            bitmap.width, bitmap.height, build_time
        );
        assert!(build_time.as_millis() < 200);

        // Perform 50,000 arbitrary point lookups
        let query_start = std::time::Instant::now();
        let num_queries = 50_000;
        let mut found_count = 0;

        for k in 0..num_queries {
            let x = ((k * 37) % 550) as f32 - 25.0;
            let y = ((k * 43) % 550) as f32 - 25.0;
            if bitmap.get_polygon_at(x, y, &nav).is_some() {
                found_count += 1;
            }
        }

        let query_time = query_start.elapsed();
        println!(
            "{} arbitrary point queries took: {:?} ({:?} / query)",
            num_queries,
            query_time,
            query_time / num_queries as u32
        );

        assert!(found_count > 0);
        // Ensure average query time is well under 10 microseconds (typically ~100ns)
        assert!(query_time.as_millis() < 250);
    }

    #[test]
    fn test_bitmap_grid_serde() {
        let nav = generate_non_uniform_navmesh();
        let bitmap = nav.build_bitmap_grid(5.0);

        let json = serde_json::to_string(&bitmap).expect("Should serialize bitmap");
        let deserialized: PolygonBitmapGrid =
            serde_json::from_str(&json).expect("Should deserialize bitmap");

        assert_eq!(bitmap.width, deserialized.width);
        assert_eq!(bitmap.height, deserialized.height);
        assert_eq!(bitmap.resolution, deserialized.resolution);
        assert_eq!(bitmap.cells, deserialized.cells);
    }
}
