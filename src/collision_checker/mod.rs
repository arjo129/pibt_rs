/// When converting between coordinate space, the grid clip mode
/// is in charge of figuring out how to handle rounding.
enum GridClipMode {
    ConservativeTopLeft,
    ConservativeBottomRight,
}

/// Internal collision checking structure for mapping between grid cells
struct MultiGridCollisionChecker {
    /// The size of an individual grid.
    /// It is assumed that each grid starts at the same top left corner
    grid_sizes: Vec<f32>,
}

impl MultiGridCollisionChecker {
    /// Gets the other blocked node
    fn get_blocked_nodes(&self, grid_id: usize, x: usize, y: usize) -> Vec<(usize, usize, usize)> {
        // Get the absolute x,y position in grid space
        let target_size = self.grid_sizes[grid_id];
        let (real_world_x, real_world_y) = (x as f32 * target_size, y as f32 * target_size);

        // Get the grid sizes
        let mut grids = vec![];
        for (id, &grid_size) in self.grid_sizes.iter().enumerate() {
            if id == grid_id {
                continue;
            }
            let (tl_x, tl_y) = (real_world_x, real_world_y);
            let (br_x, br_y) = (real_world_x + target_size, real_world_y + target_size);
            let (start_x, start_y) =
                self.get_grid_space(id, tl_x, tl_y, GridClipMode::ConservativeTopLeft);

            let (end_x, end_y) =
                self.get_grid_space(id, br_x, br_y, GridClipMode::ConservativeBottomRight);

            for dx in start_x..end_x {
                for dy in start_y..end_y {
                    grids.push((id, dx, dy));
                }
            }
        }
        return grids;
    }

    fn get_grid_space(&self, grid_id: usize, x: f32, y: f32, mode: GridClipMode) -> (usize, usize) {
        let coords = (
            (x / self.grid_sizes[grid_id]),
            (y / self.grid_sizes[grid_id]),
        );

        match (mode) {
            GridClipMode::ConservativeTopLeft => {
                (coords.0.floor() as usize, coords.1.floor() as usize)
            }
            GridClipMode::ConservativeBottomRight => {
                (coords.0.ceil() as usize, coords.1.ceil() as usize)
            }
        }
    }
}
