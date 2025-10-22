use std::rc::Rc;

use crate::conflicts::{ConflictTreeNode, ConflictType};

/// When converting between coordinate space, a grid clip mode
/// is in charge of figuring out how to handle rounding.
pub enum GridClipMode {
    ConservativeTopLeft,
    ConservativeBottomRight,
}

/// Internal collision checking structure for mapping between grid cells
pub struct MultiGridCollisionChecker {
    /// The size of an individual grid.
    /// It is assumed that each grid starts at the same top left corner
    pub grid_sizes: Vec<f32>,
}

impl MultiGridCollisionChecker {
    /// Gets the other blocked node
    pub fn get_blocked_nodes(&self, grid_id: usize, x: usize, y: usize) -> Vec<(usize, usize, usize)> {
        // Get the absolute x,y position in grid space
        let target_size = self.grid_sizes[grid_id];
        let (real_world_x, real_world_y) = (x as f32 * target_size, y as f32 * target_size);

        // Get the grid sizes
        let mut grids = vec![];
        for (id, &_grid_size) in self.grid_sizes.iter().enumerate() {
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

    // Takes in an (x,y) coordinate and returns the grid occupying it currently.
    pub fn get_grid_space(
        &self,
        grid_id: usize,
        x: f32,
        y: f32,
        mode: GridClipMode,
    ) -> (usize, usize) {
        let coords = (
            (x / self.grid_sizes[grid_id]),
            (y / self.grid_sizes[grid_id]),
        );

        match mode {
            GridClipMode::ConservativeTopLeft => {
                (coords.0.floor() as usize, coords.1.floor() as usize)
            }
            GridClipMode::ConservativeBottomRight => {
                (coords.0.ceil() as usize, coords.1.ceil() as usize)
            }
        }
    }

    pub fn build_moving_obstacle_map(
        &self,
        trajectories: &Vec<Vec<Vec<(i64, i64)>>>,
        boundaries: &Vec<(usize, usize)>,
    ) -> Result<Vec<ConflictTreeNode>, ()> {
        // agent_map
        let mut agent_map = vec![];
        let mut conflicts = vec![];
        // Assume all agents are the same.
        for graph in 0..trajectories.len() {
            let mut graphs_map = vec![];
            for t in 0..trajectories[graph].len() {
                let mut grid = vec![vec![None; boundaries[graph].0]; boundaries[graph].1];
                for agent in 0..trajectories[graph][t].len() {
                    let (ax, ay) = trajectories[graph][t][agent];
                    if ax < 0 || ay < 0 {
                        println!("Uh-oh");
                        continue;
                        //return Err(());
                    }
                    grid[ax as usize][ay as usize] = Some(agent);
                }
                graphs_map.push(grid);
            }
            agent_map.push(graphs_map);
        }

        let agent_map = Rc::new(agent_map);

        // Assume all agents are the same.
        for graph in 0..trajectories.len() {
            for t in 0..trajectories[graph].len() {
                for agent in 0..trajectories[graph][t].len() {
                    let (ax, ay) = trajectories[graph][t][agent];
                    let nodes = self.get_blocked_nodes(graph, ax as usize, ay as usize);
                    for node in nodes {
                        // Only consider multi-fleet
                        if node.0 == graph {
                            continue;
                        }
                        if let Some(agent1) = agent_map[node.0][t][node.1][node.2] {
                            // Mark conflict
                            conflicts.push(ConflictTreeNode {
                                agents_involved: ((node.0, agent1), (graph, agent)),
                                time: t,
                                conflict_type: ConflictType::Collision,
                                trajectory_map: agent_map.clone(),
                            });
                        }

                        // Handle swap
                        if t == 0 {
                            // We only check for swaps after they occur
                            continue;
                        }

                        let (from_x, from_y) = trajectories[graph][t - 1][agent];
                        let (from_x, from_y) = (from_x as usize, from_y as usize);
                        let blocked_nodes = self.get_blocked_nodes(graph, from_x, from_y);
                        for (graph2, x2, y2) in blocked_nodes {
                            if let Some(agent1) = agent_map[graph2][t][x2][y2] {
                                if agent_map[node.0][t - 1][node.1][node.2]
                                    == agent_map[graph2][t][x2][y2]
                                {
                                    conflicts.push(ConflictTreeNode {
                                        agents_involved: ((node.0, agent1), (graph, agent)),
                                        time: t,
                                        conflict_type: ConflictType::Swap,
                                        trajectory_map: agent_map.clone(),
                                    });
                                }
                            }
                        }
                    }
                }
            }
        }
        Ok(conflicts)
    }
}

#[cfg(test)]
#[test]
fn test_grid_space() {
    let shared_grid_space = MultiGridCollisionChecker {
        grid_sizes: vec![1.0, 2.0],
    };
    let other_blocked_nodes = shared_grid_space.get_blocked_nodes(0, 1, 1);
    let other_blocked_nodes2 = shared_grid_space.get_blocked_nodes(1, 0, 0);

    assert!(other_blocked_nodes.len() == 1);
    assert!(other_blocked_nodes.contains(&(1, 0, 0)));

    assert!(other_blocked_nodes2.len() == 4);
    assert!(other_blocked_nodes2.contains(&(0, 0, 1)));
    assert!(other_blocked_nodes2.contains(&(0, 1, 0)));
    assert!(other_blocked_nodes2.contains(&(0, 1, 1)));
    assert!(other_blocked_nodes2.contains(&(0, 0, 0)));
}