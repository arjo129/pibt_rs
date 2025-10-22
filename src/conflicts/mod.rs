use std::rc::Rc;

use crate::collision_checker::MultiGridCollisionChecker;

/// Conflict Type
#[derive(Debug, Clone, Copy)]
pub enum ConflictType {
    Swap,
    Collision,
}

pub type MultiGraphTrajectory = Vec<Vec<Vec<Vec<Option<usize>>>>>;
/// Conflict Tree Node
#[derive(Debug, Clone)]
pub struct ConflictTreeNode {
    pub agents_involved: ((usize, usize), (usize, usize)),
    pub time: usize,
    pub conflict_type: ConflictType,
    pub trajectory_map: Rc<MultiGraphTrajectory>,
}

impl ConflictTreeNode {
    pub fn is_move_safe(
        &self,
        grid_id: usize,
        from: (usize, usize),
        to: (usize, usize),
        t: usize,
        collision_checker: &MultiGridCollisionChecker,
    ) -> bool {
        let occupying_from = collision_checker.get_blocked_nodes(grid_id, from.0, from.1);
        let occupying_to = collision_checker.get_blocked_nodes(grid_id, to.0, to.1);

        // Head-on-collision
        for &(g, x, y) in &occupying_to {
            if self.trajectory_map[g][t][x][y].is_some() {
                return false;
            }
        }

        if t < 1 {
            return true;
        }

        // Swap
        for &(g, x, y) in &occupying_from {
            if self.trajectory_map[g][t][x][y].is_some() {
                for &(g2, x2, y2) in &occupying_from {
                    if g2 != g {
                        continue;
                    }

                    if self.trajectory_map[g][t][x][y] == self.trajectory_map[g][t - 1][x2][y2] {
                        return false;
                    }
                }
            }
        }

        true
    }
}
