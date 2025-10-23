use std::collections::{HashMap, HashSet};

use crate::collision_checker::MultiGridCollisionChecker;

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct HeterogenousTrajectory {
    pub(crate) graph_id: usize,
    pub(crate) start_time: usize,
    pub(crate) positions: Vec<(usize, usize)>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum ReservationError {
    GraphNotFound,
    OutOfGraphBounds(usize, usize, usize),
    TrajectoryForAgentAlreadyExists,
    TrajectoryOverWrites,
    TrajectoryEmpty,
    ExceedMaxAgents,
    AgentSwappedGraphs,
}

#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub(crate) struct TrajectoryRecord {
    pub(crate) start_time: usize,
    pub(crate) end_time: usize,
    pub(crate) agent_id: usize,
    pub(crate) previous_id: Option<usize>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub(crate) struct EndTimeInfo {
    pub(crate) end_time: usize,
    pub(crate) belongs_to: usize,
}

/// End time and location
#[derive(Clone, Copy, Debug)]
pub(crate) struct EndTimeAndLocation {
    pub(crate) end_time: usize,
    pub(crate) x: usize,
    pub(crate) y: usize,
}

/// A Spatio-temporal reservation system.
pub(crate) struct HeterogenousReservationSystem {
    pub(crate) collision_checker: MultiGridCollisionChecker,
    grid_bounds: Vec<(usize, usize)>,
    pub(crate) occupied: Vec<Vec<Vec<Vec<HashSet<usize>>>>>, // time, graph_id, x, y, agents - Multiple agents can occupy a single cell as we need to take into account agents from different graphs
    agent_to_cells: Vec<Vec<Vec<(usize, usize, usize)>>>, // For book keeping when we need to rollback. Format is [time][agent_id][cells_affected]
    pub(crate) max_agents: usize,
    trajectory_max_id: usize,
    trajectories: HashMap<usize, TrajectoryRecord>,
    pub(crate) agent_last_location: HashMap<usize, (usize, usize, usize)>, // agent->(graph_id,x ,y)
    pub(crate) unassigned_agents: Vec<Vec<Vec<HashMap<usize, EndTimeInfo>>>>, // Graph id, x,y, hashmap of agents ->  time
    pub(crate) agent_to_graph: Vec<Option<usize>>,
}

impl HeterogenousReservationSystem {
    /// Create a new Heterogenous reservation system
    pub(crate) fn new(
        grid_sizes: Vec<f32>,
        grid_bounds: Vec<(usize, usize)>,
        max_agents: usize,
    ) -> Self {
        let collision_checker = MultiGridCollisionChecker { grid_sizes };
        let unassigned_agents: Vec<_> = grid_bounds
            .iter()
            .map(|&(width, height)| vec![vec![HashMap::new(); height]; width])
            .collect();
        let agent_to_graph = vec![None; max_agents];

        Self {
            collision_checker,
            grid_bounds,
            occupied: vec![],
            agent_to_cells: vec![],
            max_agents,
            trajectory_max_id: 0,
            trajectories: HashMap::new(),
            agent_last_location: HashMap::new(),
            unassigned_agents,
            agent_to_graph,
        }
    }
    /// Attempts to add a trajectory to the list of trajectories
    /// returns the trjactory id of the trajectory if we can add the trajectory to the current set of trajectories
    /// otherwise it returns an error with the reason for which we cannot
    pub(crate) fn reserve_trajectory(
        &mut self,
        trajectory: &HeterogenousTrajectory,
        agent_id: usize,
    ) -> Result<Option<usize>, ReservationError> {
        if agent_id >= self.max_agents {
            return Err(ReservationError::ExceedMaxAgents);
        }

        if let Some(agent_graph) = self.agent_to_graph[agent_id] {
            if trajectory.graph_id != agent_graph {
                return Err(ReservationError::AgentSwappedGraphs);
            }
        } else {
            self.agent_to_graph[agent_id] = Some(trajectory.graph_id);
        }

        for (time_after_start, &position) in trajectory.positions.iter().enumerate() {
            let time = time_after_start + trajectory.start_time;
            if time >= self.occupied.len() {
                break;
            }
            if self.occupied[time].len() < trajectory.graph_id {
                return Err(ReservationError::GraphNotFound);
            }
            if self.agent_to_cells[time][agent_id].len() != 0 {
                return Err(ReservationError::TrajectoryForAgentAlreadyExists);
            }
            let (x, y) = position;
            if self.occupied[time][trajectory.graph_id].len() < x {
                return Err(ReservationError::OutOfGraphBounds(
                    trajectory.graph_id,
                    x,
                    y,
                ));
            }
            if self.occupied[time][trajectory.graph_id][x].len() < y {
                println!("Out of bounds");

                return Err(ReservationError::OutOfGraphBounds(
                    trajectory.graph_id,
                    x,
                    y,
                ));
            }

            if self.occupied[time][trajectory.graph_id][x][y].len() != 0 {
                // Spot is occupied
                println!(
                    "Attempted to occupy a reserved spot t={} p = {}, {}, {}",
                    time, trajectory.graph_id, x, y
                );
                return Err(ReservationError::TrajectoryOverWrites);
            }

            // Handle swap
            if time_after_start == 0 {
                // We only check for swaps after they occur
                continue;
            }

            let (from_x, from_y) = trajectory.positions[time_after_start - 1];
            for agent in &self.occupied[time][trajectory.graph_id][from_x][from_y] {
                if self.occupied[time - 1][trajectory.graph_id][x][y].contains(agent) {
                    println!("Attempted to swap.");
                    return Err(ReservationError::TrajectoryOverWrites);
                }
            }
        }

        println!("Attemping registration");
        let effective_start = trajectory.start_time.min(self.occupied.len());
        println!("DEBUG: {}", line!());

        /// Mark the end location and time.
        let Some(&(last_x, last_y)) = trajectory.positions.last() else {
            return Err(ReservationError::TrajectoryEmpty);
        };
        let mut previous_traj = None;
        let previous_location = self.agent_last_location.get(&agent_id).copied();

        if let Some(agent_last_loc) = previous_location {
            let (graph, x, y) = agent_last_loc;
            if let Some(end_time_info) = self.unassigned_agents[graph][x][y].remove(&agent_id) {
                previous_traj = Some(end_time_info.belongs_to);
            }
        }

        if let Some(prev_traj_id) = previous_traj {
            if let Some(prev_traj_record) = self.trajectories.get(&prev_traj_id) {
                let gap_start = prev_traj_record.end_time + 1;
                let gap_end = trajectory.start_time;
                if let Some((g, x, y)) = previous_location {
                    for t in gap_start..gap_end {
                        while t >= self.occupied.len() {
                            self.extend_by_one_timestep();
                        }
                        self.occupied[t][g][x][y].insert(agent_id);
                        self.agent_to_cells[t][agent_id].push((g, x, y));
                        for (cg, cx, cy) in self.collision_checker.get_blocked_nodes(g, x, y) {
                            if cx < self.grid_bounds[cg].0 && cy < self.grid_bounds[cg].1 {
                                self.occupied[t][cg][cx][cy].insert(agent_id);
                                self.agent_to_cells[t][agent_id].push((cg, cx, cy));
                            }
                        }
                    }
                }
            }
        }

        let new_end_time = trajectory.start_time + trajectory.positions.len();
        if new_end_time == 0 {
            // TODO: Find a better way to do this.
            return Err(ReservationError::TrajectoryEmpty);
        }
        let new_end_time = new_end_time - 1;
        println!(
            "Assigning last trajectoty {} {} {}",
            agent_id, last_x, last_y
        );
        self.unassigned_agents[trajectory.graph_id][last_x][last_y].insert(
            agent_id,
            EndTimeInfo {
                end_time: new_end_time,
                belongs_to: self.trajectory_max_id,
            },
        );
        self.trajectories.insert(
            self.trajectory_max_id,
            TrajectoryRecord {
                start_time: effective_start,
                end_time: new_end_time,
                agent_id,
                previous_id: previous_traj,
            },
        );
        self.trajectory_max_id += 1;

        for (time_after_start, &position) in trajectory.positions.iter().enumerate() {
            let time = time_after_start + trajectory.start_time;
            while time >= self.occupied.len() {
                self.extend_by_one_timestep();
            }
            let (x, y) = position;
            self.occupied[time][trajectory.graph_id][x][y].insert(agent_id);
            self.agent_to_cells[time][agent_id].push((trajectory.graph_id, x, y));
            let nodes = self
                .collision_checker
                .get_blocked_nodes(trajectory.graph_id, x, y);
            for (graph_id, x, y) in nodes {
                // Conversion may return out of bounds grids
                if x >= self.occupied[time][graph_id].len() {
                    continue;
                }
                if y >= self.occupied[time][graph_id][x].len() {
                    continue;
                }
                self.occupied[time][graph_id][x][y].insert(agent_id);
                self.agent_to_cells[time][agent_id].push((graph_id, x, y));
            }
        }
        self.agent_last_location
            .insert(agent_id, (trajectory.graph_id, last_x, last_y));

        Ok(Some(self.trajectory_max_id - 1))
    }

    // Internal method to remove trajectory
    // Note that behaves like a pop. You MUST remove trajectories in the reverse order from the order in which
    // they have been added
    pub(crate) fn remove_trajectory(&mut self, trajectory_id: usize) -> Result<(), ()> {
        let Some(trajectory_information) = self.trajectories.remove(&trajectory_id) else {
            return Err(());
        };
        let Some(agent_location) = self
            .agent_last_location
            .remove(&trajectory_information.agent_id)
        else {
            return Err(());
        };
        let (graph, x, y) = agent_location;
        // Remove agent from affected cells
        for time in trajectory_information.start_time..=trajectory_information.end_time {
            for &cell in &self.agent_to_cells[time][trajectory_information.agent_id] {
                self.occupied[time][cell.0][cell.1][cell.2]
                    .remove(&trajectory_information.agent_id);
            }
            self.agent_to_cells[time][trajectory_information.agent_id].clear();
        }

        let Some(_) = self.unassigned_agents[graph][x][y].remove(&trajectory_information.agent_id)
        else {
            return Err(());
        };

        let Some(restore_record) = trajectory_information.previous_id else {
            self.agent_to_graph[trajectory_information.agent_id] = None;
            return Ok(());
        };

        let Some(previous_traj) = self.trajectories.get(&restore_record) else {
            // This part of the code should not be reached.
            return Err(());
        };

        // We need to find the end location of the previous trajectory.
        // This is a bit of a hack, as we don't store the full trajectory.
        // We assume that the agent is at the start of the removed trajectory at the end time of the previous one.
        let (last_graph, last_x, last_y) = self.agent_to_cells[previous_traj.end_time]
            [trajectory_information.agent_id]
            .iter()
            .find(|(g, _, _)| *g == self.agent_to_graph[trajectory_information.agent_id].unwrap())
            .map(|(g, x, y)| (*g, *x, *y))
            .unwrap();

        self.agent_last_location.insert(
            trajectory_information.agent_id,
            (last_graph, last_x, last_y),
        );

        self.unassigned_agents[last_graph][last_x][last_y].insert(
            trajectory_information.agent_id,
            EndTimeInfo {
                end_time: previous_traj.end_time,
                belongs_to: restore_record,
            },
        );
        Ok(())
    }

    pub(crate) fn extend_by_one_timestep(&mut self) {
        let graphs: Vec<_> = self
            .grid_bounds
            .iter()
            .map(|&(width, height)| vec![vec![HashSet::new(); width]; height])
            .collect();

        self.occupied.push(graphs);
        self.agent_to_cells.push(vec![vec![]; self.max_agents]);
    }

    /// For visuallization
    pub(crate) fn get_agents_at_timestep(
        &self,
        time_step: usize,
    ) -> Vec<Option<(usize, usize, usize)>> {
        let agents = 0..self.max_agents;
        agents
            .map(|p| {
                let Some(graph_id) = self.agent_to_graph[p] else {
                    println!("Failed to get graph for agent");
                    return None;
                };
                let Some(&(_, x, y)) = self.agent_to_cells[time_step][p]
                    .iter()
                    .filter(|&(graph, _x, _y)| *graph == graph_id)
                    .next()
                else {
                    // If agent is not found at current timestep, return its last known position
                    return self.agent_last_location.get(&p).copied();
                };
                Some((graph_id, x, y))
            })
            .collect()
    }

    /// To retrieve the last time and distance to goal.
    pub(crate) fn get_agent_last_alloc_time(&self, agent: usize) -> Result<EndTimeAndLocation, ()> {
        let Some(&(graph, x, y)) = self.agent_last_location.get(&agent) else {
            return Err(());
        };
        let Some(agent) = self.unassigned_agents[graph][x][y].get(&agent) else {
            return Err(());
        };

        Ok(EndTimeAndLocation {
            end_time: agent.end_time,
            x,
            y,
        })
    }
}

#[cfg(test)]
#[test]
fn test_reservation_system_registration() {
    let grid_bounds = vec![(4, 4), (2, 2)];
    let mut res_sys = HeterogenousReservationSystem::new(vec![1.0, 2.0], grid_bounds, 5);

    let trajectory1 = HeterogenousTrajectory {
        graph_id: 0,
        start_time: 0,
        positions: vec![(1, 1), (1, 0), (0, 0)],
    };

    println!("===========================================================");
    let res = res_sys.reserve_trajectory(&trajectory1, 0);
    assert_eq!(res, Ok(Some(0)));

    // Check the ends are correctly set.
    let &agent_last_loc = res_sys.agent_last_location.get(&0usize).unwrap();
    assert_eq!(agent_last_loc, (0, 0, 0));
    let &end_time = res_sys.unassigned_agents[0][0][0].get(&0usize).unwrap();
    assert_eq!(end_time.end_time, 2);

    // Can't reserve the same trajectory twice
    println!("===========================================================");
    let res = res_sys.reserve_trajectory(&trajectory1, 1);
    assert_eq!(res, Err(ReservationError::TrajectoryOverWrites));

    // Try swapping in the same graph
    println!("===========================================================");
    let trajectory_swap = HeterogenousTrajectory {
        graph_id: 0,
        start_time: 1,
        positions: vec![(0, 0), (1, 0)],
    };
    let res = res_sys.reserve_trajectory(&trajectory_swap, 1);
    assert_eq!(res, Err(ReservationError::TrajectoryOverWrites));

    // Try overlapping trajectory on same graph
    let trajectory2 = HeterogenousTrajectory {
        graph_id: 0,
        start_time: 0,
        positions: vec![(1, 0), (0, 0), (0, 1)],
    };
    let res = res_sys.reserve_trajectory(&trajectory2, 1);
    assert_eq!(res, Ok(Some(1)));
}

#[cfg(test)]
#[test]
fn test_reservation_system_registration_across_graphs() {
    let grid_bounds = vec![(4, 4), (2, 2)];
    let mut res_sys = HeterogenousReservationSystem::new(vec![1.0, 2.0], grid_bounds.clone(), 5);

    let trajectory1 = HeterogenousTrajectory {
        graph_id: 0,
        start_time: 0,
        positions: vec![(1, 1), (1, 2)],
    };
    let res = res_sys.reserve_trajectory(&trajectory1, 0);
    assert_eq!(res, Ok(Some(0)));

    // Test blocking conflict
    let trajectory2 = HeterogenousTrajectory {
        graph_id: 1,
        start_time: 0,
        positions: vec![(0, 0), (0, 1)],
    };
    let res = res_sys.reserve_trajectory(&trajectory2, 0);
    assert!(res.is_err());
    // Test swapping conflict
    let trajectory2 = HeterogenousTrajectory {
        graph_id: 1,
        start_time: 0,
        positions: vec![(0, 1), (0, 0)],
    };
    let res = res_sys.reserve_trajectory(&trajectory2, 0);
    assert!(res.is_err());
}

#[cfg(test)]
#[test]
fn test_pop_trajectories() {
    let trajectory1 = HeterogenousTrajectory {
        graph_id: 0,
        start_time: 0,
        positions: vec![(1, 1), (1, 0), (0, 0)],
    };
    let trajectory2 = HeterogenousTrajectory {
        graph_id: 0,
        start_time: 3,
        positions: vec![(1, 1), (1, 0), (0, 0)],
    };

    let grid_bounds = vec![(4, 4), (2, 2)];
    let mut res_sys = HeterogenousReservationSystem::new(vec![1.0, 2.0], grid_bounds, 5);
    // Reserve first trajectory
    let res = res_sys.reserve_trajectory(&trajectory1, 0);
    assert_eq!(res, Ok(Some(0)));

    // Try re-reserving the trajectory again.
    let res = res_sys.reserve_trajectory(&trajectory1, 0);
    assert!(res.is_err());

    // Remove the first trajectory
    let res = res_sys.remove_trajectory(0);
    assert_eq!(res, Ok(()));

    // it should be safe to re-reserve the trajectory
    let res = res_sys.reserve_trajectory(&trajectory1, 0);
    assert_eq!(res, Ok(Some(1)));

    // Now lets add the second trajectory
    let res = res_sys.reserve_trajectory(&trajectory2, 0);
    assert_eq!(res, Ok(Some(2)));

    // We shouldn't be able to re-add that trajectory
    let res = res_sys.reserve_trajectory(&trajectory2, 0);
    assert!(res.is_err());

    // We should be able to remove it
    let res = res_sys.remove_trajectory(2);
    assert_eq!(res, Ok(()));

    // We shouldn't be able to re-add the first trajectory
    let res = res_sys.reserve_trajectory(&trajectory1, 0);
    assert!(res.is_err());

    // We should be able to re-add the removed trajectory
    let res = res_sys.reserve_trajectory(&trajectory2, 0);
    assert_eq!(res, Ok(Some(3)));
}

#[test]
fn test_disappearing_agent_on_pop() {
    let grid_bounds = vec![(4, 4), (2, 2)];
    let mut res_sys = HeterogenousReservationSystem::new(vec![1.0, 2.0], grid_bounds, 5);

    // Trajectory 1 for agent 0, from t=0 to t=3. Ends at (0,0).
    let trajectory1 = HeterogenousTrajectory {
        graph_id: 0,
        start_time: 0,
        positions: vec![(1, 1), (1, 0), (0, 0)],
    };
    let traj1_id = res_sys
        .reserve_trajectory(&trajectory1, 0)
        .unwrap()
        .unwrap();
    assert_eq!(traj1_id, 0);

    // Agent should be at (0,0) at the end of its trajectory.
    assert_eq!(res_sys.get_agents_at_timestep(2)[0], Some((0, 0, 0)));

    // Trajectory 2 for agent 0, from t=3 to t=5. Ends at (2,2).
    let trajectory2 = HeterogenousTrajectory {
        graph_id: 0,
        start_time: 3,
        positions: vec![(2, 1), (2, 2)],
    };
    let traj2_id = res_sys
        .reserve_trajectory(&trajectory2, 0)
        .unwrap()
        .unwrap();
    assert_eq!(traj2_id, 1);

    // Agent should be at (2,2) at t=4.
    assert_eq!(res_sys.get_agents_at_timestep(4)[0], Some((0, 2, 2)));

    // Now, remove trajectory 2.
    res_sys.remove_trajectory(traj2_id).unwrap();

    // After removing trajectory 2, the agent's last known location should be the end of trajectory 1, which is (0,0) at t=3.
    // Let's check this.
    let last_loc = res_sys.get_agent_last_alloc_time(0).unwrap();
    assert_eq!(last_loc.end_time, 2);
    assert_eq!(last_loc.x, 0);
    assert_eq!(last_loc.y, 0);

    // Now, add a new trajectory with a time gap.
    // Trajectory 3 for agent 0, from t=5 to t=7.
    let trajectory3 = HeterogenousTrajectory {
        graph_id: 0,
        start_time: 5,
        positions: vec![(3, 3), (3, 2)],
    };
    res_sys
        .reserve_trajectory(&trajectory3, 0)
        .unwrap()
        .unwrap();

    // During the gap (e.g., at t=4), the agent should be held at its last position from trajectory 1, which is (0,0).
    // The bug is that `agent_last_location` was cleared, so the agent will "disappear" at t=4.
    let agents_at_t4 = res_sys.get_agents_at_timestep(4);
    assert_eq!(
        agents_at_t4[0],
        Some((0, 0, 0)),
        "Agent should be held at previous location during gap"
    );
}
