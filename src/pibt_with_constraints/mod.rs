use std::{collections::VecDeque, ops::Mul};

use crate::{conflicts::ConflictTreeNode, MultiGridCollisionChecker, collision_checker};

/// Vanilla priority based inheritance
///
/// This contains a basic PiBT implementation in rust.
pub struct PiBTWithConstraints {
    pub grid: Vec<Vec<usize>>,
    q: Vec<Vec<(i64, i64)>>,
    dist: Vec<Vec<Vec<i64>>>,
    other_agents: Vec<Vec<Vec<Option<usize>>>>,
}

impl PiBTWithConstraints {
    pub fn init_empty_world(width: usize, height: usize) -> Self {
        let grid = vec![vec![0; width]; height];

        Self {
            grid,
            q: vec![],
            dist: vec![],
            other_agents: vec![],
        }
    }

    pub fn init(grid: Vec<Vec<usize>>) -> Self {
        Self {
            grid,
            q: vec![],
            dist: vec![],
            other_agents: vec![],
        }
    }

    /// Returns
    pub fn solve(
        &mut self,
        starts: &Vec<(usize, usize)>,
        ends: &Vec<(usize, usize)>,
        max_time: usize,
        constraints: &Vec<ConflictTreeNode>,
        collision_checker: &MultiGridCollisionChecker,
    ) -> Result<Vec<Vec<(i64, i64)>>, ()> {
        let mut priorities = vec![];
        // SSSP for individual agents
        for agent in 0..starts.len() {
            self.dist
                .push(vec![vec![-1; self.grid[0].len()]; self.grid.len()]);

            let (x, y) = ends[agent];

            self.dist[agent][x][y] = 0;
            let mut queue: VecDeque<(usize, usize, usize)> = VecDeque::new();

            queue.push_back((x, y, 0));

            while let Some((x, y, dist)) = queue.pop_front() {
                let neighbor = [(1, 0), (0, 1), (0, -1), (-1, 0)]
                    .iter()
                    .map(|m| (x as i64 + m.0, y as i64 + m.1))
                    .filter(|(x, y)| {
                        *x >= 0
                            && *y >= 0
                            && *x < self.grid.len() as i64
                            && *y < self.grid[0].len() as i64
                            && self.grid[*x as usize][*y as usize] == 0
                    })
                    .map(|(x, y)| (x as usize, y as usize));
                for (x, y) in neighbor.into_iter() {
                    if self.dist[agent][x][y] != -1 {
                        continue;
                    }
                    self.dist[agent][x][y] = (dist + 1) as i64;
                    queue.push_back((x, y, dist + 1));
                }
            }
            priorities.push(self.dist[agent][starts[agent].0][starts[agent].1]);
        }
        println!("Calculated goal matrix");

        // Initialize grids
        self.other_agents =
            vec![vec![vec![None; self.grid[0].len()]; self.grid.len()]; max_time + 1];
        self.q = vec![vec![(-1, -1); starts.len()]; max_time + 1];

        for agent in 0..starts.len() {
            self.q[0][agent] = (starts[agent].0 as i64, starts[agent].1 as i64);
            self.other_agents[0][starts[agent].0][starts[agent].1] = Some(agent);
        }
        // Iteratively solve the problem.
        for t in 1..max_time - 1 {
            // PiBT Logic
            let mut agents: Vec<_> = (0..starts.len()).collect();
            agents.sort_by(|p, q| priorities[*p].cmp(&priorities[*q]));
            for agent in agents {
                if self.q[t][agent] == (-1, -1) {
                    self.pibt(agent, t - 1, constraints, collision_checker);
                }
            }

            // Completion check and priority update
            let mut complete = true;
            for agent in 0..starts.len() {
                if self.q[t][agent] != (ends[agent].0 as i64, ends[agent].1 as i64) {
                    priorities[agent] += 1;
                    complete = false;
                } else {
                    priorities[agent] -= priorities[agent];
                }
            }

            if complete {
                println!("Solved in {}", t);
                self.q.truncate(t);
                return Ok(self.q.clone());
            }
        }

        println!("{:?}", self.q);
        return Err(());
    }

    fn ssp_heuristic(&self, goal: &(i64, i64), for_agent: usize) -> i64 {
        self.dist[for_agent][goal.0 as usize][goal.1 as usize]
    }

    /// Implements vanilla PiBT for high speed multi-robot planning.
    /// - agent: usize
    /// - time: usize
    /// - Conflicts: Constraints set by CBS
    fn pibt(
        &mut self,
        agent: usize,
        time: usize,
        conflicts: &Vec<ConflictTreeNode>,
        collision_checker: &MultiGridCollisionChecker,
    ) {
        // (Agent, items to reserve in stack)
        let mut stack: Vec<(usize, Vec<(usize, usize, usize)>)> = vec![(agent, vec![])];

        while let Some((agent, to_reserve)) = stack.pop() {
            // Apply stack changes
            for (agent, x, y) in &to_reserve {
                self.other_agents[time + 1][*x][*y] = Some(*agent);
                self.q[time + 1][*agent] = (*x as i64, *y as i64);
            }

            // Get neighbours
            let q_from = self.q[time][agent];
            let mut neighbor: smallvec::SmallVec<[(i64, i64); 4]> =
                [(1, 0), (0, 1), (0, -1), (-1, 0)]
                    .iter()
                    .map(|m| (q_from.0 + m.0, q_from.1 + m.1))
                    .filter(|(x, y)| {
                        *x >= 0
                            && *y >= 0
                            && *x < self.grid.len() as i64
                            && *y < self.grid[0].len() as i64
                            && self.grid[(*x as usize)][(*y as usize)] == 0
                            && self.ssp_heuristic(&(*x, *y), agent) >= 0
                    })
                    .collect();
            neighbor.sort_by(|pos1, pos2| {
                let dist1 = self.ssp_heuristic(pos1, agent);
                let dist2 = self.ssp_heuristic(pos2, agent);
                dist1.cmp(&dist2)
            });

            let mut found_solution = false;
            for (x, y) in neighbor {
                // Crustacean stuff
                let x = x as usize;
                let y = y as usize;
                // If occupied skip
                if self.other_agents[time + 1][x][y].is_some() {
                    continue;
                }
                // Check conflict tree, if its unsafe skip.
                for conflict in conflicts {
                    conflict.is_move_safe(
                        conflict.agents_involved.0.0,
                        (q_from.0 as usize, q_from.1 as usize),
                        (x, y),
                        time + 1,
                        collision_checker,
                    );
                }
                // Swap conflict prevention
                if let Some(agent_to_check) = self.other_agents[time][x][y] {
                    // There is an agent on the destination square at the current time
                    let other_agents_destination = self.q[time + 1][agent_to_check];
                    if other_agents_destination == q_from {
                        // Swap Conflict
                        continue;
                    }

                    if other_agents_destination.0 < 0 && other_agents_destination.1 < 0 {
                        // inherit the priority of this agent
                        let mut to_reserve = to_reserve.clone();
                        to_reserve.push((agent, x, y));
                        stack.push((agent_to_check, to_reserve));
                    } else {
                        self.other_agents[time + 1][x][y] = Some(agent);
                        self.q[time + 1][agent] = (x as i64, y as i64);
                        // Agent already plans to yeet
                        found_solution = true;
                        break;
                    }
                } else {
                    self.other_agents[time + 1][x][y] = Some(agent);
                    self.q[time + 1][agent] = (x as i64, y as i64);
                    found_solution = true;
                    break;
                }
            }

            if !found_solution {
                // TODO(arjo): Backtrack
                for (agent, x, y) in &to_reserve {
                    self.other_agents[time + 1][*x][*y] = None;
                    self.q[time + 1][*agent] = (-1, -1);
                }
            }
        }
    }
}

pub fn hierarchical_cbs_pibt(
    starts: Vec<(usize, usize, usize)>,
    ends: Vec<(usize, usize)>,
    bounds: Vec<(usize, usize)>,
    grid_sizes: Vec<f32>,
) -> Vec<Vec<Vec<(i64, i64)>>> {
    let mut agent_to_graph = vec![];
    let mut max_graph = 0;
    for &(graph, _, _) in &starts {
        max_graph = max_graph.max(graph);
    }
    let mut pibt_starts = vec![vec![]; max_graph + 1];
    let mut pibt_ends = vec![vec![]; max_graph + 1];
    let mut graph_to_agent = vec![vec![]; max_graph + 1];

    let mut pibts = vec![];

    for &bound in &bounds {
        // TODO(arjoc)
        pibts.push(PiBTWithConstraints::init_empty_world(bound.0, bound.1));
    }

    for (index, &start) in starts.iter().enumerate() {
        pibt_starts[start.0].push((start.1, start.2));
        pibt_ends[start.0].push(ends[index]);
        graph_to_agent[start.0].push(index);
        agent_to_graph.push((start.0, pibt_starts[start.0].len() - 1));
    }

    let mut collision_checker = MultiGridCollisionChecker { grid_sizes };

    let mut conflict_list = VecDeque::new();
    conflict_list.push_front(vec![]);

    while let Some(p) = conflict_list.pop_back() {
        let mut result = vec![];
        let mut all_solved = true;
        for g_id in 0..pibts.len() {
            let Ok(p) = pibts[g_id].solve(
                &pibt_starts[g_id],
                &pibt_ends[g_id],
                1000,
                &vec![],
                &collision_checker,
            ) else {
                println!("{:?}", pibt_starts);
                println!("{:?}", pibt_ends);
                all_solved = false;
                continue;
            };
            result.push(p);
        }

        if !all_solved {
            continue;
        }

        let max_len = result.iter().map(|traj| traj.len()).max().unwrap_or(0);

        for traj in result.iter_mut() {
            if let Some(last_pos) = traj.last().cloned() {
                while traj.len() < max_len {
                    traj.push(last_pos.clone());
                }
            }
        }

        let Ok(conflicts) = collision_checker.build_moving_obstacle_map(&result, &bounds) else {
            panic!("Some inconsistency occured");
        };
        if conflicts.len() == 0 {
            return result;
        }
        for conflict in conflicts {
            conflict_list.push_front(vec![conflict]);
        }
    }
    panic!("Unsolvable");
}
