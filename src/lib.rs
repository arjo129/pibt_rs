use std::{
    cmp::{Ordering, Reverse},
    collections::{BinaryHeap, HashMap, HashSet, VecDeque},
};

/// Vanilla priority based inheritance
///
/// This contains a basic PiBT implementation in rust.
pub struct PiBT {
    pub grid: Vec<Vec<usize>>,
    q: Vec<Vec<(i64, i64)>>,
    dist: Vec<Vec<Vec<i64>>>,
    other_agents: Vec<Vec<Vec<Option<usize>>>>,
}

impl PiBT {
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

    pub fn solve(
        &mut self,
        starts: &Vec<(usize, usize)>,
        ends: &Vec<(usize, usize)>,
        max_time: usize,
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
                    self.pibt(agent, t - 1);
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
                self.q.truncate(max_time - t);
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
    fn pibt(&mut self, agent: usize, time: usize) {
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

pub fn parse_grid(file_name: &str) -> Vec<Vec<usize>> {
    let content = std::fs::read_to_string(file_name).expect("Failed to read the file");
    let mut grid = Vec::new();
    let mut lines = content.lines();

    // Skip the header lines
    lines.next(); // type octile
    lines.next(); // height
    lines.next(); // width
    lines.next(); // map

    for line in lines {
        let mut row = Vec::new();
        for char in line.chars() {
            match char {
                '.' => row.push(0),
                '@' => row.push(1),
                _ => continue,
            }
        }
        if !row.is_empty() {
            grid.push(row);
        }
    }
    grid
}

pub fn parse_scen(file_name: &str) -> Result<Vec<Vec<(usize, usize)>>, std::num::ParseIntError> {
    let content = std::fs::read_to_string(file_name).expect("Failed to read the file");
    let mut lines = content.lines();

    lines.next();

    let mut starts = vec![];
    let mut ends = vec![];
    for line in lines {
        let pts: Vec<_> = line.split_ascii_whitespace().collect();

        let start_x: usize = pts[4].parse()?;
        let start_y: usize = pts[5].parse()?;
        let end_x: usize = pts[6].parse()?;
        let end_y: usize = pts[7].parse()?;

        starts.push((start_x, start_y));
        ends.push((end_x, end_y));
    }
    Ok(vec![starts, ends])
}

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

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct HeterogenousTrajectory {
    graph_id: usize,
    start_time: usize,
    positions: Vec<(usize, usize)>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ReservationError {
    GraphNotFound,
    OutOfGraphBounds(usize, usize, usize),
    TrajectoryForAgentAlreadyExists,
    TrajectoryEmpty,
    ExceedMaxAgents,
    AgentSwappedGraphs,
}

#[derive(Clone, Debug, PartialEq, Eq, Hash)]
struct TrajectoryRecord {
    start_time: usize,
    end_time: usize,
    agent_id: usize,
    previous_id: Option<usize>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
struct EndTimeInfo {
    end_time: usize,
    belongs_to: usize,
}

/// End time and location
#[derive(Clone, Copy, Debug)]
struct EndTimeAndLocation {
    end_time: usize,
    x: usize,
    y: usize,
}

/// A Spatio-temporal reservation system.
struct HeterogenousReservationSystem {
    collision_checker: MultiGridCollisionChecker,
    grid_bounds: Vec<(usize, usize)>,
    occupied: Vec<Vec<Vec<Vec<HashSet<usize>>>>>, // time, graph_id, x, y, agents - Multiple agents can occupy a single cell as we need to take into account agents from different graphs
    agent_to_cells: Vec<Vec<Vec<(usize, usize, usize)>>>, // For book keeping when we need to rollback. Format is [time][agent_id][cells_affected]
    max_agents: usize,
    trajectory_max_id: usize,
    trajectories: HashMap<usize, TrajectoryRecord>,
    agent_last_location: HashMap<usize, (usize, usize, usize)>, // agent->(graph_id,x ,y)
    unassigned_agents: Vec<Vec<Vec<HashMap<usize, EndTimeInfo>>>>, // Graph id, x,y, hashmap of agents ->  time
    agent_to_graph: Vec<Option<usize>>,
}

impl HeterogenousReservationSystem {
    /// Create a new Heterogenous reservation system
    fn new(grid_sizes: Vec<f32>, grid_bounds: Vec<(usize, usize)>, max_agents: usize) -> Self {
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
    fn reserve_trajectory(
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
                return Err(ReservationError::OutOfGraphBounds(
                    trajectory.graph_id,
                    x,
                    y,
                ));
            }

            if self.occupied[time][trajectory.graph_id][x][y].len() != 0 {
                // Spot is occupied
                return Ok(None);
            }

            // Handle swap
            if time_after_start == 0 {
                // We only check for swaps after they occur
                continue;
            }

            let (from_x, from_y) = trajectory.positions[time_after_start - 1];
            for agent in &self.occupied[time][trajectory.graph_id][from_x][from_y] {
                if self.occupied[time - 1][trajectory.graph_id][x][y].contains(agent) {
                    return Ok(None);
                }
            }
        }

        let mut effective_start = trajectory.start_time.min(self.occupied.len());

        for (time_after_start, &position) in trajectory.positions.iter().enumerate() {
            let time = time_after_start + trajectory.start_time;
            while time >= self.occupied.len() {
                self.extend_by_one_timestep();
                let Some(&(g, x, y)) = self.agent_last_location.get(&agent_id) else {
                    let (x, y) = trajectory.positions[0];
                    let g = trajectory.graph_id;
                    self.occupied[time][g][x][y].insert(agent_id);
                    self.agent_to_cells[time][agent_id].push((g, x, y));
                    for (g, x, y) in self.collision_checker.get_blocked_nodes(g, x, y) {
                        self.occupied[time][g][x][y].insert(agent_id);
                        self.agent_to_cells[time][agent_id].push((g, x, y));
                    }
                    continue;
                };
                self.occupied[time][g][x][y].insert(agent_id);
                self.agent_to_cells[time][agent_id].push((g, x, y));
                for (g, x, y) in self.collision_checker.get_blocked_nodes(g, x, y) {
                    self.occupied[time][g][x][y].insert(agent_id);
                    self.agent_to_cells[time][agent_id].push((g, x, y));
                }
            }
            let (x, y) = position;
            self.occupied[time][trajectory.graph_id][x][y].insert(agent_id);
            self.agent_to_cells[time][agent_id].push((trajectory.graph_id, x, y));
            let nodes = self
                .collision_checker
                .get_blocked_nodes(trajectory.graph_id, x, y);
            for (graph_id, x, y) in nodes {
                self.occupied[time][graph_id][x][y].insert(agent_id);
                self.agent_to_cells[time][agent_id].push((graph_id, x, y));
            }
        }

        let Some(&(last_x, last_y)) = trajectory.positions.last() else {
            return Err(ReservationError::TrajectoryEmpty);
        };
        let mut previous_traj = None;
        if let Some(&agent_last_loc) = self.agent_last_location.get(&agent_id) {
            let (graph, x, y) = agent_last_loc;
            if let Some(end_time_info) = self.unassigned_agents[graph][x][y].remove(&agent_id) {
                previous_traj = Some(end_time_info.belongs_to);
            }
        }
        let new_end_time = trajectory.start_time + trajectory.positions.len();
        self.agent_last_location
            .insert(agent_id, (trajectory.graph_id, last_x, last_y));
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
        Ok(Some(self.trajectory_max_id - 1))
    }

    // Internal method to remove trajectory
    // Note that behaves like a pop. You MUST remove trajectories in the reverse order from the order in which
    // they have been added
    fn remove_trajectory(&mut self, trajectory_id: usize) -> Result<(), ()> {
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
        for time in trajectory_information.start_time..trajectory_information.end_time {
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
            return Ok(());
        };

        let Some(previous_traj) = self.trajectories.get(&restore_record) else {
            return Err(());
        };
        self.unassigned_agents[graph][x][y].insert(
            trajectory_information.agent_id,
            EndTimeInfo {
                end_time: previous_traj.end_time,
                belongs_to: restore_record,
            },
        );
        Ok(())
    }

    fn extend_by_one_timestep(&mut self) {
        let graphs: Vec<_> = self
            .grid_bounds
            .iter()
            .map(|&(width, height)| vec![vec![HashSet::new(); width]; height])
            .collect();

        self.occupied.push(graphs);
        self.agent_to_cells.push(vec![vec![]; self.max_agents]);
    }

    /// For visuallization
    fn get_agents_at_timestep(&self, time_step: usize) -> Vec<Option<(usize, usize, usize)>> {
        let agents = 0..self.max_agents;
        agents
            .map(|p| {
                let Some(graph_id) = self.agent_to_graph[p] else {
                    return None;
                };
                let Some(&(_, x, y)) = self.agent_to_cells[time_step][p]
                    .iter()
                    .filter(|&(graph, _x, _y)| *graph == graph_id)
                    .next()
                else {
                    return None;
                };
                Some((graph_id, x, y))
            })
            .collect()
    }

    /// To retrieve the last time and distance to goal.
    fn get_agent_last_alloc_time(&self, agent: usize) -> Result<EndTimeAndLocation, ()> {
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

#[derive(Debug)]
struct ProposedPath {
    path: Vec<(usize, usize, usize)>,
    need_to_moveout: Vec<usize>,
}
struct BestFirstSearchInstance<'a, 'b, 'c> {
    curr_loc: (usize, usize, usize),
    start_time: usize,
    dont_occupy: &'c HashSet<(usize, usize, usize)>,
    distance_grid: &'b Vec<Vec<Vec<i64>>>,
    max_lookahead: usize,
    res_sys: &'a HeterogenousReservationSystem,
    pq: BinaryHeap<Reverse<(usize, usize, (usize, usize, usize))>>,
    came_from: HashMap<(usize, usize, usize, usize), (usize, usize, usize, usize)>,
    agent: usize,
}

impl<'a, 'b, 'c> BestFirstSearchInstance<'a, 'b, 'c> {
    fn create_search_instance(
        res_sys: &'a HeterogenousReservationSystem,
        distance_grid: &'b Vec<Vec<Vec<i64>>>,
        curr_loc: (usize, usize, usize),
        dont_occupy: &'c HashSet<(usize, usize, usize)>,
        start_time: usize,
        max_lookahead: usize,
        agent: usize,
    ) -> Self {
        let mut pq = BinaryHeap::new();
        pq.push(Reverse((0, start_time, curr_loc.clone())));
        BestFirstSearchInstance {
            curr_loc,
            start_time,
            dont_occupy,
            distance_grid,
            max_lookahead,
            res_sys,
            pq,
            came_from: HashMap::new(),
            agent,
        }
    }
}

impl<'a, 'b, 'c> Iterator for BestFirstSearchInstance<'a, 'b, 'c> {
    type Item = ProposedPath;

    fn next(&mut self) -> Option<Self::Item> {
        while let Some(Reverse(p)) = self.pq.pop() {
            let (score, curr_time, (parent_graph, parent_x, parent_y)) = p;
            if !self
                .dont_occupy
                .contains(&(parent_graph, parent_x, parent_y)) || self.distance_grid[self.agent][parent_x][parent_y] == 0
            {
                // Backtrack
                let mut node = (curr_time, parent_graph, parent_x, parent_y);
                let mut v = vec![node.clone()];
                while let Some(p) = self.came_from.get(&node) {
                    v.push(p.clone());
                    node = *p;
                }
                v.reverse();

                let mut agents_to_kickout = HashSet::new();
                for &(time, graph, x, y) in v.iter() {
                    let time = self.start_time + time;
                    for (agent, end_time_info) in &self.res_sys.unassigned_agents[graph][x][y] {
                        if end_time_info.end_time <= time && *agent != self.agent {
                            agents_to_kickout.insert(*agent);
                        }
                    }

                    let other_nodes = self
                        .res_sys
                        .collision_checker
                        .get_blocked_nodes(graph, x, y);
                    for &(graph, x, y) in &other_nodes {
                        for (agent, end_time_info) in &self.res_sys.unassigned_agents[graph][x][y] {
                            if end_time_info.end_time <= time {
                                agents_to_kickout.insert(*agent);
                            }
                        }
                    }
                }

                return Some(ProposedPath {
                    path: v.iter().map(|&(_, graph, x, y)| (graph, x, y)).collect(),
                    need_to_moveout: agents_to_kickout.iter().cloned().collect(),
                });
            }

            if curr_time + 1 > self.start_time + self.max_lookahead {
                continue;
            }

            let neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0), (0, 0)]
                .iter()
                .map(|(dx, dy)| (parent_x as i64 + dx, parent_y as i64 + dy))
                .filter(|&(x, y)| {
                    x >= 0
                        && y >= 0
                        && x < (self.distance_grid[self.agent].len() as i64)
                        && y < (self.distance_grid[self.agent][0].len() as i64)
                })
                .map(|(x, y)| (x as usize, y as usize))
                .filter(|&(x, y)| self.distance_grid[self.agent][x][y] >= 0) // Static obstacles
                .filter(|&(x, y)| {
                    self.res_sys.occupied.len() >= (curr_time + 1)
                        || self.res_sys.occupied[curr_time + 1][parent_graph][x][y].len() == 0
                }) // Dynamic obstacles
                .filter(|&(x, y)| {
                    for agent in &self.res_sys.occupied[curr_time][parent_graph][x][y] {
                        if self.res_sys.occupied[curr_time + 1][parent_graph][p.2.1][p.2.2]
                            .contains(agent)
                        {
                            return false;
                        }
                    }
                    true
                });

            for (x, y) in neighbors {
                let tentative_g_score = curr_time + self.distance_grid[self.agent][x][y] as usize;
                if self
                    .came_from
                    .contains_key(&(curr_time + 1, parent_graph, x, y))
                {
                    continue;
                }
                self.pq.push(Reverse((
                    tentative_g_score,
                    curr_time + 1,
                    (parent_graph, x, y),
                )));
                self.came_from.insert(
                    (curr_time + 1, parent_graph, x, y),
                    (curr_time, parent_graph, p.2.1, p.2.2),
                );
            }
        }
        return None;
    }
}

/// Heterogenous Agent configuration
#[derive(Debug, Clone)]
pub struct HeterogenousAgent {
    pub graph_id: usize,
    pub start: (usize, usize),
    pub end: (usize, usize),
}

/// Runs breadth first search to evaluate the distance for each agent to its goal
/// base_obstacles - The obstacle grid base that is true if there is an obstacle false if there isnt
/// graph_scale - The size of each planning cell for each agent
/// agents - Each agent's start and goal
pub fn evaluate_heterogenous_agent_grids(
    base_obstacles: &Vec<Vec<bool>>,
    graph_scale: &Vec<f32>,
    agents: &Vec<HeterogenousAgent>,
) -> Vec<Vec<Vec<i64>>> {
    let mut distance_grids = vec![];
    for agent in agents {
        let grid = evaluate_individual_agent_cost(base_obstacles, &agent, &graph_scale);
        distance_grids.push(grid);
    }
    distance_grids
}

/// Runs BFS for an individual agent and returns the cost for each location.
fn evaluate_individual_agent_cost(
    base_obstacles: &Vec<Vec<bool>>,
    agent: &HeterogenousAgent,
    graph_scale: &Vec<f32>,
) -> Vec<Vec<i64>> {
    let width = (((base_obstacles[0].len() as f32) / graph_scale[agent.graph_id]) as usize);
    let height = (((base_obstacles.len() as f32) / graph_scale[agent.graph_id]) as usize);

    println!("Dimensions: {} x {}", width, height);
    let mut distance_grid = vec![vec![-5; width]; height];

    for x in 0..base_obstacles.len() {
        for y in 0..base_obstacles[0].len() {
            if base_obstacles[x][y] {
                let x = x as f32;
                let y = y as f32;
                let x_idx = (x / graph_scale[agent.graph_id]) as usize;
                let y_idx = (y / graph_scale[agent.graph_id]) as usize;
                distance_grid[x_idx][y_idx] = -1;
            }
        }
    }

    let mut queue = VecDeque::new();
    queue.push_back((agent.end, 0));
    let directions = [(-1, 0), (0, -1), (1, 0), (0, 1)];
    distance_grid[agent.end.0][agent.end.1] = 0;

    while let Some((node, score)) = queue.pop_front() {
        let (x, y) = node;
        let (x, y) = (x as i64, y as i64);
        for &(dx, dy) in &directions {
            let nx = x + dx;
            let ny = y + dy;

            // Check bounds
            if nx >= 0 && ny >= 0 && nx < width as i64 && ny < height as i64 {
                let n_usize = (nx as usize, ny as usize);
                // Check if unvisited
                if distance_grid[n_usize.0][n_usize.1] == -5 {
                    queue.push_back((n_usize, score + 1));
                    distance_grid[n_usize.0][n_usize.1] = score + 1;
                }
            }
        }
    }

    distance_grid
}

/// Heterogenous PiBT using the reasoning module
pub struct HetPiBT {
    cost_map: Vec<Vec<Vec<i64>>>,
    reservation_system: HeterogenousReservationSystem,
}

impl HetPiBT {
    pub fn init_solver(
        base_obstacles: &Vec<Vec<bool>>,
        graph_scale: Vec<f32>,
        grid_bounds: Vec<(usize, usize)>,
        agents: Vec<HeterogenousAgent>,
    ) -> Self {
        let cost_map = evaluate_heterogenous_agent_grids(base_obstacles, &graph_scale, &agents);
        let mut reservation_system =
            HeterogenousReservationSystem::new(graph_scale, grid_bounds, agents.len());
        for (agent_id, agent) in agents.iter().enumerate() {
            let start_traj = HeterogenousTrajectory {
                graph_id: agent.graph_id,
                start_time: 0,
                positions: vec![agent.start],
            };
            reservation_system
                .reserve_trajectory(&start_traj, agent_id)
                .unwrap();
        }

        Self {
            cost_map,
            reservation_system,
        }
    }

    /// Attempt to solve for agent
    fn attempt_solve_for_agent(&mut self, agent_id: usize, forward_lookup: usize) {
        let Some(&(graph, x, y)) = self.reservation_system.agent_last_location.get(&agent_id)
        else {
            return;
        };
        let Some(&end_time) = self.reservation_system.unassigned_agents[graph][x][y].get(&agent_id)
        else {
            return;
        };
        let time = end_time.end_time - 1;

        let mut stack = vec![];
        stack.push((
            agent_id,
            HashSet::from_iter([(graph, x, y)].iter().cloned()),
            forward_lookup,
        ));
        let mut will_affect = HashMap::new();
        while let Some((agent_id, blocked_locations, forward_lookup)) = stack.pop() {
            // Try to get the robot to move out
            let search = BestFirstSearchInstance::create_search_instance(
                &self.reservation_system,
                &self.cost_map,
                (graph, x, y),
                &blocked_locations,
                time,
                forward_lookup,
                agent_id,
            );
            let proposals: Vec<_> = search.collect();
            for neighbour in proposals {
                if neighbour.need_to_moveout.len() > 1 {
                    continue;
                }
                if neighbour.need_to_moveout.len() == 1 {
                    let mut c = blocked_locations.clone();
                    for &p in &neighbour.path {
                        c.insert(p);
                    }

                    if will_affect.contains_key(&neighbour.need_to_moveout[0]) {
                        // Deadlock. Do not proceed
                        continue;
                    }

                    let my_size = self.reservation_system.collision_checker.grid_sizes
                        [self.reservation_system.agent_to_graph[agent_id].unwrap()];
                    let other_size = self.reservation_system.collision_checker.grid_sizes[self
                        .reservation_system
                        .agent_to_graph[neighbour.need_to_moveout[0]]
                        .unwrap()];
                    let mut forward_lookup = forward_lookup;
                    if other_size < my_size {
                        let factor = (my_size / other_size).round() as usize;
                        forward_lookup *= factor * factor;
                    }
                    stack.push((neighbour.need_to_moveout[0], c, forward_lookup));
                    will_affect.insert(
                        neighbour.need_to_moveout[0],
                        (agent_id, neighbour.path.clone()),
                    );
                } else {
                    let mut agent = agent_id;
                    let path_to_reserve = HeterogenousTrajectory {
                        graph_id: neighbour.path[0].0,
                        start_time: end_time.end_time.clone(),
                        positions: neighbour.path.iter().map(|&(_, x, y)| (x, y)).collect(),
                    };
                    println!("Agent {}", agent);
                    println!("{:?}", path_to_reserve);
                    self.reservation_system
                        .reserve_trajectory(&path_to_reserve, agent_id)
                        .map_err(|p| panic!("{:?}", p));

                    // Cascade the delays back up the chain
                    while let Some((agent_id, path)) = will_affect.get(&agent) {
                        agent = *agent_id;
                        let mut hypot_path = HeterogenousTrajectory {
                            graph_id: path[0].0,
                            start_time: path_to_reserve.start_time,
                            positions: path.iter().map(|&(_, x, y)| (x, y)).collect(),
                        };
                        while let Err(p) = self
                            .reservation_system
                            .reserve_trajectory(&hypot_path, *agent_id)
                        {
                            hypot_path.start_time += 1;
                        }
                    }
                    return;
                }
            }
            let path_to_reserve = HeterogenousTrajectory {
                graph_id: graph,
                start_time: end_time.end_time.clone(),
                positions: vec![(x, y)],
            };
            self.reservation_system
                .reserve_trajectory(&path_to_reserve, agent_id)
                .unwrap();
            return;
        }
    }

    pub fn solve(&mut self, max_time_steps: usize) -> Option<usize> {
        let mut agent_priorities: Vec<_> = (0..self.reservation_system.max_agents).map(|p|{
            let Some(&(graph,x,y)) = self.reservation_system.agent_last_location.get(&p) else {
                panic!("Solver was not properly initiallized");
            };
            Some(((self.cost_map[p][x][y] as f32) / self.cost_map[p].len() as f32,p))
        }).collect();

        // Hashmap tracks agent priority.
        let mut last_prio: HashMap<_,_> =
            agent_priorities.iter().enumerate().filter(|p| p.1.is_some())
            .map(|(ind, opt)|{
                let (_,b )= opt.unwrap();
                (b,ind)}).collect();
        for step in 1..max_time_steps {
            let agents = 0..self.reservation_system.max_agents;
            let mut flg_fin = true;
            let mut checked = false;
            self.reservation_system.extend_by_one_timestep();
            for a in agents {
                let agent = self
                    .reservation_system
                    .get_agent_last_alloc_time(a)
                    .unwrap();

                if agent.end_time <= step {
                    checked = true;
                    let cost = self.cost_map[a][agent.x][agent.y];
                    println!("cost at current step {}", cost);
                    let Some(&idx) = last_prio.get(&a) else {
                        panic!("Could not find ");
                    };
                    let Some(x) = agent_priorities[idx].as_mut() else {
                            panic!("Failed to calculate cost");
                        };

                    // For debugging
                    assert!(x.1 == a);

                    println!("Priority at current step {:?} for {}",x, a);
                    if cost == 0 {
                        x.0 -= x.0.floor();
                    }
                    else {
                        x.0 += 1.0;
                        flg_fin = false;
                    }
                }
            }
            if !checked {
                continue;
            }
            if flg_fin {
                return Some(step);
            }
            agent_priorities.sort_by(|a, b| {
                let Some(a) = a else {
                    let Some(b) = b else {
                        return Ordering::Equal;
                    };
                    return Ordering::Greater;
                };

                let Some(b) = b else {
                    return Ordering::Less;
                };
                a.partial_cmp(b).unwrap()
            });

            last_prio =
            agent_priorities.iter().enumerate().filter(|p| p.1.is_some())
            .map(|(ind, opt)|{
                let (_,b )= opt.unwrap();
                (b,ind)}).collect();
            for &agent in &agent_priorities {
                let Some((_cost, agent_id)) = agent else {
                    continue;
                };
                let last_time = self
                    .reservation_system
                    .get_agent_last_alloc_time(agent_id)
                    .unwrap();
                if last_time.end_time > step {
                    continue;
                }
                self.attempt_solve_for_agent(agent_id, 1);
            }
        }
        None
    }

    pub fn get_trajectories(&self, time_step: usize) -> Vec<Option<(usize, usize, usize)>> {
        self.reservation_system.get_agents_at_timestep(time_step)
    }
}

#[cfg(test)]
#[test]
fn test_best_first_search() {
    let base_obstacles = vec![
        vec![false, false, false, false],
        vec![false, false, false, false],
        vec![false; 4],
        vec![false; 4],
    ];
    let grid_bounds = vec![(4, 4), (2, 2)];
    let mut graph_scale = vec![1.0, 2.0];
    let agent1 = HeterogenousAgent {
        graph_id: 0,
        start: (0, 3),
        end: (3, 3),
    };
    let agent2 = HeterogenousAgent {
        graph_id: 1,
        start: (0, 0),
        end: (0, 1),
    };
    let agents = vec![agent1, agent2];
    let mut het_pibt = HetPiBT::init_solver(&base_obstacles, graph_scale, grid_bounds, agents);
    het_pibt.reservation_system.extend_by_one_timestep();
    //het_pibt.attempt_solve_for_agent(1,1);

    let blocked = HashSet::from_iter([(1, 0, 0)].iter().cloned());
    let search = BestFirstSearchInstance::create_search_instance(
        &het_pibt.reservation_system,
        &het_pibt.cost_map,
        (1, 0, 0),
        &blocked,
        0,
        1,
        1,
    );
    let paths: Vec<_> = search.collect();
    assert_eq!(paths.len(), 2);
    assert_eq!(paths[0].path, vec![(1, 0, 0), (1, 0, 1)]);
    assert_eq!(paths[1].path, vec![(1, 0, 0), (1, 1, 0)]);
    assert_eq!(paths[0].need_to_moveout, [0]);
    assert_eq!(paths[1].need_to_moveout, []);
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

    let res = res_sys.reserve_trajectory(&trajectory1, 0);
    assert_eq!(res, Ok(Some(0)));

    // Check the ends are correctly set.
    let &agent_last_loc = res_sys.agent_last_location.get(&0usize).unwrap();
    assert_eq!(agent_last_loc, (0, 0, 0));
    let &end_time = res_sys.unassigned_agents[0][0][0].get(&0usize).unwrap();
    assert_eq!(end_time.end_time, 3);

    // Can't reserve the same trajectory twise
    let res = res_sys.reserve_trajectory(&trajectory1, 1);
    assert_eq!(res, Ok(None));

    // Try swapping in the same graph
    let trajectory_swap = HeterogenousTrajectory {
        graph_id: 0,
        start_time: 1,
        positions: vec![(0, 0), (1, 0)],
    };
    let res = res_sys.reserve_trajectory(&trajectory_swap, 1);
    assert_eq!(res, Ok(None));

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
    let mut res_sys = HeterogenousReservationSystem::new(vec![1.0, 2.0], grid_bounds, 5);

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
