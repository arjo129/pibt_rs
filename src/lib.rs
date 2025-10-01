use std::collections::{BinaryHeap, HashMap, HashSet, VecDeque};

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
    CONSERVATIVE_TOP_LEFT,
    CONSERVATIVE_BOTTOM_RIGHT,
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
                self.get_grid_space(id, tl_x, tl_y, GridClipMode::CONSERVATIVE_TOP_LEFT);

            let (end_x, end_y) =
                self.get_grid_space(id, br_x, br_y, GridClipMode::CONSERVATIVE_BOTTOM_RIGHT);

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
            GridClipMode::CONSERVATIVE_TOP_LEFT => {
                (coords.0.floor() as usize, coords.1.floor() as usize)
            }
            GridClipMode::CONSERVATIVE_BOTTOM_RIGHT => {
                (coords.0.ceil() as usize, coords.1.ceil() as usize)
            }
        }
    }
}

struct HeterogenousTrajectory {
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

/// A Spatio-temporal reservation system.
struct HeterogenousReservationSystem {
    collision_checker: MultiGridCollisionChecker,
    grid_bounds: Vec<(usize, usize)>,
    occupied: Vec<Vec<Vec<Vec<HashSet<usize>>>>>, // time, graph_id, x, y, agents - Multiple agents can occupy a single cell as we need to take into account agents from different graphs
    agent_to_cells: Vec<Vec<Vec<(usize, usize, usize)>>>, // For book keeping when we need to rollback. Format is [time][agent_id][cells_affected]
    max_agents: usize,
    trajectory_max_id: usize,
    trajectories: HashMap<usize, TrajectoryRecord>,
    agent_last_location: HashMap<usize, (usize, usize, usize)>,
    unassigned_agents: Vec<Vec<Vec<HashMap<usize, EndTimeInfo>>>>, // Graph id, x,y, hashmap of agents ->  time
}

impl HeterogenousReservationSystem {
    /// Create a new Heterogenous reservation system
    fn new(grid_sizes: Vec<f32>, grid_bounds: Vec<(usize, usize)>, max_agents: usize) -> Self {
        let collision_checker = MultiGridCollisionChecker { grid_sizes };
        let unassigned_agents: Vec<_> = grid_bounds
            .iter()
            .map(|&(width, height)| vec![vec![HashMap::new(); height]; width])
            .collect();

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
                start_time: trajectory.start_time,
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
}

struct ProposedPath {
    path: Vec<(usize, usize, usize)>,
    need_to_moveout: Vec<usize>,
}
struct BestFirstSearchInstance<'a, 'b> {
    curr_loc: (usize, usize, usize),
    start_time: usize,
    dont_occupy: HashSet<(usize, usize, usize)>,
    distance_grid: &'b Vec<Vec<Vec<i64>>>,
    max_lookahead: usize,
    res_sys: &'a HeterogenousReservationSystem,
    pq: BinaryHeap<(usize, usize, (usize, usize, usize))>,
    came_from: HashMap<(usize, usize, usize, usize), (usize, usize, usize, usize)>,
    agent: usize
}

impl<'a, 'b> BestFirstSearchInstance<'a, 'b> {
    fn create_search_instance(
        res_sys: &'a HeterogenousReservationSystem,
        distance_grid: &'b Vec<Vec<Vec<i64>>>,
        curr_loc: (usize, usize, usize),
        dont_occupy: HashSet<(usize, usize, usize)>,
        start_time: usize,
        max_lookahead: usize,
        agent: usize
    ) -> Self {
        let mut pq = BinaryHeap::new();
        pq.push((0, start_time, curr_loc.clone()));
        BestFirstSearchInstance {
            curr_loc,
            start_time,
            dont_occupy,
            distance_grid,
            max_lookahead,
            res_sys,
            pq,
            came_from: HashMap::new(),
            agent
        }
    }
}

impl<'a, 'b> Iterator for BestFirstSearchInstance<'a, 'b> {
    type Item = ProposedPath;

    fn next(&mut self) -> Option<Self::Item> {
        while let Some(p) = self.pq.pop() {
            let (score, curr_time, (graph, x, y)) = p;
            if !self.dont_occupy.contains(&(graph, x, y)) {
                // Backtrack
                let mut node = (curr_time, graph, x, y);
                let mut v = vec![node.clone()];
                while let Some(p) = self.came_from.get(&node) {
                    v.push(p.clone());
                    node = *p;
                }
                v.reverse();

                let mut agents_to_kickout = HashSet::new();
                for &(time, graph, x, y) in v.iter() {
                    for (agent, end_time_info) in &self.res_sys.unassigned_agents[graph][x][y] {
                        if end_time_info.end_time < time {
                            agents_to_kickout.insert(*agent);
                        }
                    }

                    let other_nodes = self
                        .res_sys
                        .collision_checker
                        .get_blocked_nodes(graph, x, y);
                    for &(graph, x, y) in &other_nodes {
                        for (agent, end_time_info) in &self.res_sys.unassigned_agents[graph][x][y] {
                            if end_time_info.end_time < time {
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
                .map(|(dx, dy)| (x as i64 + dx, y as i64 + dy))
                .filter(|&(x, y)| {
                    x > 0
                        && y > 0
                        && x < (self.distance_grid[self.agent].len() as i64)
                        && y < (self.distance_grid[self.agent][0].len() as i64)
                })
                .map(|(x, y)| (x as usize, y as usize))
                .filter(|&(x, y)| self.distance_grid[self.agent][x][y] >= 0) // Static obstacles
                .filter(|&(x, y)| self.res_sys.occupied[curr_time + 1][graph][x][y].len() == 0) // Dynamic obstacles
                .filter(|&(x, y)| {
                    for agent in &self.res_sys.occupied[curr_time][graph][x][y] {
                        if self.res_sys.occupied[curr_time + 1][graph][p.2.1][p.2.2].contains(agent)
                        {
                            return false;
                        }
                    }
                    true
                });

            for (x, y) in neighbors {
                let tentative_g_score = curr_time + self.distance_grid[self.agent][x][y] as usize;
                if self.came_from.contains_key(&(curr_time + 1, graph, x, y)) {
                    continue;
                }
                self.pq
                    .push((tentative_g_score, curr_time + 1, (graph, x, y)));
                self.came_from.insert(
                    (curr_time + 1, graph, x, y),
                    (curr_time, graph, p.2.1, p.2.2),
                );
            }
        }
        return None;
    }
}

struct Agent {
    graph_id: usize,
    start: (usize, usize),
    end: (usize,usize)
}

fn evaluate_heterogenous_agent_grids(base_obstacles: &Vec<Vec<bool>>,
    graph_scale: Vec<f32>, agents: Vec<Agent>) -> Vec<Vec<Vec<i64>>>
{
    let mut distance_grids = vec![];
    for agent in agents {
        let grid = evaluate_individual_agent_cost(base_obstacles, &agent, &graph_scale);
        distance_grids.push(grid);
    }
    distance_grids
}

fn evaluate_individual_agent_cost(base_obstacles: &Vec<Vec<bool>>, agent: &Agent, graph_scale: &Vec<f32>) -> Vec<Vec<i64>>
{
    let width = (((base_obstacles[0].len() as f32)/ graph_scale[agent.graph_id]) as usize);
    let height = (((base_obstacles.len() as f32)/ graph_scale[agent.graph_id]) as usize);
    let mut distance_grid = vec![vec![-5;  width];  height];

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
    while let Some((node, score)) = queue.pop_front() {
        distance_grid[node.0][node.1]= score;
        let neighbours = [(-1,0),(0,-1), (1,0),(0,1)].iter().map(|(dx,dy)| (node.0 as i64 +dx, node.1 as i64+dy))
        .filter(|&(x,y)| x >= 0 && y >= 0 && x < width as i64 && y < height as i64)
        .map(|(x,y)| (x as usize, y as usize))
        .filter(|&(x,y)| distance_grid[x][y] == -5);
        for n in neighbours {
            queue.push_back((n, score+1));
        }
    }

    distance_grid
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
        positions: vec![(0,0), (0,1)]
    };
    let res = res_sys.reserve_trajectory(&trajectory2, 0);
    assert!(res.is_err());

    // Test swapping conflict
    let trajectory2 = HeterogenousTrajectory {
        graph_id: 1,
        start_time: 0,
        positions: vec![(0,1), (0,0)]
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
