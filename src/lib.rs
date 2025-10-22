use std::{
    cmp::{Ordering, Reverse},
    collections::{BinaryHeap, HashMap, HashSet, VecDeque},
    rc::Rc,
};

pub mod collision_checker;
pub mod pibt_with_constraints;
pub mod hierarchical_cbs_pibt_wrapper;
pub mod reservation_system;
pub mod conflicts;

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

pub fn parse_grid_with_scale(file_name: &str, scale: usize) -> Vec<Vec<usize>> {
    let content = std::fs::read_to_string(file_name).expect("Failed to read the file");
    let mut scaled_grid = Vec::new(); // Changed name for clarity
    let mut lines = content.lines();

    // Skip the header lines
    lines.next(); // type octile
    lines.next(); // height
    lines.next(); // width
    lines.next(); // map

    // Check if scale is valid to prevent infinite loops or panics
    let final_scale = if scale == 0 { 1 } else { scale };

    for line in lines {
        let mut original_row = Vec::new();
        for char in line.chars() {
            match char {
                '.' => original_row.push(0),
                '@' => original_row.push(1),
                _ => continue,
            }
        }

        if original_row.is_empty() {
            continue;
        }

        // 1. Scale the row horizontally
        let mut scaled_row = Vec::new();
        for &cell_value in original_row.iter() {
            // Repeat the cell value `final_scale` times
            for _ in 0..final_scale {
                scaled_row.push(cell_value);
            }
        }

        // 2. Scale the row vertically
        // Repeat the entire scaled row `final_scale` times
        for _ in 0..final_scale {
            scaled_grid.push(scaled_row.clone()); // Use clone to push a copy of the row
        }
    }

    scaled_grid
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

use crate::collision_checker::MultiGridCollisionChecker;

use crate::reservation_system::{HeterogenousReservationSystem, HeterogenousTrajectory};

#[derive(Debug)]
struct ProposedPath {
    path: Vec<(usize, usize, usize)>,
    need_to_moveout: Vec<usize>,
}

/// This is used to implement pathfinding given the WinPiBT
/// concept of "Disentangled" paths.
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
                .contains(&(parent_graph, parent_x, parent_y))
                || self.distance_grid[self.agent][parent_x][parent_y] == 0
                || (self.curr_loc == (parent_graph, parent_x, parent_y)
                    && curr_time != self.start_time)
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

                    let other_nodes: Vec<_> = self
                        .res_sys
                        .collision_checker
                        .get_blocked_nodes(graph, x, y)
                        .iter()
                        .filter(|(g, x, y)| {
                            self.res_sys.unassigned_agents[*g].len() > *x
                                && self.res_sys.unassigned_agents[*g][*x].len() > *y
                        })
                        .cloned()
                        .collect();
                    for (graph, x, y) in other_nodes {
                        println!("Checking {:?}", (graph,x,y));
                        println!("Unassigned {:?}", self.res_sys.unassigned_agents);
                        for (agent, end_time_info) in &self.res_sys.unassigned_agents[graph][x][y] {
                            println!("Found agent {} blocking {:?} till {:?}", agent, (graph,x,y), end_time_info);
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

            if curr_time + 1 > self.start_time + self.max_lookahead || curr_time + 1 >= self.res_sys.occupied.len() {
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
                    self.res_sys.occupied[curr_time + 1][parent_graph].len() > x
                        && self.res_sys.occupied[curr_time + 1][parent_graph][x].len() > y
                })
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
                if x_idx >= distance_grid.len() {
                    continue;
                }
                if y_idx >= distance_grid[x_idx].len() {
                    continue;
                }
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
    fn attempt_solve_for_agent(&mut self, agent_id: usize, forward_lookup: usize) -> Vec<usize> {
        let Some(&(graph, x, y)) = self.reservation_system.agent_last_location.get(&agent_id)
        else {
            return vec![];
        };
        let Some(&end_time) = self.reservation_system.unassigned_agents[graph][x][y].get(&agent_id)
        else {
            return vec![];
        };
        let time = end_time.end_time-1;

        let mut stack = VecDeque::new();

        let mut blocked_nodes = HashSet::from_iter([(graph, x, y)].iter().cloned());
        let search = BestFirstSearchInstance::create_search_instance(
            &self.reservation_system,
            &self.cost_map,
            (graph, x, y),
            &blocked_nodes,
            time,
            forward_lookup,
            agent_id,
        );
        let mut will_affect: HashMap<usize, (usize, Vec<(usize, usize, usize)>)> = HashMap::new();
        for path in search {
            if path.need_to_moveout.len() > 1 {
                continue;
            }

            println!("Starting agent at {:?}", (graph,x,y));
            println!("Pushing path  {:?} for agent {}", path, agent_id);
            // Hack even though it DFS, we want the earliest node to be expanded to
            // be the first one generated.
            stack.push_back((agent_id, blocked_nodes.clone(), forward_lookup, 0, path));
        }

        while let Some((agent_id, blocked_locations, forward_lookup, depth, neighbour)) =
            stack.pop_front()
        {
            println!("Expanding agent {} {:?}", agent_id, neighbour);
            if neighbour.need_to_moveout.len() == 0 {
                let mut agent = agent_id;
                let mut path_to_reserve = HeterogenousTrajectory {
                    graph_id: neighbour.path[0].0,
                    start_time: end_time.end_time.clone(),
                    positions: neighbour.path.iter().map(|&(_, x, y)| (x, y)).collect(),
                };
                println!("Agent {}", agent);
                println!("{:?}", path_to_reserve);
                println!("First path for agent {}: {:?}", agent_id, path_to_reserve);

                while let Err(e) = self
                    .reservation_system
                    .reserve_trajectory(&path_to_reserve, agent) {
                        println!("Delaying");
                        path_to_reserve.start_time +=1;
                    }

                // Cascade the delays back up the chain
                while let Some((agent_id, path)) = will_affect.get(&agent) {
                    agent = *agent_id;
                    let mut hypot_path = HeterogenousTrajectory {
                        graph_id: path[0].0,
                        start_time: path_to_reserve.start_time,
                        positions: path.iter().map(|&(_, x, y)| (x, y)).collect(),
                    };
                    while let Err(e) =  self
                        .reservation_system
                        .reserve_trajectory(&hypot_path, *agent_id)
                    {
                        hypot_path.start_time +=1;
                                     }
                    println!("Chosen path for agent {}: {:?}", agent_id, hypot_path);
                }
                return vec![];
            }

            println!("Exploring path for {}, {}", agent_id, depth);
            let mut c = blocked_locations.clone();
            for &p in &neighbour.path {
                c.insert(p);
                for blocked_node in self.reservation_system.collision_checker.get_blocked_nodes(p.0, p.1, p.2) {
                    c.insert(blocked_node);
                }
            }

            if will_affect.contains_key(&neighbour.need_to_moveout[0]) || neighbour.need_to_moveout.len() > 1 {
                // Deadlock. Do not proceed
                println!("Deadlock");
                continue;
            }

            let my_size = self.reservation_system.collision_checker.grid_sizes
                [self.reservation_system.agent_to_graph[agent_id].unwrap()];
            let other_size = self.reservation_system.collision_checker.grid_sizes
                [self.reservation_system.agent_to_graph[neighbour.need_to_moveout[0]].unwrap()];
            let mut forward_lookup = forward_lookup;
            if other_size < my_size {
                let factor = (my_size / other_size).round() as usize;
                forward_lookup *= factor * factor;
                forward_lookup = forward_lookup.max(20);
            }
            let Some(p) = self.reservation_system.agent_last_location.get(&neighbour.need_to_moveout[0]) else {
                continue;
            };

             let Some(end_time) = self.reservation_system.unassigned_agents[p.0][p.1][p.2].get(&neighbour.need_to_moveout[0]) else {
                continue;
            };

            c.insert(*p);
            println!("pos: {:?} {:?} {}", p, c, forward_lookup);
            // Try to get the robot to move out
            let search = BestFirstSearchInstance::create_search_instance(
                &self.reservation_system,
                &self.cost_map,
                *p,
                &c,
                end_time.end_time - 1,
                forward_lookup,
                neighbour.need_to_moveout[0],
            );
            for path in search {
                if path.need_to_moveout.len() > 1 {
                    continue;
                }

                will_affect.insert(
                    neighbour.need_to_moveout[0],
                    (agent_id, neighbour.path.clone()),
                );
                println!("Adding path for {} {:?}", neighbour.need_to_moveout[0], path);
                stack.push_front((
                    neighbour.need_to_moveout[0],
                    c.clone(),
                    forward_lookup,
                    depth + 1,
                    path,
                ));
            }
        }
        let path_to_reserve = HeterogenousTrajectory {
            graph_id: graph,
            start_time: end_time.end_time.clone(),
            positions: vec![(x, y)],
        };
        let Ok(Some(p)) = self.reservation_system
            .reserve_trajectory(&path_to_reserve, agent_id)
            else {
                return vec![];
            };
        return vec![p];
    }

    pub fn solve(&mut self, max_time_steps: usize) -> Option<usize> {
        let mut agent_priorities: Vec<_> = (0..self.reservation_system.max_agents)
            .map(|p| {
                let Some(&(graph, x, y)) = self.reservation_system.agent_last_location.get(&p)
                else {
                    panic!("Solver was not properly initiallized");
                };
                Some((
                    (self.cost_map[p][x][y] as f32) / self.cost_map[p].len() as f32,
                    p,
                ))
            })
            .collect();

        // Hashmap tracks agent priority.
        let mut last_prio: HashMap<_, _> = agent_priorities
            .iter()
            .enumerate()
            .filter(|p| p.1.is_some())
            .map(|(ind, opt)| {
                let (_, b) = opt.unwrap();
                (b, ind)
            })
            .collect();
        for step in 1..max_time_steps {
            let agents = 0..self.reservation_system.max_agents;
            let mut flg_fin = true;
            let mut checked = false;
            println!("===========================================");
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

                    println!("Priority at current step {:?} for {}", x, a);
                    if cost == 0 {
                        x.0 -= x.0.floor();
                    } else {
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

            last_prio = agent_priorities
                .iter()
                .enumerate()
                .filter(|p| p.1.is_some())
                .map(|(ind, opt)| {
                    let (_, b) = opt.unwrap();
                    (b, ind)
                })
                .collect();
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
    println!("{:?}", paths);
    assert_eq!(paths.len(), 3);
    // Best case we try to move near the goal forcing the blocking agent out
    assert_eq!(paths[0].path, vec![(1, 0, 0), (1, 0, 1)]);
    // Second best case is we stay put
    assert_eq!(paths[1].path, vec![(1, 0, 0), (1, 0, 0)]);
    // We could oscillate out of the way but it is much less desireable
    assert_eq!(paths[2].path, vec![(1, 0, 0), (1, 1, 0)]);
    assert_eq!(paths[0].need_to_moveout, [0]);
    assert_eq!(paths[1].need_to_moveout, []);
}

