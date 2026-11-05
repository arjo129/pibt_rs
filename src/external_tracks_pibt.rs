use rand::{rngs::StdRng, seq::SliceRandom, SeedableRng};
use std::collections::VecDeque;

pub struct PiBTWithExternalTracks {
    pub grid: Vec<Vec<usize>>,
    q: Vec<Vec<(i64, i64)>>,
    dist: Vec<Vec<Vec<i64>>>,
    other_agents: Vec<Vec<Vec<Option<usize>>>>,
    pub lookahead: usize,
    pub enforce_directionality: bool,
    // (x, y) -> list of track IDs that pass through this cell
    pos_to_tracks: Vec<Vec<Vec<usize>>>,
    // track_id -> set of valid directed edges (from_pos, to_pos)
    track_edges: Vec<std::collections::HashSet<((i64, i64), (i64, i64))>>,
    rng: StdRng,
}

impl PiBTWithExternalTracks {
    pub fn init(grid: Vec<Vec<usize>>) -> Self {
        let rows = grid.len();
        let cols = grid[0].len();
        Self {
            grid,
            q: vec![],
            dist: vec![],
            other_agents: vec![],
            lookahead: 0,
            enforce_directionality: false,
            pos_to_tracks: vec![vec![vec![]; cols]; rows],
            track_edges: vec![],
            rng: StdRng::seed_from_u64(42),
        }
    }

    pub fn set_seed(&mut self, seed: u64) {
        self.rng = StdRng::seed_from_u64(seed);
    }

    pub fn solve(
        &mut self,
        starts: &Vec<(usize, usize)>,
        ends: &Vec<(usize, usize)>,
        external_tracks: &Vec<Vec<(usize, usize)>>,
        max_time: usize,
    ) -> Result<Vec<Vec<(i64, i64)>>, ()> {
        let mut priorities = vec![];
        let num_internal = starts.len();
        let num_external = external_tracks.len();
        let total_agents = num_internal + num_external;

        // Reset constraints
        self.pos_to_tracks = vec![vec![vec![]; self.grid[0].len()]; self.grid.len()];
        self.track_edges = vec![std::collections::HashSet::new(); num_external];
        
        if self.enforce_directionality {
            for (track_id, track) in external_tracks.iter().enumerate() {
                for pos in track {
                    if !self.pos_to_tracks[pos.0][pos.1].contains(&track_id) {
                        self.pos_to_tracks[pos.0][pos.1].push(track_id);
                    }
                }
                for i in 0..track.len().saturating_sub(1) {
                    let curr = (track[i].0 as i64, track[i].1 as i64);
                    let next = (track[i+1].0 as i64, track[i+1].1 as i64);
                    if curr != next {
                        self.track_edges[track_id].insert((curr, next));
                    }
                }
            }
        }

        // SSSP for internal agents
        self.dist = vec![];
        for agent in 0..num_internal {
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

        // Initialize grids
        self.other_agents =
            vec![vec![vec![None; self.grid[0].len()]; self.grid.len()]; max_time + 1];
        self.q = vec![vec![(-1, -1); total_agents]; max_time + 1];

        // Set internal agents' starts
        for agent in 0..num_internal {
            self.q[0][agent] = (starts[agent].0 as i64, starts[agent].1 as i64);
            self.other_agents[0][starts[agent].0][starts[agent].1] = Some(agent);
        }

        // Set external agents' tracks
        for (i, track) in external_tracks.iter().enumerate() {
            let agent_id = num_internal + i;
            for (t, pos) in track.iter().enumerate() {
                if t > max_time {
                    break;
                }
                if self.q[t][agent_id] == (-1, -1) {
                    self.q[t][agent_id] = (pos.0 as i64, pos.1 as i64);
                }
                
                let end_t = (t + self.lookahead).min(max_time);
                for t_blocked in 0..=end_t {
                    if self.other_agents[t_blocked][pos.0][pos.1].is_none() {
                        self.other_agents[t_blocked][pos.0][pos.1] = Some(agent_id);
                    }
                }
            }
            // If track is shorter than max_time, external agent stays at last position
            if let Some(last_pos) = track.last() {
                for t in track.len()..=max_time {
                    if self.q[t][agent_id] == (-1, -1) {
                        self.q[t][agent_id] = (last_pos.0 as i64, last_pos.1 as i64);
                    }
                    
                    for t_blocked in 0..=max_time {
                        if self.other_agents[t_blocked][last_pos.0][last_pos.1].is_none() {
                            self.other_agents[t_blocked][last_pos.0][last_pos.1] = Some(agent_id);
                        }
                    }
                }
            }
        }

        // Iteratively solve the problem.
        for t in 1..max_time {
            // PiBT Logic for internal agents
            let mut agents: Vec<_> = (0..num_internal).collect();
            agents.sort_by(|p, q| priorities[*q].cmp(&priorities[*p]));
            for agent in agents {
                if self.q[t][agent] == (-1, -1) {
                    if !self.pibt(agent, t - 1, num_internal) {
                        // Fallback: stay at current position if PiBT fails
                        let (x, y) = self.q[t - 1][agent];
                        self.q[t][agent] = (x, y);
                        // Try to reserve it if not already taken
                        if self.other_agents[t][x as usize][y as usize].is_none() {
                            self.other_agents[t][x as usize][y as usize] = Some(agent);
                        }
                    }
                }
            }

            // Completion check and priority update
            let mut complete = true;
            for agent in 0..num_internal {
                if self.q[t][agent] != (ends[agent].0 as i64, ends[agent].1 as i64) {
                    priorities[agent] += 1;
                    complete = false;
                } else {
                    priorities[agent] = 0;
                }
            }

            if complete {
                // Return only tracks for internal agents
                let mut result = vec![vec![(-1, -1); num_internal]; t + 1];
                for time in 0..=t {
                    for agent in 0..num_internal {
                        result[time][agent] = self.q[time][agent];
                    }
                }
                return Ok(result);
            }
        }

        // If we reach here, we didn't complete in max_time, but we can still return what we have
        let mut result = vec![vec![(-1, -1); num_internal]; max_time];
        for time in 0..max_time {
            for agent in 0..num_internal {
                result[time][agent] = self.q[time][agent];
            }
        }
        Ok(result)
    }

    fn ssp_heuristic(&self, goal: &(i64, i64), for_agent: usize) -> i64 {
        self.dist[for_agent][goal.0 as usize][goal.1 as usize]
    }

    fn pibt(&mut self, agent: usize, time: usize, num_internal: usize) -> bool {
        let q_from = self.q[time][agent];
        let mut neighbors: smallvec::SmallVec<[(i64, i64); 5]> =
            [(1, 0), (0, 1), (0, -1), (-1, 0), (0, 0)]
                .iter()
                .map(|m| (q_from.0 + m.0, q_from.1 + m.1))
                .filter(|(x, y)| {
                    if *x < 0 || *y < 0 || *x >= self.grid.len() as i64 || *y >= self.grid[0].len() as i64 {
                        return false;
                    }
                    if self.grid[*x as usize][*y as usize] != 0 {
                        return false;
                    }
                    if self.ssp_heuristic(&(*x, *y), agent) < 0 {
                        return false;
                    }

                    // Directionality constraint
                    if self.enforce_directionality {
                        let u = q_from;
                        let v = (*x, *y);
                        if u != v {
                            // Check if this move is "against" any uncontrolled agent's flow.
                            let u_tracks = &self.pos_to_tracks[u.0 as usize][u.1 as usize];
                            let v_tracks = &self.pos_to_tracks[v.0 as usize][v.1 as usize];
                            
                            for &track_id in u_tracks {
                                if v_tracks.contains(&track_id) {
                                    // If moving between two nodes on the same track,
                                    // must follow the directed edge.
                                    if !self.track_edges[track_id].contains(&(u, v)) {
                                        return false;
                                    }
                                }
                            }
                            
                            // Also check if v to u is a forward edge in any track (prevent head-on)
                            for &track_id in v_tracks {
                                if u_tracks.contains(&track_id) {
                                    // already handled above
                                } else {
                                    // If we are entering a track from the side, 
                                    // we should ensure we are not moving "backwards" 
                                    // into a position that would immediately cause a head-on.
                                    // Actually, the simplest check is: if (v, u) is a track edge, then (u, v) is forbidden.
                                    if self.track_edges[track_id].contains(&(v, u)) {
                                        return false;
                                    }
                                }
                            }
                        }
                    }

                    true
                })
                .collect();

        neighbors.shuffle(&mut self.rng);

        neighbors.sort_by(|pos1, pos2| {
            let dist1 = self.ssp_heuristic(pos1, agent);
            let dist2 = self.ssp_heuristic(pos2, agent);
            dist1.cmp(&dist2)
        });

        for (nx, ny) in neighbors {
            let x = nx as usize;
            let y = ny as usize;

            // If someone already reserved this for time + 1, skip
            if self.other_agents[time + 1][x][y].is_some() {
                continue;
            }

            // Check who is here at time
            if let Some(agent_to_check) = self.other_agents[time][x][y] {
                // If it's me, I can just stay here (as long as no one else reserved it)
                if agent_to_check == agent {
                    self.other_agents[time + 1][x][y] = Some(agent);
                    self.q[time + 1][agent] = (x as i64, y as i64);
                    return true;
                }

                let other_agents_destination = self.q[time + 1][agent_to_check];

                // Swap conflict prevention
                if other_agents_destination == q_from {
                    continue;
                }

                // If the agent hasn't planned its move yet
                if other_agents_destination.0 < 0 && other_agents_destination.1 < 0 {
                    // Only push if it's an internal agent
                    if agent_to_check < num_internal {
                        // Pre-reserve to avoid others taking it while we recurse
                        self.other_agents[time + 1][x][y] = Some(agent);
                        self.q[time + 1][agent] = (x as i64, y as i64);

                        if self.pibt(agent_to_check, time, num_internal) {
                            return true;
                        } else {
                            // Backtrack
                            self.other_agents[time + 1][x][y] = None;
                            self.q[time + 1][agent] = (-1, -1);
                            continue;
                        }
                    } else {
                        // External agent cannot be pushed.
                        continue;
                    }
                } else {
                    // Agent already plans to move elsewhere
                    self.other_agents[time + 1][x][y] = Some(agent);
                    self.q[time + 1][agent] = (x as i64, y as i64);
                    return true;
                }
            } else {
                // Cell is empty at time and time + 1
                self.other_agents[time + 1][x][y] = Some(agent);
                self.q[time + 1][agent] = (x as i64, y as i64);
                return true;
            }
        }

        false
    }
}
