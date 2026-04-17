use hetpibt::external_tracks_pibt::PiBTWithExternalTracks;
use std::collections::HashMap;
use std::sync::Arc;
use std::time::{Instant, Duration};
use rand::{SeedableRng, Rng, rngs::StdRng, seq::SliceRandom};
use mapf_post::{
    mapf_post, MapfResult, Trajectory, WaypointFollower,
    na::{Isometry2, Vector2},
    shape::Ball,
};

const OBSTACLE_SPEED: f32 = 4.0; 
const DT: f32 = 0.1;
const MAX_SIM_TIME: f32 = 100.0;
const PIBT_DT: f32 = 1.0 / OBSTACLE_SPEED;

struct MovingObstacle {
    path: Vec<Vector2<f32>>,
    follower: Option<WaypointFollower>,
    agent_id: usize,
    current_pos: Vector2<f32>,
    goal: Vector2<f32>,
    uncontrolled: bool,
    stall_timer: f32,
    original_path: Vec<Vector2<f32>>, 
}

impl MovingObstacle {
    fn new(path: Vec<Vector2<f32>>, agent_id: usize, uncontrolled: bool) -> Self {
        let start_pos = path[0];
        let goal = *path.last().unwrap();
        Self {
            path: path.clone(),
            follower: None,
            agent_id,
            current_pos: start_pos,
            goal,
            uncontrolled,
            stall_timer: 0.0,
            original_path: path,
        }
    }

    fn update(&mut self, dt: f32, claim_dict: &HashMap<usize, mapf_post::CurrentlyAllocatedTrajSegment>, seed: u64, stall_prob: f32) {
        if self.path.is_empty() { return; }
        if self.stall_timer > 0.0 {
            self.stall_timer -= dt;
            return;
        }

        if let Some(follower) = &mut self.follower {
            let can_move = if self.uncontrolled { true } 
            else if let Some(segment) = claim_dict.get(&self.agent_id) {
                follower.get_semantic_waypoint().trajectory_index < segment.end_id
            } else { false };

            if can_move {
                let next_wp = follower.next_waypoint();
                let target_pos = Vector2::new(next_wp.translation.x, next_wp.translation.y);
                let dir = target_pos - self.current_pos;
                let dist = dir.norm();
                
                if dist > 0.001 {
                    let move_dist = (OBSTACLE_SPEED * dt).min(dist);
                    self.current_pos += dir.normalize() * move_dist;
                } else {
                    let wp_idx = follower.get_semantic_waypoint().trajectory_index;
                    let mut rng = StdRng::seed_from_u64(seed + self.agent_id as u64 * 1000 + wp_idx as u64);
                    let mut extra_stall = 0.0;
                    while rng.random_range(0.0..1.0) < stall_prob { extra_stall += 0.3; }
                    if extra_stall > 0.0 {
                        self.stall_timer = extra_stall;
                        return;
                    }
                }
                let isometry = Isometry2::new(Vector2::new(self.current_pos.x, self.current_pos.y), 0.0);
                follower.update_position_estimate(&isometry, 0.1);
            }
        }
    }
}

fn generate_scenario(
    size: usize, 
    num_uncontrolled: usize, 
    num_controlled: usize, 
    seed: u64
) -> (Vec<MovingObstacle>, Vec<Vec<bool>>) {
    let mut rng = StdRng::seed_from_u64(seed);
    let grid = vec![vec![false; size]; size];
    let mut agents = vec![];
    let mut occupied_cells = std::collections::HashSet::new();

    for _ in 0..num_uncontrolled {
        for _ in 0..100 {
            let is_horizontal = rng.random_bool(0.5);
            let mut track = vec![];
            if is_horizontal {
                let y = rng.random_range(0..size);
                let reverse = rng.random_bool(0.5);
                if reverse { for x in (0..size).rev() { track.push(Vector2::new(x as f32, y as f32)); } }
                else { for x in 0..size { track.push(Vector2::new(x as f32, y as f32)); } }
            } else {
                let x = rng.random_range(0..size);
                let reverse = rng.random_bool(0.5);
                if reverse { for y in (0..size).rev() { track.push(Vector2::new(x as f32, y as f32)); } }
                else { for y in 0..size { track.push(Vector2::new(x as f32, y as f32)); } }
            }
            
            let start = (track[0].y.round() as usize, track[0].x.round() as usize);
            if !occupied_cells.contains(&start) {
                for pos in &track { occupied_cells.insert((pos.y.round() as usize, pos.x.round() as usize)); }
                agents.push(MovingObstacle::new(track, agents.len(), true));
                break;
            }
        }
    }

    let mut starts = vec![];
    let mut ends = vec![];
    let mut possible_cells: Vec<(usize, usize)> = vec![];
    for r in 0..size { for c in 0..size { possible_cells.push((r, c)); } }

    for _ in 0..num_controlled {
        possible_cells.shuffle(&mut rng);
        let mut found = false;
        for i in 0..possible_cells.len() {
            let s = possible_cells[i];
            if !occupied_cells.contains(&s) {
                for j in 0..possible_cells.len() {
                    let e = possible_cells[j];
                    if s != e {
                        starts.push(s);
                        ends.push(e);
                        occupied_cells.insert(s);
                        found = true;
                        break;
                    }
                }
            }
            if found { break; }
        }
    }

    let external_tracks: Vec<Vec<(usize, usize)>> = agents.iter()
        .filter(|a| a.uncontrolled)
        .map(|a| a.path.iter().map(|p| (p.y.round() as usize, p.x.round() as usize)).collect())
        .collect();

    let pibt_grid = vec![vec![0; size]; size];
    let mut solver = PiBTWithExternalTracks::init(pibt_grid);
    solver.lookahead = 2; 
    solver.enforce_directionality = true;
    solver.set_seed(seed);

    let result = solver.solve(&starts, &ends, &external_tracks, 100);

    if let Ok(trajectories) = result {
        if !trajectories.is_empty() {
            let num_internal_agents = trajectories[0].len();
            for i in 0..num_internal_agents {
                let mut path = vec![];
                for t in 0..trajectories.len() {
                    let pos = trajectories[t][i];
                    path.push(Vector2::new(pos.1 as f32, pos.0 as f32));
                }
                agents.push(MovingObstacle::new(path, agents.len(), false));
            }
        }
    }
    (agents, grid)
}

#[derive(Default, Clone)]
struct LatencyStats {
    pibt_times: Vec<Duration>,
    post_times: Vec<Duration>,
    total_replan_times: Vec<Duration>,
}

fn check_for_detailed_violation_mixed(plan: &mapf_post::SemanticPlan, current_pos: &[mapf_post::SemanticWaypoint]) -> bool {
    let mut agent_to_pos = HashMap::new();
    for &wp in current_pos { agent_to_pos.insert(wp.agent, wp); }
    for wp in current_pos {
        if let Some(deps) = plan.comes_before(wp) {
            for &dep_idx in deps {
                let dep_wp = plan.waypoints[dep_idx];
                if dep_wp.agent == wp.agent { continue; }
                if let Some(other_agent) = agent_to_pos.get(&dep_wp.agent) {
                    if other_agent.trajectory_index >= dep_wp.trajectory_index { return true; }
                }
            }
        }
    }
    false
}

fn replan(
    agents: &mut [MovingObstacle], 
    grid: &Vec<Vec<bool>>, 
    lookahead: usize, 
    seed: u64,
    sim_time: f32,
    stats: &mut LatencyStats,
) -> mapf_post::SemanticPlan {
    let total_start = Instant::now();
    
    let mut pibt_grid = vec![vec![0; grid[0].len()]; grid.len()];
    for r in 0..grid.len() {
        for c in 0..grid[0].len() {
            if grid[r][c] { pibt_grid[r][c] = 1; }
        }
    }

    let mut solver = PiBTWithExternalTracks::init(pibt_grid);
    solver.lookahead = lookahead;
    solver.enforce_directionality = true;
    solver.set_seed(seed + (sim_time * 1000.0) as u64);

    let mut starts = vec![];
    let mut ends = vec![];
    let mut internal_agent_ids = vec![];

    for obs in agents.iter() {
        if !obs.uncontrolled {
            starts.push((obs.current_pos.y.round() as usize, obs.current_pos.x.round() as usize));
            ends.push((obs.goal.y.round() as usize, obs.goal.x.round() as usize));
            internal_agent_ids.push(obs.agent_id);
        }
    }

    let mut external_tracks: Vec<Vec<(usize, usize)>> = vec![];
    for obs in agents.iter_mut() {
        if obs.uncontrolled {
            let mut track = vec![];
            let current_wp_idx = obs.follower.as_mut().map(|f| f.get_semantic_waypoint().trajectory_index).unwrap_or(0);
            for j in current_wp_idx..obs.original_path.len() {
                let p = obs.original_path[j];
                track.push((p.y.round() as usize, p.x.round() as usize));
            }
            if track.is_empty() {
                let p = obs.original_path.last().unwrap();
                track.push((p.y.round() as usize, p.x.round() as usize));
            }
            external_tracks.push(track);
        }
    }

    let pibt_start = Instant::now();
    let result = solver.solve(&starts, &ends, &external_tracks, 100);
    stats.pibt_times.push(pibt_start.elapsed());

    if let Ok(pibt_trajectories) = result {
        if !pibt_trajectories.is_empty() {
            for (i, &agent_id) in internal_agent_ids.iter().enumerate() {
                let mut new_path = vec![];
                for t in 0..pibt_trajectories.len() {
                    let pos = pibt_trajectories[t][i];
                    new_path.push(Vector2::new(pos.1 as f32, pos.0 as f32));
                }
                for obs in agents.iter_mut() {
                    if obs.agent_id == agent_id {
                        obs.path = new_path;
                        break;
                    }
                }
            }
        }
    }

    let mut post_trajectories = vec![];
    let mut footprints = vec![];
    for obs in agents.iter_mut() {
        if obs.uncontrolled {
            let current_wp_idx = obs.follower.as_mut().map(|f| f.get_semantic_waypoint().trajectory_index).unwrap_or(0);
            let mut remaining = vec![];
            remaining.push(obs.current_pos); 
            for j in (current_wp_idx + 1)..obs.original_path.len() {
                remaining.push(obs.original_path[j]);
            }
            if remaining.len() < 2 {
                let last = *obs.original_path.last().unwrap();
                remaining = vec![last, last];
            }
            obs.path = remaining;
        } else {
            if !obs.path.is_empty() { obs.path[0] = obs.current_pos; }
        }
        let poses = obs.path.iter().map(|p| Isometry2::new(*p, 0.0)).collect();
        post_trajectories.push(Trajectory { poses });
        footprints.push(Arc::new(Ball::new(0.45)) as Arc<dyn mapf_post::shape::Shape>);
    }

    let post_start = Instant::now();
    let plan = mapf_post(&MapfResult {
        trajectories: post_trajectories,
        footprints,
        discretization_timestep: PIBT_DT, 
    });
    stats.post_times.push(post_start.elapsed());

    for (i, obs) in agents.iter_mut().enumerate() {
        let poses = obs.path.iter().map(|p| Isometry2::new(*p, 0.0)).collect();
        obs.follower = Some(WaypointFollower::from_trajectory(i, Trajectory { poses }));
    }
    
    stats.total_replan_times.push(total_start.elapsed());
    plan
}

fn run_simulation(size: usize, num_uncontrolled: usize, num_controlled: usize, seed: u64, stats: &mut LatencyStats) {
    let (mut agents, grid) = generate_scenario(size, num_uncontrolled, num_controlled, seed);
    if agents.len() < (num_uncontrolled + num_controlled) { return; }

    let mut post_trajectories = vec![];
    let mut footprints = vec![];
    for obs in &agents {
        let poses = obs.path.iter().map(|p| Isometry2::new(*p, 0.0)).collect();
        post_trajectories.push(Trajectory { poses });
        footprints.push(Arc::new(Ball::new(0.45)) as Arc<dyn mapf_post::shape::Shape>);
    }

    let mut plan = mapf_post(&MapfResult {
        trajectories: post_trajectories,
        footprints,
        discretization_timestep: PIBT_DT,
    });

    for (i, obs) in agents.iter_mut().enumerate() {
        let poses = obs.path.iter().map(|p| Isometry2::new(*p, 0.0)).collect();
        obs.follower = Some(WaypointFollower::from_trajectory(i, Trajectory { poses }));
        obs.current_pos = obs.path[0];
    }

    let mut sim_time = 0.0;
    while sim_time < MAX_SIM_TIME {
        let current_waypoints: Vec<_> = agents.iter_mut()
            .filter_map(|obs| obs.follower.as_mut().map(|f| f.get_semantic_waypoint()))
            .collect();
        
        let claim_dict = plan.get_claim_dict(&current_waypoints);
        for obs in &mut agents { obs.update(DT, &claim_dict, seed, 0.8); }

        // We don't check collision here, we just want to measure replan latency
        if check_for_detailed_violation_mixed(&plan, &current_waypoints) {
            plan = replan(&mut agents, &grid, 2, seed, sim_time, stats);
        }

        let mut all_reached = true;
        for obs in &agents {
            if !obs.uncontrolled {
                if (obs.current_pos - obs.goal).norm() > 0.1 { all_reached = false; break; }
            }
        }
        if all_reached { break; }
        sim_time += DT;
    }
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let num_scenes = args.get(1).and_then(|s| s.parse::<usize>().ok()).unwrap_or(10);
    
    let configs = [(16, 8, 4), (16, 16, 8), (32, 32, 16)];
    
    println!("Phase 5: Latency Profiling ({} scenes per config)", num_scenes);
    println!("{:<12} | {:<10} | {:<10} | {:<10} | {:<10}", 
             "Config (U/C)", "PIBT Avg", "POST Avg", "Total Avg", "99th Pct");
    println!("{:-<12}-|-{:-<10}-|-{:-<10}-|-{:-<10}-|-{:-<10}", 
             "", "", "", "", "");

    for (size, num_unctrl, num_ctrl) in configs {
        let mut stats = LatencyStats::default();

        for scene_seed in 0..num_scenes {
            run_simulation(size, num_unctrl, num_ctrl, scene_seed as u64, &mut stats);
        }

        if stats.total_replan_times.is_empty() {
            println!("{:<12} | No replans occurred.", format!("{}/{}", num_unctrl, num_ctrl));
            continue;
        }

        let avg_pibt = stats.pibt_times.iter().sum::<Duration>() / stats.pibt_times.len() as u32;
        let avg_post = stats.post_times.iter().sum::<Duration>() / stats.post_times.len() as u32;
        let avg_total = stats.total_replan_times.iter().sum::<Duration>() / stats.total_replan_times.len() as u32;
        
        let mut all_totals = stats.total_replan_times.clone();
        all_totals.sort();
        let p99 = all_totals[(all_totals.len() as f32 * 0.99) as usize];

        println!("{:<12} | {:<10.2?} | {:<10.2?} | {:<10.2?} | {:<10.2?}", 
                 format!("{}/{}", num_unctrl, num_ctrl), avg_pibt, avg_post, avg_total, p99);
    }
}
