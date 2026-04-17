use hetpibt::external_tracks_pibt::PiBTWithExternalTracks;
use std::collections::HashMap;
use std::sync::Arc;
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
    seed: u64,
    enforce_directionality: bool,
) -> (Vec<MovingObstacle>, Vec<Vec<bool>>) {
    let mut rng = StdRng::seed_from_u64(seed);
    let grid = vec![vec![false; size]; size];
    let mut agents = vec![];

    let mut occupied_starts = std::collections::HashSet::new();
    let mut occupied_goals = std::collections::HashSet::new();
    let mut occupied_lanes_h = std::collections::HashSet::new();
    let mut occupied_lanes_v = std::collections::HashSet::new();

    // Generate Uncontrolled Agents
    for _ in 0..num_uncontrolled {
        for _ in 0..100 { 
            let is_horizontal = rng.random_bool(0.5);
            let mut track = vec![];
            if is_horizontal {
                let y = rng.random_range(0..size);
                if occupied_lanes_h.contains(&y) { continue; }
                let reverse = rng.random_bool(0.5);
                if reverse { for x in (0..size).rev() { track.push(Vector2::new(x as f32, y as f32)); } }
                else { for x in 0..size { track.push(Vector2::new(x as f32, y as f32)); } }
                occupied_lanes_h.insert(y);
            } else {
                let x = rng.random_range(0..size);
                if occupied_lanes_v.contains(&x) { continue; }
                let reverse = rng.random_bool(0.5);
                if reverse { for y in (0..size).rev() { track.push(Vector2::new(x as f32, y as f32)); } }
                else { for y in 0..size { track.push(Vector2::new(x as f32, y as f32)); } }
                occupied_lanes_v.insert(x);
            }
            
            let start = (track[0].y.round() as usize, track[0].x.round() as usize);
            let goal = (track.last().unwrap().y.round() as usize, track.last().unwrap().x.round() as usize);
            occupied_starts.insert(start);
            occupied_goals.insert(goal);
            agents.push(MovingObstacle::new(track, agents.len(), true));
            break;
        }
    }

    // Generate Controlled Agents
    let mut starts = vec![];
    let mut ends = vec![];
    let mut possible_cells: Vec<(usize, usize)> = vec![];
    for r in 0..size { for c in 0..size { possible_cells.push((r, c)); } }

    for _ in 0..num_controlled {
        possible_cells.shuffle(&mut rng);
        let mut found = false;
        for i in 0..possible_cells.len() {
            let s = possible_cells[i];
            if !occupied_starts.contains(&s) && !occupied_goals.contains(&s) {
                for j in 0..possible_cells.len() {
                    let e = possible_cells[j];
                    if s != e && !occupied_starts.contains(&e) && !occupied_goals.contains(&e) {
                        starts.push(s);
                        ends.push(e);
                        occupied_starts.insert(s);
                        occupied_goals.insert(e);
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
    solver.lookahead = 5; 
    solver.enforce_directionality = enforce_directionality;
    solver.set_seed(seed);

    let result = solver.solve(&starts, &ends, &external_tracks, 200);

    match result {
        Ok(trajectories) if !trajectories.is_empty() => {
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
        _ => {}
    }

    (agents, grid)
}

fn check_collision(agents: &[MovingObstacle]) -> bool {
    for i in 0..agents.len() {
        for j in i + 1..agents.len() {
            let a = &agents[i];
            let b = &agents[j];
            if a.uncontrolled && b.uncontrolled { continue; }
            let dist = (a.current_pos - b.current_pos).norm();
            if dist < 0.9 { return true; }
        }
    }
    false
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
    enforce_directionality: bool,
) -> mapf_post::SemanticPlan {
    let mut pibt_grid = vec![vec![0; grid[0].len()]; grid.len()];
    for r in 0..grid.len() {
        for c in 0..grid[0].len() {
            if grid[r][c] { pibt_grid[r][c] = 1; }
        }
    }

    let mut solver = PiBTWithExternalTracks::init(pibt_grid);
    solver.lookahead = lookahead;
    solver.enforce_directionality = enforce_directionality;
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

    let result = solver.solve(&starts, &ends, &external_tracks, 200);

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

    let plan = mapf_post(&MapfResult {
        trajectories: post_trajectories,
        footprints,
        discretization_timestep: PIBT_DT, 
    });

    for (i, obs) in agents.iter_mut().enumerate() {
        let poses = obs.path.iter().map(|p| Isometry2::new(*p, 0.0)).collect();
        obs.follower = Some(WaypointFollower::from_trajectory(i, Trajectory { poses }));
    }
    plan
}

#[derive(Default)]
struct SimulationResult {
    collision: bool,
    success: bool,
    num_replans: usize,
    agents_reached: usize,
}

fn run_simulation(size: usize, num_uncontrolled: usize, num_controlled: usize, seed: u64, stall_prob: f32, enforce_directionality: bool) -> SimulationResult {
    let (mut agents, grid) = generate_scenario(size, num_uncontrolled, num_controlled, seed, enforce_directionality);
    if agents.len() < (num_uncontrolled + num_controlled) { 
        return SimulationResult { collision: true, success: false, num_replans: 0, agents_reached: 0 }; 
    }

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
    let mut result = SimulationResult::default();

    while sim_time < MAX_SIM_TIME {
        let current_waypoints: Vec<_> = agents.iter_mut()
            .filter_map(|obs| obs.follower.as_mut().map(|f| f.get_semantic_waypoint()))
            .collect();
        
        let claim_dict = plan.get_claim_dict(&current_waypoints);
        for obs in &mut agents { obs.update(DT, &claim_dict, seed, stall_prob); }

        if check_collision(&agents) {
            result.collision = true;
            let mut reached = 0;
            for obs in &agents {
                if !obs.uncontrolled && (obs.current_pos - obs.goal).norm() <= 0.1 {
                    reached += 1;
                }
            }
            result.agents_reached = reached;
            return result;
        }

        if check_for_detailed_violation_mixed(&plan, &current_waypoints) {
            plan = replan(&mut agents, &grid, 5, seed, sim_time, enforce_directionality);
            result.num_replans += 1;
            if result.num_replans > 2000 { break; }
        }

        let mut reached_count = 0;
        for obs in &agents {
            if !obs.uncontrolled {
                if (obs.current_pos - obs.goal).norm() <= 0.1 { 
                    reached_count += 1;
                }
            }
        }
        result.agents_reached = reached_count;

        if reached_count == num_controlled {
            result.success = true;
            return result;
        }
        sim_time += DT;
    }
    result
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let num_scenes = args.get(1).and_then(|s| s.parse::<usize>().ok()).unwrap_or(20);
    
    let stall_probs = [0.0, 0.2, 0.4, 0.6, 0.8, 0.9, 0.95];
    let map_size = 16;
    let num_unctrl = 12;
    let num_ctrl = 5;

    println!("Phase 2: Stochasticity Sensitivity (Comparison: Baseline vs Hybrid, {} scenes per config)", num_scenes);
    println!("{:<10} | {:<12} | {:<12} | {:<12} | {:<12}", 
             "Stall Prob", "Base Succ%", "Hyb Succ%", "Base Agnt%", "Hyb Agnt%");
    println!("{:-<10}-|-{:-<12}-|-{:-<12}-|-{:-<12}-|-{:-<12}", 
             "", "", "", "", "");

    for &p in &stall_probs {
        let mut base_successes = 0;
        let mut hyb_successes = 0;
        let mut base_agents_reached = 0;
        let mut hyb_agents_reached = 0;

        for scene_seed in 0..num_scenes {
            // Baseline
            let res_base = run_simulation(map_size, num_unctrl, num_ctrl, scene_seed as u64, p, false);
            if res_base.success { base_successes += 1; }
            base_agents_reached += res_base.agents_reached;

            // Hybrid
            let res_hyb = run_simulation(map_size, num_unctrl, num_ctrl, scene_seed as u64, p, true);
            if res_hyb.success { hyb_successes += 1; }
            hyb_agents_reached += res_hyb.agents_reached;
        }

        let base_succ_perc = (base_successes as f32 / num_scenes as f32) * 100.0;
        let hyb_succ_perc = (hyb_successes as f32 / num_scenes as f32) * 100.0;
        let base_agent_perc = (base_agents_reached as f32 / (num_scenes * num_ctrl) as f32) * 100.0;
        let hyb_agent_perc = (hyb_agents_reached as f32 / (num_scenes * num_ctrl) as f32) * 100.0;

        println!("{:<10.2} | {:<11.1}% | {:<11.1}% | {:<11.1}% | {:<11.1}%", 
                 p, base_succ_perc, hyb_succ_perc, base_agent_perc, hyb_agent_perc);
    }
}