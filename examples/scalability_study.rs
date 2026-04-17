use hetpibt::external_tracks_pibt::PiBTWithExternalTracks;
use std::collections::HashMap;
use std::sync::Arc;
use rand::{SeedableRng, Rng, rngs::StdRng, seq::SliceRandom};
use mapf_post::{
    mapf_post, MapfResult, Trajectory, WaypointFollower, SemanticPlan, SemanticWaypoint,
    na::{Isometry2, Vector2},
    shape::Ball,
};

const DT: f32 = 0.25;
const MAX_SIM_TIME: f32 = 120.0;
const PIBT_DT: f32 = 0.25;

impl Clone for MovingObstacle {
    fn clone(&self) -> Self {
        Self {
            path: self.path.clone(),
            original_path: self.original_path.clone(),
            follower: None, // Follower is re-initialized after scenario generation
            agent_id: self.agent_id,
            current_pos: self.current_pos,
            uncontrolled: self.uncontrolled,
            stall_timer: self.stall_timer,
            goal: self.goal,
        }
    }
}

struct MovingObstacle {
    path: Vec<Vector2<f32>>,
    original_path: Vec<Vector2<f32>>, // Keep original track for uncontrolled
    follower: Option<WaypointFollower>,
    agent_id: usize,
    current_pos: Vector2<f32>,
    uncontrolled: bool,
    stall_timer: f32,
    goal: Vector2<f32>,
}

impl MovingObstacle {
    fn new(path: Vec<Vector2<f32>>, goal: Vector2<f32>, agent_id: usize, uncontrolled: bool) -> Self {
        let start_pos = path[0];
        Self {
            path: path.clone(),
            original_path: path,
            follower: None,
            agent_id,
            current_pos: start_pos,
            uncontrolled,
            stall_timer: 0.0,
            goal,
        }
    }

    fn update(&mut self, dt: f32, claim_dict: &HashMap<usize, mapf_post::CurrentlyAllocatedTrajSegment>, seed: u64) {
        if self.path.is_empty() {
            return;
        }

        if self.stall_timer > 0.0 {
            self.stall_timer -= dt;
            return;
        }

        if let Some(follower) = &mut self.follower {
            let can_move = if self.uncontrolled {
                true
            } else if let Some(segment) = claim_dict.get(&self.agent_id) {
                follower.get_semantic_waypoint().trajectory_index < segment.end_id
            } else {
                false
            };

            if can_move {
                if self.uncontrolled {
                    let wp_idx = follower.get_semantic_waypoint().trajectory_index;
                    let mut rng = StdRng::seed_from_u64(seed + self.agent_id as u64 * 1000 + wp_idx as u64);

                    if rng.random_range(0..100) < 20 { 
                        self.stall_timer = DT; // Stall for one step
                        return; // Do NOT advance
                    }
                }

                let next_wp = follower.next_waypoint();
                self.current_pos = Vector2::new(next_wp.translation.x, next_wp.translation.y);
                
                let isometry = Isometry2::new(self.current_pos, 0.0);
                follower.update_position_estimate(&isometry, dt);
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
) -> Option<Vec<MovingObstacle>> {
    let mut rng = StdRng::seed_from_u64(seed);
    let mut agents = vec![];
    let mut forbidden_for_starts = std::collections::HashSet::new();
    let mut forbidden_for_ends = std::collections::HashSet::new();

    // Generate uncontrolled agents
    for _ in 0..num_uncontrolled {
        let is_horizontal = rng.random_bool(0.5);
        let mut track = vec![];
        
        let mut attempts = 0;
        loop {
            track.clear();
            if is_horizontal {
                let y = rng.random_range(0..size);
                let reverse = rng.random_bool(0.5);
                if reverse { for x in (0..size).rev() { track.push(Vector2::new(x as f32 + 0.5, y as f32 + 0.5)); } }
                else { for x in 0..size { track.push(Vector2::new(x as f32 + 0.5, y as f32 + 0.5)); } }
            } else {
                let x = rng.random_range(0..size);
                let reverse = rng.random_bool(0.5);
                if reverse { for y in (0..size).rev() { track.push(Vector2::new(x as f32 + 0.5, y as f32 + 0.5)); } }
                else { for y in 0..size { track.push(Vector2::new(x as f32 + 0.5, y as f32 + 0.5)); } }
            }
            
            let start = (track[0].y as usize, track[0].x as usize);
            let end = (track.last().unwrap().y as usize, track.last().unwrap().x as usize);
            
            if !forbidden_for_starts.contains(&start) && !forbidden_for_ends.contains(&end) {
                break;
            }
            attempts += 1;
            if attempts > 20 { break; }
        }
        
        forbidden_for_starts.insert((track[0].y as usize, track[0].x as usize));
        forbidden_for_ends.insert((track.last().unwrap().y as usize, track.last().unwrap().x as usize));
        
        let goal = *track.last().unwrap();
        agents.push(MovingObstacle::new(track, goal, agents.len(), true));
    }

    let mut start_candidates: Vec<(usize, usize)> = vec![];
    let mut end_candidates: Vec<(usize, usize)> = vec![];
    for r in 0..size {
        for c in 0..size {
            if !forbidden_for_starts.contains(&(r, c)) { start_candidates.push((r, c)); }
            if !forbidden_for_ends.contains(&(r, c)) { end_candidates.push((r, c)); }
        }
    }
    
    if start_candidates.len() < num_controlled || end_candidates.len() < num_controlled {
        return None;
    }

    for i in 0..10 { // Retry 10 times with different goals
        let mut starts = vec![];
        let mut ends = vec![];
        let mut used_starts = std::collections::HashSet::new();
        let mut used_ends = std::collections::HashSet::new();
        
        start_candidates.shuffle(&mut rng);
        end_candidates.shuffle(&mut rng);

        let mut si = 0;
        let mut ei = 0;
        let mut possible = true;
        for _ in 0..num_controlled {
            while si < start_candidates.len() && used_starts.contains(&start_candidates[si]) { si += 1; }
            if si >= start_candidates.len() { possible = false; break; }
            let s = start_candidates[si];
            used_starts.insert(s);
            starts.push(s);

            while ei < end_candidates.len() && (used_ends.contains(&end_candidates[ei]) || end_candidates[ei] == s) { ei += 1; }
            if ei >= end_candidates.len() { possible = false; break; }
            let e = end_candidates[ei];
            used_ends.insert(e);
            ends.push(e);
        }

        if !possible { continue; }

        let pibt_grid = vec![vec![0; size]; size];
        let mut solver = PiBTWithExternalTracks::init(pibt_grid);
        solver.lookahead = 5;
        solver.set_seed(seed + i as u64 * 100);
        solver.enforce_directionality = enforce_directionality;

        let external_tracks: Vec<Vec<(usize, usize)>> = agents.iter()
            .map(|a| a.path.iter().map(|p| (p.y as usize, p.x as usize)).collect())
            .collect();

        match solver.solve(&starts, &ends, &external_tracks, 5000) {
            Ok(trajectories) if !trajectories.is_empty() && trajectories[0].len() == num_controlled => {
                let mut new_agents = agents.clone();
                let num_internal = trajectories[0].len();
                for i in 0..num_internal {
                    let mut path = vec![];
                    for t in 0..trajectories.len() {
                        let pos = trajectories[t][i];
                        path.push(Vector2::new(pos.1 as f32 + 0.5, pos.0 as f32 + 0.5));
                    }
                    let goal = Vector2::new(ends[i].1 as f32 + 0.5, ends[i].0 as f32 + 0.5);
                    new_agents.push(MovingObstacle::new(path, goal, new_agents.len(), false));
                }
                return Some(new_agents);
            }
            _ => {}
        }
    }
    None
}

fn replan(
    agents: &mut [MovingObstacle],
    grid: &[Vec<usize>],
    lookahead: usize,
    seed: u64,
    sim_time: f32,
    enforce_directionality: bool,
) -> SemanticPlan {
    let mut starts = vec![];
    let mut ends = vec![];
    let mut internal_agent_ids = vec![];

    for obs in agents.iter() {
        if !obs.uncontrolled {
            starts.push((obs.current_pos.y as usize, obs.current_pos.x as usize));
            ends.push((obs.goal.y as usize, obs.goal.x as usize));
            internal_agent_ids.push(obs.agent_id);
        }
    }

    let mut external_tracks = vec![];
    for obs in agents.iter_mut() {
        if obs.uncontrolled {
            let current_wp_idx = obs.follower.as_mut().map(|f| f.get_semantic_waypoint().trajectory_index).unwrap_or(0);
            let mut track = vec![];
            for j in current_wp_idx..obs.original_path.len() {
                let p = obs.original_path[j];
                track.push((p.y as usize, p.x as usize));
            }
            if track.is_empty() {
                let p = obs.original_path.last().unwrap();
                track.push((p.y as usize, p.x as usize));
            }
            external_tracks.push(track);
        }
    }

    let mut solver = PiBTWithExternalTracks::init(grid.to_vec());
    solver.lookahead = lookahead;
    solver.set_seed(seed + (sim_time * 1000.0) as u64);
    solver.enforce_directionality = enforce_directionality;

    if let Ok(pibt_trajectories) = solver.solve(&starts, &ends, &external_tracks, 5000) {
        if !pibt_trajectories.is_empty() {
            for (i, &agent_id) in internal_agent_ids.iter().enumerate() {
                let mut new_path = vec![];
                for t in 0..pibt_trajectories.len() {
                    let pos = pibt_trajectories[t][i];
                    new_path.push(Vector2::new(pos.1 as f32 + 0.5, pos.0 as f32 + 0.5));
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
        } else if !obs.path.is_empty() {
            obs.path[0] = obs.current_pos;
        }
        
        let poses = obs.path.iter().map(|p| Isometry2::new(*p, 0.0)).collect();
        post_trajectories.push(Trajectory { poses });
        footprints.push(Arc::new(Ball::new(0.4)) as Arc<dyn mapf_post::shape::Shape>);
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

fn check_collision(agents: &[MovingObstacle]) -> bool {
    for i in 0..agents.len() {
        for j in i + 1..agents.len() {
            let a = &agents[i];
            let b = &agents[j];
            if a.uncontrolled == b.uncontrolled { continue; }
            let dist = (a.current_pos - b.current_pos).norm();
            if dist < 0.8 { return true; }
        }
    }
    false
}

fn check_for_violation(plan: &SemanticPlan, current_pos: &[SemanticWaypoint]) -> bool {
    let mut agent_to_pos = HashMap::new();
    for &wp in current_pos {
        agent_to_pos.insert(wp.agent, wp);
    }
    for wp in current_pos {
        if let Some(deps) = plan.comes_before(wp) {
            for &dep_idx in deps {
                let dep_wp = plan.waypoints[dep_idx];
                if dep_wp.agent == wp.agent { continue; }
                if let Some(other_agent) = agent_to_pos.get(&dep_wp.agent) {
                    if other_agent.trajectory_index < dep_wp.trajectory_index { 
                        return true; 
                    }
                }
            }
        }
    }
    false
}

#[derive(Default)]
struct SimulationResult {
    collision: bool,
    violation: bool,
    success: bool,
    num_replans: usize,
    skipped: bool,
    pibt_failed: bool,
}

fn run_simulation(size: usize, num_uncontrolled: usize, num_controlled: usize, seed: u64, enforce_directionality: bool) -> SimulationResult {
    let Some(mut agents) = generate_scenario(size, num_uncontrolled, num_controlled, seed, enforce_directionality) else {
        return SimulationResult { skipped: true, ..Default::default() };
    };

    // Sanity Check: Verify the raw PIBT plan succeeds in deterministic execution
    for a in &agents {
        if !a.uncontrolled {
            if (a.path.last().unwrap() - a.goal).norm() > 0.1 {
                return SimulationResult { pibt_failed: true, ..Default::default() };
            }
        }
    }

    let grid = vec![vec![0; size]; size];

    let mut trajectories = vec![];
    let mut footprints = vec![];
    for a in &agents {
        let poses = a.path.iter().map(|p| Isometry2::new(*p, 0.0)).collect();
        trajectories.push(Trajectory { poses });
        footprints.push(Arc::new(Ball::new(0.4)) as Arc<dyn mapf_post::shape::Shape>);
    }

    let mut plan = mapf_post(&MapfResult { trajectories, footprints, discretization_timestep: PIBT_DT });

    for (i, a) in agents.iter_mut().enumerate() {
        let poses = a.path.iter().map(|p| Isometry2::new(*p, 0.0)).collect();
        a.follower = Some(WaypointFollower::from_trajectory(i, Trajectory { poses }));
        a.current_pos = a.path[0];
    }

    let mut sim_time = 0.0;
    let mut result = SimulationResult::default();

    while sim_time < MAX_SIM_TIME {
        let mut current_waypoints = vec![];
        for a in &mut agents {
            if let Some(f) = a.follower.as_mut() { current_waypoints.push(f.get_semantic_waypoint()); }
        }

        if check_for_violation(&plan, &current_waypoints) {
            result.violation = true;
            plan = replan(&mut agents, &grid, 5, seed, sim_time, enforce_directionality);
            result.num_replans += 1;
            if result.num_replans > 200 { break; }
            
            current_waypoints.clear();
            for a in &mut agents {
                if let Some(f) = a.follower.as_mut() { current_waypoints.push(f.get_semantic_waypoint()); }
            }
        }

        let claim_dict = plan.get_claim_dict(&current_waypoints);
        for a in &mut agents { a.update(DT, &claim_dict, seed); }

        if check_collision(&agents) {
            result.collision = true;
            return result;
        }

        let mut all_reached = true;
        for a in &agents {
            if !a.uncontrolled && (a.current_pos - a.goal).norm() > 0.5 {
                all_reached = false;
                break;
            }
        }
        if all_reached {
            result.success = true;
            return result;
        }
        sim_time += DT;
    }
    result
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let num_scenes = args.get(1).and_then(|s| s.parse::<usize>().ok()).unwrap_or(10);

    println!("Scalability Study: Directionality vs No Directionality ({} scenes)", num_scenes);
    println!("{:<25} | {:<5} | {:<10} | {:<10} | {:<10} | {:<12} | {:<8} | {:<10}", 
             "Config", "Dir", "Success%", "Coll.%", "Viol.%", "Avg Replans", "Skipped", "PIBT Fail");
    println!("{:-<25}-|-{:-<5}-|-{:-<10}-|-{:-<10}-|-{:-<10}-|-{:-<12}-|-{:-<8}-|-{:-<10}", "", "", "", "", "", "", "", "");

    let configs = vec![
        (8, 0, 4),   // 6% density
        (8, 2, 4),   // 9% density
        (16, 0, 8),  // 3% density
        (16, 12, 12), // 9% density
        (16, 24, 24), // 18% density
        (32, 0, 16),  // 1.5% density
        (32, 50, 50), // 10% density
        (32, 100, 100), // 20% density
    ];

    for (size, u, c) in configs {
        let label = format!("{}x{}, {}U, {}C", size, size, u, c);

        for dir in [true, false] {
            let mut success = 0;
            let mut coll = 0;
            let mut viol = 0;
            let mut replans = 0;
            let mut skipped = 0;
            let mut pibt_fail = 0;
            for s in 0..num_scenes {
                let res = run_simulation(size, u, c, s as u64, dir);
                if res.skipped { skipped += 1; continue; }
                if res.pibt_failed { pibt_fail += 1; continue; }
                if res.success { success += 1; }
                if res.collision { coll += 1; }
                if res.violation { viol += 1; }
                replans += res.num_replans;
            }
            let n = (num_scenes - skipped - pibt_fail) as f32;
            if n > 0.0 {
                println!("{:<25} | {:<5} | {:<9.1}% | {:<9.1}% | {:<9.1}% | {:<12.2} | {:<8} | {:<10}", 
                         label, if dir { "Y" } else { "N" }, (success as f32 / n) * 100.0, (coll as f32 / n) * 100.0, (viol as f32 / n) * 100.0, replans as f32 / n, skipped, pibt_fail);
            } else {
                println!("{:<25} | {:<5} | {:<10} | {:<10} | {:<10} | {:<12} | {:<8} | {:<10}", 
                         label, if dir { "Y" } else { "N" }, "N/A", "N/A", "N/A", "N/A", skipped, pibt_fail);
            }
        }
        println!("{:-<25}-|-{:-<5}-|-{:-<10}-|-{:-<10}-|-{:-<10}-|-{:-<12}-|-{:-<8}-|-{:-<10}", "", "", "", "", "", "", "", "");
    }
}
