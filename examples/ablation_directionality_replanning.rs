use hetpibt::external_tracks_pibt::PiBTWithExternalTracks;
use std::collections::HashMap;
use std::sync::Arc;
use rand::{SeedableRng, Rng, rngs::StdRng};
use mapf_post::{
    mapf_post, MapfResult, Trajectory, WaypointFollower,
    na::{Isometry2, Vector2},
    shape::Ball,
};

const OBSTACLE_SPEED: f32 = 4.0; // Cells per second
const DT: f32 = 0.1;
const MAX_SIM_TIME: f32 = 100.0;
const PIBT_DT: f32 = 1.0 / OBSTACLE_SPEED; // 0.25s per PIBT step

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
    fn new(path: Vec<Vector2<f32>>, agent_id: usize) -> Self {
        let start_pos = path[0];
        let goal = *path.last().unwrap();
        Self {
            path: path.clone(),
            follower: None,
            agent_id,
            current_pos: start_pos,
            goal,
            uncontrolled: true,
            stall_timer: 0.0,
            original_path: path,
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
                    while rng.random_range(0..100) < 80 {
                        extra_stall += 0.3;
                    }
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

fn get_default_agents(lookahead: usize, seed: u64) -> (Vec<MovingObstacle>, Vec<Vec<bool>>) {
    let grid = vec![vec![false; 8]; 8];
    let mut agents = vec![];

    // External 0
    let mut track0 = vec![];
    for x in 0..8 { track0.push(Vector2::new(x as f32, 4.0)); }
    agents.push(MovingObstacle::new(track0, agents.len()));

    // External 1
    let mut track1 = vec![];
    for y in 0..8 { track1.push(Vector2::new(4.0, y as f32)); }
    agents.push(MovingObstacle::new(track1, agents.len()));

    // External 2
    let mut track2 = vec![];
    for x in (0..8).rev() { track2.push(Vector2::new(x as f32, 2.0)); }
    track2.push(Vector2::new(0.0, 1.0));
    track2.push(Vector2::new(0.0, 0.0));
    agents.push(MovingObstacle::new(track2, agents.len()));

    let external_tracks: Vec<Vec<(usize, usize)>> = agents.iter()
        .filter(|a| a.uncontrolled)
        .map(|a| a.path.iter().map(|p| (p.y.round() as usize, p.x.round() as usize)).collect())
        .collect();

    let mut pibt_grid = vec![vec![0; 8]; 8];
    let mut solver = PiBTWithExternalTracks::init(pibt_grid);
    solver.lookahead = lookahead;
    solver.enforce_directionality = true;
    solver.set_seed(seed);

    let starts = vec![(0, 0), (0, 7)];
    let ends = vec![(7, 7), (7, 0)];

    let result = solver.solve(&starts, &ends, &external_tracks, 60);

    if let Ok(trajectories) = result {
        if !trajectories.is_empty() {
            let num_internal_agents = trajectories[0].len();
            for i in 0..num_internal_agents {
                let mut path = vec![];
                for t in 0..trajectories.len() {
                    let pos = trajectories[t][i];
                    path.push(Vector2::new(pos.1 as f32, pos.0 as f32));
                }
                let mut obs = MovingObstacle::new(path, agents.len());
                obs.uncontrolled = false;
                agents.push(obs);
            }
        }
    }

    (agents, grid)
}

fn check_collision(agents: &[MovingObstacle]) -> bool {
    for i in 0..agents.len() {
        for j in i + 1..agents.len() {
            let a = &agents[i];
            let b = &agents[j];
            if a.uncontrolled == b.uncontrolled { continue; }
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
) -> mapf_post::SemanticPlan {
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
            // Start from where they are on their path
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

    let result = solver.solve(&starts, &ends, &external_tracks, 60);

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

    // For mapf_post, the "trajectory" is sampled at PIBT_DT (0.25s)
    let mut post_trajectories = vec![];
    let mut footprints = vec![];
    for obs in agents.iter_mut() {
        if obs.uncontrolled {
            let current_wp_idx = obs.follower.as_mut().map(|f| f.get_semantic_waypoint().trajectory_index).unwrap_or(0);
            let mut remaining = vec![];
            remaining.push(obs.current_pos); // Start from ACTUAL position
            for j in (current_wp_idx + 1)..obs.original_path.len() {
                remaining.push(obs.original_path[j]);
            }
            if remaining.len() < 2 {
                let last = *obs.original_path.last().unwrap();
                remaining = vec![last, last];
            }
            obs.path = remaining;
        } else {
            // Internal agents: ensure path starts from actual current_pos
            if !obs.path.is_empty() {
                obs.path[0] = obs.current_pos;
            }
        }
        
        let poses = obs.path.iter().map(|p| Isometry2::new(*p, 0.0)).collect();
        post_trajectories.push(Trajectory { poses });
        footprints.push(Arc::new(Ball::new(0.45)) as Arc<dyn mapf_post::shape::Shape>);
    }

    let plan = mapf_post(&MapfResult {
        trajectories: post_trajectories,
        footprints,
        discretization_timestep: PIBT_DT, // Input poses are 0.25s apart
    });

    for (i, obs) in agents.iter_mut().enumerate() {
        let poses = obs.path.iter().map(|p| Isometry2::new(*p, 0.0)).collect();
        obs.follower = Some(WaypointFollower::from_trajectory(i, Trajectory { poses }));
    }

    plan
}

#[derive(Default)]
struct SimulationResult {
    collision_time: Option<f32>,
    violation_time: Option<f32>,
    num_replans: usize,
}

fn run_simulation(lookahead: usize, scene_seed: u64) -> SimulationResult {
    let (mut agents, grid) = get_default_agents(lookahead, scene_seed);
    if agents.is_empty() { return SimulationResult::default(); }

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

        for obs in &mut agents {
            obs.update(DT, &claim_dict, scene_seed);
        }

        if result.collision_time.is_none() && check_collision(&agents) {
            result.collision_time = Some(sim_time);
        }

        if check_for_detailed_violation_mixed(&plan, &current_waypoints) {
            if result.violation_time.is_none() { result.violation_time = Some(sim_time); }
            plan = replan(&mut agents, &grid, lookahead, scene_seed, sim_time);
            result.num_replans += 1;
            if result.num_replans > 1000 { break; }
        }

        let mut all_reached = true;
        for obs in &agents {
            if !obs.uncontrolled {
                if (obs.current_pos - obs.goal).norm() > 0.05 { all_reached = false; break; }
            }
        }
        if all_reached { return result; }
        sim_time += DT;
    }
    result
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let num_scenes = args.get(1).and_then(|s| s.parse::<usize>().ok()).unwrap_or(100);
    let lookaheads = if args.len() > 2 {
        args[2..].iter().filter_map(|s| s.parse::<usize>().ok()).collect()
    } else {
        vec![0, 1, 2, 3, 4]
    };

    println!("Running ablation WITH DIRECTIONALITY + REPLANNING ({} scenes) lookaheads: {:?}", num_scenes, lookaheads);
    println!("{:<10} | {:<12} | {:<12} | {:<15} | {:<15} | {:<10}", 
             "Lookahead", "Coll. %", "Viol. %", "Mean Coll. T", "Mean Viol. T", "Avg Replans");
    println!("{:-<10}-|-{:-<12}-|-{:-<12}-|-{:-<15}-|-{:-<15}-|-{:-<10}", 
             "", "", "", "", "", "");

    for &lookahead in &lookaheads {
        let mut collisions = 0;
        let mut violations = 0;
        let mut total_collision_time = 0.0;
        let mut total_violation_time = 0.0;
        let mut total_replans = 0;

        for scene_seed in 0..num_scenes {
            let res = run_simulation(lookahead, scene_seed as u64);
            if let Some(t) = res.collision_time { collisions += 1; total_collision_time += t; }
            if let Some(t) = res.violation_time { violations += 1; total_violation_time += t; }
            total_replans += res.num_replans;
        }

        let coll_perc = (collisions as f32 / num_scenes as f32) * 100.0;
        let viol_perc = (violations as f32 / num_scenes as f32) * 100.0;
        let mean_coll_t = if collisions > 0 { total_collision_time / collisions as f32 } else { 0.0 };
        let mean_viol_t = if violations > 0 { total_violation_time / violations as f32 } else { 0.0 };
        let avg_replans = total_replans as f32 / num_scenes as f32;

        println!("{:<10} | {:<11.1}% | {:<11.1}% | {:<15.2} | {:<15.2} | {:<10.2}", 
                 lookahead, coll_perc, viol_perc, mean_coll_t, mean_viol_t, avg_replans);
    }
}
