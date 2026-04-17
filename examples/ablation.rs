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

struct MovingObstacle {
    path: Vec<Vector2<f32>>,
    follower: Option<WaypointFollower>,
    agent_id: usize,
    current_pos: Vector2<f32>,
    uncontrolled: bool,
    stall_timer: f32,
}

impl MovingObstacle {
    fn new(path: Vec<Vector2<f32>>, agent_id: usize) -> Self {
        let start_pos = path[0];
        Self {
            path,
            follower: None,
            agent_id,
            current_pos: start_pos,
            uncontrolled: true,
            stall_timer: 0.0,
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

fn get_default_agents(lookahead: usize, seed: u64) -> Vec<MovingObstacle> {
    let grid = vec![vec![false; 8]; 8];
    let mut agents = vec![];

    // External 0
    let mut track0 = vec![];
    for x in 0..8 {
        track0.push(Vector2::new(x as f32, 4.0));
    }
    let mut obs0 = MovingObstacle::new(track0, agents.len());
    obs0.uncontrolled = true;
    agents.push(obs0);

    // External 1
    let mut track1 = vec![];
    for y in 0..8 {
        track1.push(Vector2::new(4.0, y as f32));
    }
    let mut obs1 = MovingObstacle::new(track1, agents.len());
    obs1.uncontrolled = true;
    agents.push(obs1);

    // External 2
    let mut track2 = vec![];
    for x in (0..8).rev() {
        track2.push(Vector2::new(x as f32, 2.0));
    }
    track2.push(Vector2::new(0.0, 1.0));
    track2.push(Vector2::new(0.0, 0.0));
    let mut obs2 = MovingObstacle::new(track2, agents.len());
    obs2.uncontrolled = true;
    agents.push(obs2);

    let external_tracks: Vec<Vec<(usize, usize)>> = agents.iter()
        .filter(|a| a.uncontrolled)
        .map(|a| a.path.iter().map(|p| (p.y.round() as usize, p.x.round() as usize)).collect())
        .collect();

    let rows = grid.len();
    let cols = grid[0].len();
    let mut pibt_grid = vec![vec![0; cols]; rows];
    for r in 0..rows {
        for c in 0..cols {
            if grid[r][c] {
                pibt_grid[r][c] = 1;
            }
        }
    }

    let mut solver = PiBTWithExternalTracks::init(pibt_grid);
    solver.lookahead = lookahead;
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

    agents
}

fn check_collision(agents: &[MovingObstacle]) -> bool {
    for i in 0..agents.len() {
        for j in i + 1..agents.len() {
            let a = &agents[i];
            let b = &agents[j];
            
            // We only care about collisions between controlled and uncontrolled agents
            if a.uncontrolled == b.uncontrolled {
                continue;
            }

            let dist = (a.current_pos - b.current_pos).norm();
            if dist < 0.9 { // 0.45 * 2
                return true;
            }
        }
    }
    false
}

fn check_for_detailed_violation_mixed(plan: &mapf_post::SemanticPlan, current_pos: &[mapf_post::SemanticWaypoint]) -> bool {
    let mut agent_to_pos = HashMap::new();
    for &wp in current_pos {
        agent_to_pos.insert(wp.agent, wp);
    }

    for wp in current_pos {
        if let Some(deps) = plan.comes_before(wp) {
            for &dep_idx in deps {
                let dep_wp = plan.waypoints[dep_idx];

                if dep_wp.agent == wp.agent {
                    continue;
                }

                if let Some(other_agent) = agent_to_pos.get(&dep_wp.agent) {
                    if other_agent.trajectory_index >= dep_wp.trajectory_index {
                        // println!("VIOLATION DETECTED: Agent {} at {} vs Dep Agent {} at {} (dep was {})",
                        //    wp.agent, wp.trajectory_index, other_agent.agent, other_agent.trajectory_index, dep_wp.trajectory_index);
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
    collision_time: Option<f32>,
    violation_time: Option<f32>,
}

fn run_simulation(lookahead: usize, scene_seed: u64) -> SimulationResult {
    let mut agents = get_default_agents(lookahead, scene_seed);
    if agents.is_empty() {
        return SimulationResult::default();
    }

    // Initialize MAPF-POST
    let mut trajectories = vec![];
    let mut footprints = vec![];
    for obs in agents.iter() {
        let poses = obs.path.iter().map(|p| {
            Isometry2::new(Vector2::new(p.x, p.y), 0.0)
        }).collect();
        trajectories.push(Trajectory { poses });
        footprints.push(Arc::new(Ball::new(0.45)) as Arc<dyn mapf_post::shape::Shape>);
    }

    let mapf_result = MapfResult {
        trajectories,
        footprints,
        discretization_timestep: 0.1,
    };

    let plan = mapf_post(&mapf_result);

    for (i, obs) in agents.iter_mut().enumerate() {
        let poses = obs.path.iter().map(|p| {
            Isometry2::new(Vector2::new(p.x, p.y), 0.0)
        }).collect();
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

        if result.violation_time.is_none() && check_for_detailed_violation_mixed(&plan, &current_waypoints) {
            result.violation_time = Some(sim_time);
        }

        // Exit early only if both happened
        if result.collision_time.is_some() && result.violation_time.is_some() {
            return result;
        }

        // Check if all controlled agents reached their destination
        let mut all_reached = true;
        for obs in &agents {
            if !obs.uncontrolled {
                let target = *obs.path.last().unwrap();
                let dist = (obs.current_pos - target).norm();
                if dist > 0.05 {
                    all_reached = false;
                    break;
                }
            }
        }
        if all_reached {
            return result;
        }

        sim_time += DT;
    }

    result
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    
    let num_scenes = args.get(1)
        .and_then(|s| s.parse::<usize>().ok())
        .unwrap_or(100);
    
    let lookaheads = if args.len() > 2 {
        args[2..].iter()
            .filter_map(|s| s.parse::<usize>().ok())
            .collect::<Vec<_>>()
    } else {
        vec![0, 1, 2, 4, 8, 16]
    };

    println!("Running ablation with {} scenes and lookaheads: {:?}", num_scenes, lookaheads);
    println!("{:<10} | {:<12} | {:<12} | {:<15} | {:<15}", 
             "Lookahead", "Coll. %", "Viol. %", "Mean Coll. T", "Mean Viol. T");
    println!("{:-<10}-|-{:-<12}-|-{:-<12}-|-{:-<15}-|-{:-<15}", 
             "", "", "", "", "");

    for &lookahead in &lookaheads {
        let mut collisions = 0;
        let mut violations = 0;
        let mut total_collision_time = 0.0;
        let mut total_violation_time = 0.0;

        for scene_seed in 0..num_scenes {
            let res = run_simulation(lookahead, scene_seed as u64);
            if let Some(t) = res.collision_time {
                collisions += 1;
                total_collision_time += t;
            }
            if let Some(t) = res.violation_time {
                violations += 1;
                total_violation_time += t;
            }
        }

        let coll_perc = (collisions as f32 / num_scenes as f32) * 100.0;
        let viol_perc = (violations as f32 / num_scenes as f32) * 100.0;
        let mean_coll_t = if collisions > 0 { total_collision_time / collisions as f32 } else { 0.0 };
        let mean_viol_t = if violations > 0 { total_violation_time / violations as f32 } else { 0.0 };

        println!("{:<10} | {:<11.1}% | {:<11.1}% | {:<15.2} | {:<15.2}", 
                 lookahead, coll_perc, viol_perc, mean_coll_t, mean_viol_t);
    }
}
