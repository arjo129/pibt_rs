use macroquad::prelude::*;
use hetpibt::external_tracks_pibt::PiBTWithExternalTracks;
use std::collections::{VecDeque, HashMap};
use std::fs;
use std::sync::Arc;
use ::rand::{SeedableRng, Rng, rngs::StdRng};
use mapf_post::{
    mapf_post, MapfResult, SemanticPlan, Trajectory, WaypointFollower,
    na::{Isometry2, Vector2},
    shape::Ball,
};

const OBSTACLE_SPEED: f32 = 4.0; // Cells per second
const SEED: u64 = 42;

struct MovingObstacle {
    path: Vec<Vec2>,
    follower: Option<WaypointFollower>,
    color: Color,
    agent_id: usize,
    current_pos: Vec2,
    uncontrolled: bool,
    stall_timer: f32,
}

impl MovingObstacle {
    fn new(path: Vec<Vec2>, color: Color, agent_id: usize) -> Self {
        let start_pos = path[0];
        Self {
            path,
            follower: None,
            color,
            agent_id,
            current_pos: start_pos,
            uncontrolled: true,
            stall_timer: 0.0,
        }
    }

    fn update(&mut self, dt: f32, claim_dict: &HashMap<usize, mapf_post::CurrentlyAllocatedTrajSegment>) {
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
                // We can move towards the next waypoint
                let next_wp = follower.next_waypoint();
                let target_pos = vec2(next_wp.translation.x, next_wp.translation.y);
                
                let dir = target_pos - self.current_pos;
                let dist = dir.length();
                
                if dist > 0.001 {
                    let move_dist = (OBSTACLE_SPEED * dt).min(dist);
                    self.current_pos += dir.normalize() * move_dist;
                } else {
                    // Randomly stall at waypoints, seeded for fairness
                    let wp_idx = follower.get_semantic_waypoint().trajectory_index;
                    let mut rng = StdRng::seed_from_u64(SEED + self.agent_id as u64 * 1000 + wp_idx as u64);

                    let mut extra_stall = 0.0;
                    while rng.random_range(0..100) < 80 {
                        extra_stall += 0.3;
                    }
                    if extra_stall > 0.0 {
                        self.stall_timer = extra_stall;
                        return;
                    }
                }

                // Update follower with new position
                let isometry = Isometry2::new(Vector2::new(self.current_pos.x, self.current_pos.y), 0.0);
                follower.update_position_estimate(&isometry, 0.1);
            }
        }
    }

    fn draw(&self, cell_size: f32) {
        let pos = self.current_pos;
        let mut color = self.color;
        if self.stall_timer > 0.0 {
            color.a = 0.5;
        }
        draw_rectangle(
            pos.x * cell_size + 2.0,
            pos.y * cell_size + 2.0,
            cell_size - 4.0,
            cell_size - 4.0,
            color,
        );
        if self.uncontrolled {
            draw_rectangle_lines(
                pos.x * cell_size + 2.0,
                pos.y * cell_size + 2.0,
                cell_size - 4.0,
                cell_size - 4.0,
                3.0,
                WHITE,
            );
        }
    }
}

fn find_path_bfs(grid: &[Vec<bool>], start: Vec2, end: Vec2) -> Option<Vec<Vec2>> {
    let rows = grid.len();
    let cols = grid[0].len();
    let start_node = (start.y as i32, start.x as i32);
    let end_node = (end.y as i32, end.x as i32);

    if start_node.0 < 0 || start_node.0 >= rows as i32 || start_node.1 < 0 || start_node.1 >= cols as i32 ||
       end_node.0 < 0 || end_node.0 >= rows as i32 || end_node.1 < 0 || end_node.1 >= cols as i32 {
        return None;
    }

    if grid[start_node.0 as usize][start_node.1 as usize] || grid[end_node.0 as usize][end_node.1 as usize] {
        return None;
    }

    let mut queue = VecDeque::new();
    queue.push_back(start_node);

    let mut parent = HashMap::new();
    parent.insert(start_node, None);

    while let Some(current) = queue.pop_front() {
        if current == end_node {
            let mut path = Vec::new();
            let mut curr = Some(end_node);
            while let Some(node) = curr {
                path.push(vec2(node.1 as f32, node.0 as f32));
                curr = parent[&node];
            }
            path.reverse();
            return Some(path);
        }

        let (r, c) = current;
        for (dr, dc) in [(-1, 0), (1, 0), (0, -1), (0, 1)] {
            let nr = r + dr;
            let nc = c + dc;

            if nr >= 0 && nr < rows as i32 && nc >= 0 && nc < cols as i32 {
                let next = (nr, nc);
                if !grid[nr as usize][nc as usize] && !parent.contains_key(&next) {
                    parent.insert(next, Some(current));
                    queue.push_back(next);
                }
            }
        }
    }

    None
}

fn load_grid_from_file(path: &str) -> Option<Vec<Vec<bool>>> {
    let content = fs::read_to_string(path).ok()?;
    let mut grid = Vec::new();
    let lines = content.lines();

    let mut lines_iter = lines.peekable();
    if let Some(first_line) = lines_iter.peek() {
        if first_line.starts_with("type") {
            for _ in 0..4 {
                lines_iter.next();
            }
        }
    }

    for line in lines_iter {
        let row: Vec<bool> = line.chars().map(|c| c == '@' || c == '#').collect();
        if !row.is_empty() {
            grid.push(row);
        }
    }
    Some(grid)
}

fn get_default_grid() -> Vec<Vec<bool>> {
    vec![vec![false; 8]; 8]
}

fn get_default_agents(grid: &[Vec<bool>], lookahead: usize) -> Vec<MovingObstacle> {
    let mut agents = vec![];

    // External 0: Horizontal across the middle (row 4)
    let mut track0 = vec![];
    for x in 0..8 {
        track0.push(vec2(x as f32, 4.0));
    }
    let mut obs0 = MovingObstacle::new(track0, RED, agents.len());
    obs0.uncontrolled = true;
    agents.push(obs0);

    // External 1: Vertical across the middle (col 4)
    let mut track1 = vec![];
    for y in 0..8 {
        track1.push(vec2(4.0, y as f32));
    }
    let mut obs1 = MovingObstacle::new(track1, RED, agents.len());
    obs1.uncontrolled = true;
    agents.push(obs1);

    // External 2: row 2, moving backwards, then stay at (0,0)
    let mut track2 = vec![];
    for x in (0..8).rev() {
        track2.push(vec2(x as f32, 2.0));
    }
    track2.push(vec2(0.0, 1.0));
    track2.push(vec2(0.0, 0.0));
    let mut obs2 = MovingObstacle::new(track2, RED, agents.len());
    obs2.uncontrolled = true;
    agents.push(obs2);

    // Collect external tracks for PIBT
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
    solver.set_seed(SEED);

    let starts = vec![(0, 0), (0, 7)]; // (row, col)
    let ends = vec![(7, 7), (7, 0)];   // (row, col)

    let max_time = 500;
    let result = solver.solve(&starts, &ends, &external_tracks, max_time);

    if let Ok(trajectories) = result {
        // trajectories is time x agent_id
        if !trajectories.is_empty() {
            let num_internal_agents = trajectories[0].len();
            for i in 0..num_internal_agents {
                let mut path = vec![];
                for t in 0..trajectories.len() {
                    let pos = trajectories[t][i];
                    path.push(vec2(pos.1 as f32, pos.0 as f32));
                }
                let mut obs = MovingObstacle::new(path, BLUE, agents.len());
                obs.uncontrolled = false;
                agents.push(obs);
            }
        }
    } else {
        // Fallback to BFS if PIBT fails
        // Free Agent 0: (0,0) to (7,7)
        if let Some(path) = find_path_bfs(grid, vec2(0.0, 0.0), vec2(7.0, 7.0)) {
            let mut obs = MovingObstacle::new(path, BLUE, agents.len());
            obs.uncontrolled = false;
            agents.push(obs);
        }

        // Free Agent 1: (7,0) to (0,7)
        if let Some(path) = find_path_bfs(grid, vec2(7.0, 0.0), vec2(0.0, 7.0)) {
            let mut obs = MovingObstacle::new(path, BLUE, agents.len());
            obs.uncontrolled = false;
            agents.push(obs);
        }
    }

    agents
}

fn check_for_detailed_violation_mixed(plan: &SemanticPlan, current_pos: &[mapf_post::SemanticWaypoint], agents: &[MovingObstacle]) {
    let mut agent_to_pos = HashMap::new();
    for &wp in current_pos {
        agent_to_pos.insert(wp.agent, wp);
    }

    let agent_uncontrolled: HashMap<usize, bool> = agents.iter().map(|a| (a.agent_id, a.uncontrolled)).collect();

    for wp in current_pos {
        // Only check for violations if THIS agent (the 'after' agent) is uncontrolled.
        // Controlled agents will wait and thus won't violate the plan.
        if !agent_uncontrolled.get(&wp.agent).cloned().unwrap_or(false) {
            continue;
        }

        if let Some(deps) = plan.comes_before(wp) {
            for &dep_idx in deps {
                let dep_wp = plan.waypoints[dep_idx];

                if dep_wp.agent == wp.agent {
                    continue;
                }

                if let Some(other_agent) = agent_to_pos.get(&dep_wp.agent) {
                    if other_agent.trajectory_index <= dep_wp.trajectory_index {
                        println!("violation found at t={} between agents [{}, {}]", 
                            wp.trajectory_index, wp.agent, other_agent.agent);
                    }
                }
            }
        }
    }
}

#[macroquad::main("Grid and MAPF-POST")]
async fn main() {
    let args: Vec<String> = std::env::args().collect();
    let grid = if args.len() > 1 {
        load_grid_from_file(&args[1]).unwrap_or_else(get_default_grid)
    } else {
        get_default_grid()
    };

    let rows = grid.len();
    let cols = if rows > 0 { grid[0].len() } else { 0 };
    let cell_size = 40.0;

    let mut lookahead = 0;
    let mut moving_obstacles: Vec<MovingObstacle> = get_default_agents(&grid, lookahead);
    let mut pending_start: Option<Vec2> = None;
    let mut status_msg = String::from("Click twice to add agent. L: Toggle Lookahead. SPACE: MAPF-POST.");
    let mut semantic_plan: Option<SemanticPlan> = None;

    loop {
        clear_background(BLACK);

        let dt = get_frame_time();
        let (mouse_x, mouse_y) = mouse_position();
        let grid_mouse_pos = vec2(
            (mouse_x / cell_size).floor(),
            (mouse_y / cell_size).floor(),
        );

        // Input handling for adding obstacles or toggling uncontrolled
        if is_mouse_button_pressed(MouseButton::Left) {
            let mut clicked_agent = false;
            for obs in &mut moving_obstacles {
                let pos = obs.current_pos;
                if grid_mouse_pos.x == pos.x.round() && grid_mouse_pos.y == pos.y.round() {
                    obs.uncontrolled = !obs.uncontrolled;
                    status_msg = format!("Agent {} uncontrolled: {}", obs.agent_id, obs.uncontrolled);
                    clicked_agent = true;
                    break;
                }
            }

            if !clicked_agent {
                if let Some(start) = pending_start {
                    if let Some(path) = find_path_bfs(&grid, start, grid_mouse_pos) {
                        let agent_id = moving_obstacles.len();
                        let color = Color::from_rgba(
                            macroquad::rand::gen_range(100, 255) as u8,
                            macroquad::rand::gen_range(100, 255) as u8,
                            macroquad::rand::gen_range(100, 255) as u8,
                            255,
                        );
                        let mut obs = MovingObstacle::new(path, color, agent_id);
                        obs.uncontrolled = false; // New agents controlled by default
                        moving_obstacles.push(obs);
                        status_msg = format!("Added agent {}. Total: {}", agent_id, moving_obstacles.len());
                        semantic_plan = None; 
                    } else {
                        status_msg = String::from("No valid path found!");
                    }
                    pending_start = None;
                } else {
                    pending_start = Some(grid_mouse_pos);
                    status_msg = String::from("Select destination...");
                }
            }
        }

        if is_key_pressed(KeyCode::L) {
            lookahead = if lookahead == 0 { 3 } else { 0 };
            moving_obstacles = get_default_agents(&grid, lookahead);
            semantic_plan = None;
            status_msg = format!("Lookahead set to {}. PIBT re-run.", lookahead);
        }

        if is_key_pressed(KeyCode::Space) && !moving_obstacles.is_empty() {
            let mut trajectories = vec![];
            let mut footprints = vec![];
            for obs in &moving_obstacles {
                let poses = obs.path.iter().map(|p| {
                    Isometry2::new(Vector2::new(p.x, p.y), 0.0)
                }).collect();
                trajectories.push(Trajectory { poses });
                footprints.push(Arc::new(Ball::new(0.45)) as Arc<dyn mapf_post::shape::Shape>);
            }

            let mapf_result = MapfResult {
                trajectories,
                footprints,
                discretization_timestep: 1.0,
            };

            let plan = mapf_post(&mapf_result);
            semantic_plan = Some(plan);

            for (i, obs) in moving_obstacles.iter_mut().enumerate() {
                let poses = obs.path.iter().map(|p| {
                    Isometry2::new(Vector2::new(p.x, p.y), 0.0)
                }).collect();
                obs.follower = Some(WaypointFollower::from_trajectory(i, Trajectory { poses }));
                obs.current_pos = obs.path[0];
                // obs.uncontrolled is preserved
            }
            status_msg = String::from("MAPF-POST plan generated! White-bordered agents are uncontrolled.");
        }

        if is_key_pressed(KeyCode::C) {
            moving_obstacles.clear();
            semantic_plan = None;
            status_msg = String::from("Cleared agents.");
        }

        // Update obstacles
        let mut claim_dict = HashMap::new();
        if let Some(plan) = &semantic_plan {
            let current_waypoints: Vec<_> = moving_obstacles.iter_mut()
                .filter_map(|obs| obs.follower.as_mut().map(|f| f.get_semantic_waypoint()))
                .collect();
            
            // We always compute claim_dict based on current positions of ALL agents
            claim_dict = plan.get_claim_dict(&current_waypoints);
            
            // We also check for violations for UNCONTROLLED agents
            check_for_detailed_violation_mixed(plan, &current_waypoints, &moving_obstacles);
        }

        for obs in &mut moving_obstacles {
            obs.update(dt, &claim_dict);
        }

        // Draw grid
        for r in 0..rows {
            for c in 0..cols {
                let x = c as f32 * cell_size;
                let y = r as f32 * cell_size;
                draw_rectangle_lines(x, y, cell_size, cell_size, 1.0, DARKGRAY);
                if grid[r][c] {
                    draw_rectangle(x + 2.0, y + 2.0, cell_size - 4.0, cell_size - 4.0, GRAY);
                }
            }
        }

        // Draw visual aid for pending obstacle
        if let Some(start) = pending_start {
            if let Some(path) = find_path_bfs(&grid, start, grid_mouse_pos) {
                for i in 0..path.len() - 1 {
                    let p1 = path[i];
                    let p2 = path[i+1];
                    draw_line(
                        p1.x * cell_size + cell_size/2.0, p1.y * cell_size + cell_size/2.0,
                        p2.x * cell_size + cell_size/2.0, p2.y * cell_size + cell_size/2.0,
                        2.0, YELLOW
                    );
                }
                draw_rectangle(start.x * cell_size + 2.0, start.y * cell_size + 2.0, cell_size - 4.0, cell_size - 4.0, YELLOW);
                draw_rectangle(grid_mouse_pos.x * cell_size + 2.0, grid_mouse_pos.y * cell_size + 2.0, cell_size - 4.0, cell_size - 4.0, Color::new(1.0, 1.0, 0.0, 0.5));
            } else {
                draw_rectangle(start.x * cell_size + 2.0, start.y * cell_size + 2.0, cell_size - 4.0, cell_size - 4.0, RED);
            }
        }

        // Draw moving obstacles
        for obs in &moving_obstacles {
            obs.draw(cell_size);
        }

        // UI
        draw_text(&status_msg, 10.0, screen_height() - 40.0, 20.0, WHITE);
        draw_text("SPACE: Run MAPF-POST. 'C': Clear. ESC: Exit.", 10.0, screen_height() - 20.0, 20.0, WHITE);

        if is_key_down(KeyCode::Escape) {
            break;
        }

        next_frame().await
    }
}
