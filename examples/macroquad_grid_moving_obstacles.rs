use macroquad::prelude::*;
use std::collections::{VecDeque, HashMap};
use std::fs;

const OBSTACLE_SPEED: f32 = 4.0; // Cells per second

struct MovingObstacle {
    path: Vec<Vec2>,
    current_segment: usize,
    segment_progress: f32,
    moving_forward: bool,
    color: Color,
}

impl MovingObstacle {
    fn new(path: Vec<Vec2>, color: Color) -> Self {
        Self {
            path,
            current_segment: 0,
            segment_progress: 0.0,
            moving_forward: true,
            color,
        }
    }

    fn update(&mut self, dt: f32) {
        if self.path.len() < 2 {
            return;
        }

        let p1 = self.path[self.current_segment];
        let p2 = if self.moving_forward {
            self.path[self.current_segment + 1]
        } else {
            self.path[self.current_segment - 1]
        };

        let distance = p1.distance(p2);
        if distance < 0.001 {
            self.advance_segment();
            return;
        }

        let travel_time = distance / OBSTACLE_SPEED;
        let progress_step = dt / travel_time;

        self.segment_progress += progress_step;

        if self.segment_progress >= 1.0 {
            self.segment_progress = 0.0;
            self.advance_segment();
        }
    }

    fn advance_segment(&mut self) {
        if self.moving_forward {
            self.current_segment += 1;
            if self.current_segment >= self.path.len() - 1 {
                self.moving_forward = false;
            }
        } else {
            self.current_segment -= 1;
            if self.current_segment == 0 {
                self.moving_forward = true;
            }
        }
    }

    fn current_pos(&self) -> Vec2 {
        if self.path.is_empty() { return vec2(0.0, 0.0); }
        if self.path.len() == 1 { return self.path[0]; }

        let p1 = self.path[self.current_segment];
        let p2 = if self.moving_forward {
            self.path[self.current_segment + 1]
        } else {
            self.path[self.current_segment - 1]
        };

        p1.lerp(p2, self.segment_progress)
    }

    fn draw(&self, cell_size: f32) {
        let pos = self.current_pos();
        draw_rectangle(
            pos.x * cell_size + 2.0,
            pos.y * cell_size + 2.0,
            cell_size - 4.0,
            cell_size - 4.0,
            self.color,
        );
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
    vec![
        vec![false; 10],
        vec![false, true,  true,  true,  true,  true,  true,  true,  true,  false],
        vec![false, false, false, false, false, false, false, false, true,  false],
        vec![true,  true,  true,  true,  true,  true,  true,  false, true,  false],
        vec![false, false, false, false, false, false, false, false, true,  false],
        vec![false, true,  true,  true,  true,  true,  true,  true,  true,  false],
        vec![false, false, false, false, false, false, false, false, false, false],
        vec![true,  true,  true,  true,  true,  true,  true,  true,  true,  false],
        vec![false, false, false, false, false, false, false, false, false, false],
        vec![false, false, false, false, false, false, false, false, false, false],
    ]
}

#[macroquad::main("Grid and Moving Obstacles (BFS Path)")]
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

    let mut moving_obstacles = vec![];
    let mut pending_start: Option<Vec2> = None;
    let mut status_msg = String::from("Click twice to add an obstacle. BFS will find the shortest path.");

    loop {
        clear_background(BLACK);

        let dt = get_frame_time();
        let (mouse_x, mouse_y) = mouse_position();
        let grid_mouse_pos = vec2(
            (mouse_x / cell_size).floor(),
            (mouse_y / cell_size).floor(),
        );

        // Input handling for adding obstacles
        if is_mouse_button_pressed(MouseButton::Left) {
            if let Some(start) = pending_start {
                if let Some(path) = find_path_bfs(&grid, start, grid_mouse_pos) {
                    let path_len = path.len();
                    let color = Color::from_rgba(
                        rand::gen_range(100, 255) as u8,
                        rand::gen_range(100, 255) as u8,
                        rand::gen_range(100, 255) as u8,
                        255,
                    );
                    moving_obstacles.push(MovingObstacle::new(path, color));
                    status_msg = format!("Added obstacle with path length {}", path_len);
                } else {
                    status_msg = String::from("No valid path found (blocked by static obstacles)!");
                }
                pending_start = None;
            } else {
                pending_start = Some(grid_mouse_pos);
                status_msg = String::from("Select destination...");
            }
        }

        if is_key_pressed(KeyCode::C) {
            moving_obstacles.clear();
            status_msg = String::from("Cleared obstacles.");
        }

        if is_key_pressed(KeyCode::P) {
            export_paths(&moving_obstacles);
            status_msg = String::from("Paths exported to console and 'moving_obstacles.json'");
        }

        // Update obstacles
        for obs in &mut moving_obstacles {
            obs.update(dt);
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
        draw_text("Press 'P' to export paths. 'C' to clear. ESC to exit.", 10.0, screen_height() - 20.0, 20.0, WHITE);

        if is_key_down(KeyCode::Escape) {
            break;
        }

        next_frame().await
    }
}

fn export_paths(obstacles: &[MovingObstacle]) {
    if obstacles.is_empty() {
        println!("No obstacles to export.");
        return;
    }

    let mut output = String::from("[\n");
    for (i, obs) in obstacles.iter().enumerate() {
        output.push_str("  {\n");
        output.push_str("    \"path\": [");
        for (j, pos) in obs.path.iter().enumerate() {
            output.push_str(&format!("[{}, {}]", pos.x as i32, pos.y as i32));
            if j < obs.path.len() - 1 {
                output.push_str(", ");
            }
        }
        output.push_str("]\n");
        output.push_str("  }");
        if i < obstacles.len() - 1 {
            output.push_str(",\n");
        } else {
            output.push_str("\n");
        }
    }
    output.push_str("]");
    
    println!("\n=== EXPORTED PATHS (JSON) ===\n{}\n============================\n", output);
    
    match std::fs::write("moving_obstacles.json", output) {
        Ok(_) => println!("Successfully saved to 'moving_obstacles.json'"),
        Err(e) => eprintln!("Error saving to file: {}", e),
    }
}
