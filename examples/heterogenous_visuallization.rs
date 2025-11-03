use std::{
    fs::File,
    io::{BufRead, BufReader},
};

use hetpibt::{
    HetPiBT, HeterogenousAgent, collision_checker::MultiGridCollisionChecker, parse_grid,
    parse_grid_with_scale,
};
use macroquad::prelude::*;

fn load_grid() -> (
    Vec<f32>,
    Vec<(usize, usize)>,
    Vec<HeterogenousAgent>,
    Vec<Vec<bool>>,
) {
    let args: Vec<String> = std::env::args().collect();

    if args.len() == 3 {
        let map_file = &args[1];
        let scene_file = &args[2];

        let grid = parse_grid_with_scale(map_file, 10);

        let mut new_grid = vec![vec![false; grid[0].len()]; grid.len()];

        for x in 0..grid.len() {
            for y in 0..grid.len() {
                if grid[x][y] != 0 {
                    new_grid[x][y] = true;
                }
            }
        }

        let f = File::open(scene_file).unwrap();
        let reader = BufReader::new(f);
        let mut graph_scales = vec![];
        let mut graph_bounds = vec![];
        let mut agents = vec![];
        for line in reader.lines() {
            let Ok(l) = line else {
                continue;
            };
            let p: Vec<_> = l.split_ascii_whitespace().collect();
            let _agent_id: usize = p[0].parse().unwrap();
            let fleet_id: usize = p[1].parse().unwrap();
            let footprint_size: f32 = p[2].parse().unwrap();
            let _velocity: f32 = p[3].parse().unwrap();
            let start_x: f32 = p[4].parse().unwrap();
            let start_y: f32 = p[5].parse().unwrap();
            let goal_x: f32 = p[6].parse().unwrap();
            let goal_y: f32 = p[7].parse().unwrap();
            let grid_width: usize = p[8].parse().unwrap();
            let grid_height: usize = p[9].parse().unwrap();

            while graph_scales.len() <= fleet_id {
                println!("Got {}  but had", fleet_id);
                graph_scales.push(0.0);
            }
            graph_scales[fleet_id] = footprint_size;

            let collision_checker = MultiGridCollisionChecker {
                grid_sizes: graph_scales.clone(),
            };
            let start = collision_checker.get_grid_space(fleet_id, start_x, start_y);
            let end = collision_checker.get_grid_space(fleet_id, goal_x, goal_y);
            agents.push(HeterogenousAgent {
                graph_id: fleet_id,
                start,
                end,
            });

            println!("{:?}", (start, end));
            println!("{:?}", agents.last());

            while graph_bounds.len() <= fleet_id {
                graph_bounds.push((0, 0));
            }
            graph_bounds[fleet_id] = (grid_width, grid_height);
        }
        return (graph_scales, graph_bounds, agents, new_grid);
    }
    let base_obstacles = vec![vec![false; 20]; 20];
    let graph_scale = vec![1.0, 2.0];
    let graph_bound = vec![(20, 20), (10, 10)];
    /// Default
    let agents = vec![
        HeterogenousAgent {
            graph_id: 0,
            start: (0, 4),
            end: (0, 6),
        },
        HeterogenousAgent {
            graph_id: 0,
            start: (0, 3),
            end: (0, 7),
        },
        HeterogenousAgent {
            graph_id: 1,
            start: (0, 0),
            end: (0, 1),
        }
    ];

    // Push
    /*let agents = vec![
        HeterogenousAgent {
            graph_id: 0,
            start: (0, 4),
            end: (0, 6),
        },
        HeterogenousAgent {
            graph_id: 0,
            start: (0, 3),
            end: (0, 7),
        },
    ];*/

    // swap
    /*let agents = vec![
        HeterogenousAgent {
            graph_id: 0,
            start: (0, 2),
            end: (0, 5),
        },
        HeterogenousAgent {
            graph_id: 0,
            start: (0, 3),
            end: (0, 0),
        },
    ];*/
    (graph_scale, graph_bound, agents, base_obstacles)
}

#[macroquad::main("Movement Visualization")]
async fn main() {
    println!("Reading configuration");
    let (graph_scale, graph_bounds, agents, base_obstacles) = load_grid();
    println!("Initializing solver");
    let mut het_pibt = HetPiBT::init_solver(
        &base_obstacles,
        graph_scale.clone(),
        graph_bounds,
        agents.clone(),
    );
    let steps = 500;
    println!("Calculated individual agent cost maps");
    let result = het_pibt.solve(steps);
    println!("Result time: {:?}", result);
    let mut last_update = std::time::SystemTime::now();
    let mut time = 0;
    loop {
        clear_background(BLACK);
        let p = het_pibt.get_trajectories(time);
        if last_update.elapsed().unwrap().as_secs_f64() > 0.1 {
            time += 1;
            last_update = std::time::SystemTime::now();
            if let Some(max_time) = result {
                if time > max_time {
                    time = 0;
                }
            } else {
                if time > steps {
                    time = 0;
                }
            }
        }

        let colors = [RED, GREEN, BLUE, YELLOW, ORANGE, PURPLE, VIOLET];
        for (agent, px) in p.iter().enumerate() {
            if let Some((g, x, y)) = px {
                let g_scale = graph_scale[*g];
                let h_scale = 1.0;
                let x = *x as f32;
                let y = *y as f32;
                let x = x * g_scale * h_scale + g_scale * h_scale / 2.0;
                let y = y * g_scale * h_scale + g_scale * h_scale / 2.0;
                let color = colors[agent % colors.len()];
                /// Draws a circle to the end goal
                draw_circle(x, y, h_scale * g_scale / 2.0, color);
                let end_x =
                    (agents[agent].end.0 as f32) * g_scale * h_scale + g_scale * h_scale / 2.0;
                let end_y =
                    (agents[agent].end.1 as f32) * g_scale * h_scale + g_scale * h_scale / 2.0;
                /// Draws a line to the goal
                draw_line(x, y, end_x, end_y, 1.0, color);
            }
        }

        next_frame().await;
    }
}
