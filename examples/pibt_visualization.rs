use hetpibt::{PiBT, parse_grid, parse_scen};
use macroquad::prelude::*;
use quad_rand as rand;

fn get_scenario() -> (Vec<Vec<(usize, usize)>>, PiBT) {
    let args: Vec<String> = std::env::args().collect();

    if args.len() == 3 {
        let map_file = &args[1];
        let scene_file = &args[2];

        let grid = parse_grid(map_file);
        let scene = parse_scen(scene_file).unwrap();

        return (scene, PiBT::init(grid));
    } else {
        let problem = vec![vec![(0, 0), (1, 0), (1, 1)], vec![(2, 0), (2, 1), (2, 2)]];

        let solver = PiBT::init_empty_world(10, 10);
        (problem, solver)
    }
}

#[macroquad::main("Movement Visualization")]
async fn main() {
    let (problem, mut solver) = get_scenario();
    let trajectories = solver.solve(&problem[0], &problem[1], 20).unwrap();

    let mut agent_colors = vec![];
    for _ in 0..problem[0].len() {
        let r = rand::gen_range(0.0, 1.0);
        let g = rand::gen_range(0.0, 1.0);
        let b = rand::gen_range(0.0, 1.0);

        agent_colors.push(Color::new(r, g, b, 1.0));
    }

    let mut last_update = std::time::SystemTime::now();
    let mut time = 0;
    loop {
        clear_background(BLACK);

        for x in 0..solver.grid.len() {
            for y in 0..solver.grid[x].len() {
                if solver.grid[x][y] != 0 {
                    let x = x as f32 * 20.0;
                    let y = y as f32 * 20.0;
                    draw_rectangle(x - 10.0, y - 10.0, 20.0, 20.0, WHITE);
                }
            }
        }

        for agent in 0..trajectories[time].len() {
            let pos = trajectories[time][agent];
            let (x, y) = (pos.0 as f32 * 20.0, pos.1 as f32 * 20.0);
            draw_circle(x + 10.0, y + 10.0, 10.0, agent_colors[agent]);
        }

        if last_update.elapsed().unwrap().as_secs_f64() > 1.0 {
            time += 1;
            if time >= trajectories.len() {
                time = 0;
            }
            last_update = std::time::SystemTime::now();
        }

        next_frame().await
    }
}
