use hetpibt::{HetPiBT, HeterogenousAgent};
use macroquad::prelude::*;

#[macroquad::main("Movement Visualization")]
async fn main() {
    let base_obstacles = vec![vec![false; 20]; 20];
    let graph_scale = vec![1.0, 2.0];
    let agents = vec![
            HeterogenousAgent {
                graph_id: 0,
                start: (0, 2),
                end: (0, 6),
            },
            HeterogenousAgent {
                graph_id: 0,
                start: (0, 3),
                end: (0, 8),
            },
            HeterogenousAgent {
                graph_id: 1,
                start: (0, 0),
                end: (0, 1),
            },
        ];
    let mut het_pibt = HetPiBT::init_solver(
        &base_obstacles,
        graph_scale.clone(),
        vec![(20, 20), (10, 10)],
        agents.clone()
    );
    println!("Calculated dijkstra");
    let result = het_pibt.solve(100);
    println!("Result time: {:?}", result);
    let mut last_update = std::time::SystemTime::now();
    let mut time = 0;
    loop {
        clear_background(BLACK);
        let p = het_pibt.get_trajectories(time);
        if last_update.elapsed().unwrap().as_secs_f64() > 1.0 {
            time += 1;
            last_update = std::time::SystemTime::now();
            if let Some(max_time) = result {
                if time >= max_time{
                    time = 0;
                }
            }
        }

        for (agent,px) in p.iter().enumerate() {
            if let Some((g,x,y)) = px {
                let g_scale = graph_scale[*g];
                let h_scale = 10.0;
                let x = *x as f32;
                let y = *y as f32;
                let x = x * g_scale *h_scale + g_scale *h_scale /2.0;
                let y = y * g_scale * h_scale + g_scale *h_scale /2.0;
                draw_circle(x,y, h_scale * g_scale/2.0, RED);
                let end_x = (agents[agent].end.0 as f32) * g_scale *h_scale + g_scale *h_scale /2.0;
                let end_y = (agents[agent].end.1 as f32) * g_scale *h_scale + g_scale *h_scale /2.0;
                draw_line(x, y, end_x, end_y, 1.0, RED);
            }
        }

        next_frame().await;
    }
}
