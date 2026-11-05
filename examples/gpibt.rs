use hetpibt::*;
use macroquad::prelude::*;

#[macroquad::main("gpibt nav-mesh demo")]
async fn main() {
    let nav = generate_highway_with_dropoffs((292.0, 90.0), 60.0);

    // Spawn 6 robots at the 6 drop-off bays (3 on left, 3 on right) targeting opposite bays
    let starts = vec![
        nav.polygons[21].centroid, // Left drop-off 0 (row 1)
        nav.polygons[22].centroid, // Left drop-off 1 (row 3)
        nav.polygons[23].centroid, // Left drop-off 2 (row 5)
        nav.polygons[24].centroid, // Right drop-off 0 (row 1)
        nav.polygons[25].centroid, // Right drop-off 1 (row 3)
        nav.polygons[26].centroid, // Right drop-off 2 (row 5)
    ];
    let ends = vec![
        nav.polygons[26].centroid, // Right drop-off 2 (row 5)
        nav.polygons[25].centroid, // Right drop-off 1 (row 3)
        nav.polygons[24].centroid, // Right drop-off 0 (row 1)
        nav.polygons[23].centroid, // Left drop-off 2 (row 5)
        nav.polygons[22].centroid, // Left drop-off 1 (row 3)
        nav.polygons[21].centroid, // Left drop-off 0 (row 1)
    ];

    let mut solver = PIBTOverNavGraph::init(nav.clone());
    let trajectories = solver.solve(starts, ends.clone(), 50);

    let agent_colors = [
        Color::new(0.95, 0.30, 0.30, 1.0), // Red
        Color::new(0.30, 0.85, 0.45, 1.0), // Green
        Color::new(0.30, 0.60, 0.95, 1.0), // Blue
        Color::new(0.95, 0.80, 0.25, 1.0), // Yellow
        Color::new(0.85, 0.40, 0.95, 1.0), // Purple
        Color::new(0.25, 0.90, 0.90, 1.0), // Cyan
    ];

    let mut last_update = std::time::Instant::now();
    let mut time = 0usize;

    loop {
        clear_background(Color::new(0.08, 0.08, 0.1, 1.0));

        // Draw 3x7 center highway with 3 drop-off points on either side and connections
        nav.draw_styled(Color::new(0.7, 0.7, 0.8, 1.0), 2.0, true);

        // Draw goal markers for each agent
        for (agent, &(gx, gy)) in ends.iter().enumerate() {
            draw_circle_lines(gx, gy, 18.0, 2.0, agent_colors[agent]);
        }

        // Draw agents at current timestep
        if !trajectories.is_empty() {
            for (agent, &(x, y)) in trajectories[time].iter().enumerate() {
                draw_circle(x, y, 14.0, agent_colors[agent]);
            }

            if last_update.elapsed().as_secs_f32() >= 0.5 {
                time = (time + 1) % trajectories.len();
                last_update = std::time::Instant::now();
            }
        }

        next_frame().await;
    }
}
