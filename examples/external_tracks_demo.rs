use hetpibt::external_tracks_pibt::PiBTWithExternalTracks;
use macroquad::prelude::*;

#[macroquad::main("External Tracks PIBT Demo")]
async fn main() {
    // 8x8 grid
    let width = 8;
    let height = 8;
    let grid = vec![vec![0; width]; height];

    let mut solver = PiBTWithExternalTracks::init(grid);

    // Internal agents (Free agents)
    let starts = vec![(0, 0), (0, 7)];
    let ends = vec![(7, 7), (7, 0)];

    // External agents (Fixed/Not-free agents)
    let mut external_tracks = vec![];

    // External 0: Horizontal across the middle
    let mut track0 = vec![];
    for y in 0..8 {
        track0.push((4, y));
    }
    external_tracks.push(track0);

    // External 1: Vertical across the middle
    let mut track1 = vec![];
    for x in 0..8 {
        track1.push((x, 4));
    }
    external_tracks.push(track1);

    // External 2: Horizontal at row 2, moving backwards, then stay at (0,0)
    let mut track2 = vec![];
    for y in (0..8).rev() {
        track2.push((2, y));
    }
    // Add some steps moving it out of the way if possible, or just change its track
    // Actually, let's just make it move to row 1 after it finishes to clear row 2
    track2.push((1, 0));
    track2.push((0, 0));
    external_tracks.push(track2);

    let max_time = 100;
    let result = solver.solve(&starts, &ends, &external_tracks, max_time);

    if let Ok(trajectories) = result {
        println!("Solved! Visualization starting...");
        
        let free_agent_color = BLUE;
        let external_agent_color = RED;

        let mut time = 0;
        let mut last_update = get_time();

        loop {
            clear_background(BLACK);

            // Draw Grid
            for i in 0..=width {
                draw_line(i as f32 * 40.0, 0.0, i as f32 * 40.0, height as f32 * 40.0, 1.0, DARKGRAY);
            }
            for j in 0..=height {
                draw_line(0.0, j as f32 * 40.0, width as f32 * 40.0, j as f32 * 40.0, 1.0, DARKGRAY);
            }

            // Draw External Agents (Red)
            for (idx, ext_track) in external_tracks.iter().enumerate() {
                let ext_pos = if time < ext_track.len() {
                    ext_track[time]
                } else {
                    *ext_track.last().unwrap()
                };
                draw_rectangle(ext_pos.1 as f32 * 40.0 + 5.0, ext_pos.0 as f32 * 40.0 + 5.0, 30.0, 30.0, external_agent_color);
                draw_text(&format!("Ext {}", idx), ext_pos.1 as f32 * 40.0, ext_pos.0 as f32 * 40.0 - 5.0, 15.0, RED);
            }

            // Draw Free Agents Goals (Light Blue)
            for (idx, pos) in ends.iter().enumerate() {
                draw_circle_lines(pos.1 as f32 * 40.0 + 20.0, pos.0 as f32 * 40.0 + 20.0, 18.0, 2.0, SKYBLUE);
                draw_text(&format!("Goal {}", idx), pos.1 as f32 * 40.0, pos.0 as f32 * 40.0 - 25.0, 15.0, SKYBLUE);
            }

            // Draw Free Agents (Blue)
            for agent_idx in 0..trajectories[time].len() {
                let pos = trajectories[time][agent_idx];
                draw_circle(pos.1 as f32 * 40.0 + 20.0, pos.0 as f32 * 40.0 + 20.0, 15.0, free_agent_color);
                draw_text(&format!("Free {}", agent_idx), pos.1 as f32 * 40.0, pos.0 as f32 * 40.0 - 5.0, 15.0, BLUE);
            }

            draw_text(&format!("Time: {}", time), 10.0, height as f32 * 40.0 + 20.0, 20.0, WHITE);
            draw_text("Blue: Free Agents (PIBT controlled)", 10.0, height as f32 * 40.0 + 40.0, 20.0, BLUE);
            draw_text("Red: External Agent (Fixed track)", 10.0, height as f32 * 40.0 + 60.0, 20.0, RED);

            if get_time() - last_update > 0.5 {
                time += 1;
                if time >= trajectories.len() {
                    time = 0;
                }
                last_update = get_time();
            }

            next_frame().await
        }
    } else {
        println!("Failed to solve the demo scenario.");
    }
}
