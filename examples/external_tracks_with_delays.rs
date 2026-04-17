use ::rand::Rng;
use hetpibt::external_tracks_pibt::PiBTWithExternalTracks;
use macroquad::prelude::*;

fn add_random_delays(track: Vec<(usize, usize)>, delay_prob: f64) -> Vec<(usize, usize)> {
    let mut new_track = vec![];
    let mut rng = ::rand::rng();

    for pos in track {
        new_track.push(pos);
        // Randomly stay at the same position
        while rng.random_bool(delay_prob) {
            new_track.push(pos);
        }
    }
    new_track
}

#[macroquad::main("External Tracks with Delays")]
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
    let mut base_external_tracks = vec![];

    // External 0: Horizontal across the middle
    let mut track0 = vec![];
    for y in 0..8 {
        track0.push((4, y));
    }
    base_external_tracks.push(track0);

    // External 1: Vertical across the middle
    let mut track1 = vec![];
    for x in 0..8 {
        track1.push((x, 4));
    }
    base_external_tracks.push(track1);

    // External 2: Horizontal at row 2, moving backwards, then stay at (0,0)
    let mut track2 = vec![];
    for y in (0..8).rev() {
        track2.push((2, y));
    }
    track2.push((1, 0));
    track2.push((0, 0));
    base_external_tracks.push(track2);

    // Apply random delays to uncontrolled agents
    let delay_probability = 0.8; // 10% chance to stay in place each step
    let mut delayed_external_tracks: Vec<Vec<(usize, usize)>> = base_external_tracks
        .clone()
        .into_iter()
        .map(|t| add_random_delays(t, delay_probability))
        .collect();

    for (i, track) in delayed_external_tracks.iter().enumerate() {
        println!("Delayed External Agent {} track length: {}", i, track.len());
    }

    let max_time = 500; // Increased to account for delays
    // PIBT plans with base tracks (no delays)
    let result = solver.solve(&starts, &ends, &base_external_tracks, max_time);

    if let Ok(trajectories) = result {
        println!("Solved with base tracks! Visualization starting...");

        let free_agent_color = BLUE;
        let external_agent_color = RED;

        let mut time = 0;
        let mut last_update = get_time();

        loop {
            clear_background(BLACK);

            // Draw Grid
            for i in 0..=width {
                draw_line(
                    i as f32 * 40.0,
                    0.0,
                    i as f32 * 40.0,
                    height as f32 * 40.0,
                    1.0,
                    DARKGRAY,
                );
            }
            for j in 0..=height {
                draw_line(
                    0.0,
                    j as f32 * 40.0,
                    width as f32 * 40.0,
                    j as f32 * 40.0,
                    1.0,
                    DARKGRAY,
                );
            }

            // Get Current External Agent Positions (Delayed)
            let mut current_external_positions = vec![];
            for (idx, ext_track) in delayed_external_tracks.iter().enumerate() {
                let ext_pos = if time < ext_track.len() {
                    ext_track[time]
                } else {
                    *ext_track.last().unwrap()
                };
                current_external_positions.push(ext_pos);

                // Draw External Agents (Red)
                draw_rectangle(
                    ext_pos.1 as f32 * 40.0 + 5.0,
                    ext_pos.0 as f32 * 40.0 + 5.0,
                    30.0,
                    30.0,
                    external_agent_color,
                );
                draw_text(
                    &format!("Ext {}", idx),
                    ext_pos.1 as f32 * 40.0,
                    ext_pos.0 as f32 * 40.0 - 5.0,
                    15.0,
                    RED,
                );
            }

            // Draw Free Agents Goals (Light Blue)
            for (idx, pos) in ends.iter().enumerate() {
                draw_circle_lines(
                    pos.1 as f32 * 40.0 + 20.0,
                    pos.0 as f32 * 40.0 + 20.0,
                    18.0,
                    2.0,
                    SKYBLUE,
                );
                draw_text(
                    &format!("Goal {}", idx),
                    pos.1 as f32 * 40.0,
                    pos.0 as f32 * 40.0 - 25.0,
                    15.0,
                    SKYBLUE,
                );
            }

            // Draw and Check Free Agents (Blue)
            let current_time_internal_positions = if time < trajectories.len() {
                &trajectories[time]
            } else {
                trajectories.last().unwrap()
            };

            for agent_idx in 0..current_time_internal_positions.len() {
                let pos = current_time_internal_positions[agent_idx];
                draw_circle(
                    pos.1 as f32 * 40.0 + 20.0,
                    pos.0 as f32 * 40.0 + 20.0,
                    15.0,
                    free_agent_color,
                );
                draw_text(
                    &format!("Free {}", agent_idx),
                    pos.1 as f32 * 40.0,
                    pos.0 as f32 * 40.0 - 5.0,
                    15.0,
                    BLUE,
                );

                // Collision Detection with External Agents
                for (ext_idx, ext_pos) in current_external_positions.iter().enumerate() {
                    if pos.0 == ext_pos.0 as i64 && pos.1 == ext_pos.1 as i64 {
                        println!(
                            "COLLISION at time {}: Free Agent {} with External Agent {} at {:?}",
                            time, agent_idx, ext_idx, pos
                        );
                        draw_circle(
                            pos.1 as f32 * 40.0 + 20.0,
                            pos.0 as f32 * 40.0 + 20.0,
                            20.0,
                            YELLOW,
                        );
                    }
                }

                // Collision Detection between Free Agents (shouldn't happen in PIBT but good to check)
                for other_idx in (agent_idx + 1)..current_time_internal_positions.len() {
                    let other_pos = current_time_internal_positions[other_idx];
                    if pos == other_pos {
                        println!(
                            "INTERNAL COLLISION at time {}: Free Agent {} with Free Agent {} at {:?}",
                            time, agent_idx, other_idx, pos
                        );
                    }
                }
            }

            draw_text(
                &format!("Time: {}", time),
                10.0,
                height as f32 * 40.0 + 20.0,
                20.0,
                WHITE,
            );
            draw_text(
                "Blue: Free Agents (PIBT planned w/o delays)",
                10.0,
                height as f32 * 40.0 + 40.0,
                20.0,
                BLUE,
            );
            draw_text(
                "Red: External Agents (ACTUAL with delays)",
                10.0,
                height as f32 * 40.0 + 60.0,
                20.0,
                RED,
            );
            draw_text(
                "Yellow: Collision!",
                10.0,
                height as f32 * 40.0 + 80.0,
                20.0,
                YELLOW,
            );

            if get_time() - last_update > 0.3 {
                let max_sim_time = trajectories.len().max(
                    delayed_external_tracks
                        .iter()
                        .map(|t| t.len())
                        .max()
                        .unwrap_or(0),
                );

                time += 1;
                if time >= max_sim_time {
                    println!("--- End of Simulation ---");

                    // Calculate collision stats for the round
                    let mut total_collision_count = 0;
                    let mut steps_with_collision = 0;
                    for t in 0..max_sim_time {
                        let mut step_had_collision = false;
                        let internal_positions = if t < trajectories.len() {
                            &trajectories[t]
                        } else {
                            trajectories.last().unwrap()
                        };

                        for pos in internal_positions.iter() {
                            for ext_track in delayed_external_tracks.iter() {
                                let ext_pos = if t < ext_track.len() {
                                    ext_track[t]
                                } else {
                                    *ext_track.last().unwrap()
                                };

                                if pos.0 == ext_pos.0 as i64 && pos.1 == ext_pos.1 as i64 {
                                    total_collision_count += 1;
                                    step_had_collision = true;
                                }
                            }
                        }
                        if step_had_collision {
                            steps_with_collision += 1;
                        }
                    }

                    let collision_percent =
                        (steps_with_collision as f64 / max_sim_time as f64) * 100.0;
                    println!("Collision Stats for Round:");
                    println!(
                        "  Steps with collisions: {} / {} ({:.2}%)",
                        steps_with_collision, max_sim_time, collision_percent
                    );
                    println!(
                        "  Total agent-agent collision events: {}",
                        total_collision_count
                    );

                    time = 0;

                    // Regenerate delays for the next round
                    delayed_external_tracks = base_external_tracks
                        .clone()
                        .into_iter()
                        .map(|t| add_random_delays(t, delay_probability))
                        .collect();
                    for (i, track) in delayed_external_tracks.iter().enumerate() {
                        println!(
                            "New Round - Delayed External Agent {} track length: {}",
                            i,
                            track.len()
                        );
                    }
                }
                last_update = get_time();
            }

            next_frame().await
        }
    } else {
        println!("Failed to solve the base scenario.");
    }
}
