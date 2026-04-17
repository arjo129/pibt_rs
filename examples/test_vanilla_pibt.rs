use hetpibt::external_tracks_pibt::PiBTWithExternalTracks;
use rand::{SeedableRng, Rng, rngs::StdRng, seq::SliceRandom};

fn main() {
    let size = 16;
    let num_controlled = 5;
    let mut rng = StdRng::seed_from_u64(42);
    
    let mut starts = vec![];
    let mut ends = vec![];
    let mut possible_cells: Vec<(usize, usize)> = vec![];
    for r in 0..size { for c in 0..size { possible_cells.push((r, c)); } }
    possible_cells.shuffle(&mut rng);

    for i in 0..num_controlled {
        starts.push(possible_cells[i*2]);
        ends.push(possible_cells[i*2 + 1]);
    }

    let grid = vec![vec![0; size]; size];
    let mut solver = PiBTWithExternalTracks::init(grid);
    solver.lookahead = 0;
    
    println!("Testing Vanilla PIBT: {} agents on {}x{} grid", num_controlled, size, size);
    let result = solver.solve(&starts, &ends, &vec![], 200);

    match result {
        Ok(trajectories) => {
            println!("Success! Trajectory length: {}", trajectories.len());
            // Verify if goals were reached
            let last_step = trajectories.last().unwrap();
            let mut all_reached = true;
            for i in 0..num_controlled {
                let pos = last_step[i];
                let goal = (ends[i].0 as i64, ends[i].1 as i64);
                if pos != goal {
                    println!("Agent {} failed to reach goal {:?}, ended at {:?}", i, goal, pos);
                    all_reached = false;
                }
            }
            if all_reached {
                println!("All agents reached their goals.");
            }
        }
        Err(_) => println!("PIBT Failed to find a solution."),
    }
}