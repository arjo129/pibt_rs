use hetpibt::external_tracks_pibt::PiBTWithExternalTracks;
use rand::Rng;

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

fn main() {
    let width = 8;
    let height = 8;
    let grid = vec![vec![0; width]; height];

    let mut solver = PiBTWithExternalTracks::init(grid);

    let starts = vec![(0, 0), (0, 7)];
    let ends = vec![(7, 7), (7, 0)];

    let mut base_external_tracks = vec![];
    let mut track0 = vec![];
    for y in 0..8 { track0.push((4, y)); }
    base_external_tracks.push(track0);

    let mut track1 = vec![];
    for x in 0..8 { track1.push((x, 4)); }
    base_external_tracks.push(track1);

    let mut track2 = vec![];
    for y in (0..8).rev() { track2.push((2, y)); }
    track2.push((1, 0));
    track2.push((0, 0));
    base_external_tracks.push(track2);

    let delay_probability = 0.8;

    for i in 0..1000 {
        if i % 100 == 0 {
            println!("Iteration {}", i);
        }
        let _delayed_external_tracks: Vec<Vec<(usize, usize)>> = base_external_tracks
            .clone()
            .into_iter()
            .map(|t| add_random_delays(t, delay_probability))
            .collect();
        
        let max_time = 500;
        let _result = solver.solve(&starts, &ends, &base_external_tracks, max_time);
    }
    println!("Finished 1000 iterations successfully.");
}
