use hetpibt::external_tracks_pibt::PiBTWithExternalTracks;

#[test]
fn test_external_tracks_avoidance() {
    // 5x5 grid
    let grid = vec![
        vec![0, 0, 0, 0, 0],
        vec![0, 0, 0, 0, 0],
        vec![0, 0, 0, 0, 0],
        vec![0, 0, 0, 0, 0],
        vec![0, 0, 0, 0, 0],
    ];

    let mut pibt = PiBTWithExternalTracks::init(grid);

    // Internal agent starts at (2, 0) and wants to go to (2, 4)
    let starts = vec![(2, 0)];
    let ends = vec![(2, 4)];

    // External agent is at (2, 2) for all time, blocking the direct path
    // But since it's a 5x5 grid, internal agent can go around.
    // Wait, it's still greedy. Let's make the external agent move across the path.
    let external_tracks = vec![
        vec![(0, 2), (1, 2), (2, 2), (3, 2), (4, 2)]
    ];

    let result = pibt.solve(&starts, &ends, &external_tracks, 20);

    assert!(result.is_ok());
    let tracks = result.unwrap();
    
    // Check for collisions with external agent at each time step
    for t in 0..tracks.len() {
        let internal_pos = tracks[t][0];
        let external_pos = if t < external_tracks[0].len() {
            (external_tracks[0][t].0 as i64, external_tracks[0][t].1 as i64)
        } else {
            (external_tracks[0].last().unwrap().0 as i64, external_tracks[0].last().unwrap().1 as i64)
        };
        assert_ne!(internal_pos, external_pos, "Collision with external agent at t={}", t);
    }
    
    // Check that it reached the goal
    assert_eq!(*tracks.last().unwrap().get(0).unwrap(), (2, 4));
}

#[test]
fn test_external_tracks_push() {
    // 3x3 grid
    let grid = vec![
        vec![0, 0, 0],
        vec![0, 0, 0],
        vec![0, 0, 0],
    ];

    let mut pibt = PiBTWithExternalTracks::init(grid);

    // Agent A (internal, lower priority) starts at (1, 1)
    // Agent B (internal, higher priority) starts at (1, 0) and wants to go to (1, 2)
    // External Agent C (highest priority) starts at (0, 0)
    
    let starts = vec![(1, 1), (1, 0)];
    let ends = vec![(1, 1), (1, 2)];
    
    // External agent is at (0, 2)
    let external_tracks = vec![
        vec![(0, 2), (0, 2), (0, 2)]
    ];

    // Priorities: Agent 1 (dist 2 to (1,2)) vs Agent 0 (dist 0 to (1,1))
    // Agent 1 has higher priority (longer distance).
    // So Agent 1 will push Agent 0.
    
    let result = pibt.solve(&starts, &ends, &external_tracks, 10);

    assert!(result.is_ok());
    let tracks = result.unwrap();
    
    // Agent 1 should reach (1, 2)
    assert_eq!(tracks.last().unwrap()[1], (1, 2));
}
