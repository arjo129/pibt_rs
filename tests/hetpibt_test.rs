
use hetpibt::{HetPiBT, HeterogenousAgent};

#[test]
fn test_swap_case() {
    let base_obstacles = vec![vec![false; 20]; 20];
    let graph_scale = vec![1.0, 2.0];
    let graph_bound = vec![(20, 20), (10, 10)];
    let agents = vec![
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
    ];

    let mut het_pibt = HetPiBT::init_solver(
        &base_obstacles,
        graph_scale.clone(),
        graph_bound,
        agents.clone(),
    );
    let result = het_pibt.solve(5000);
    assert!(result.is_some());
}
