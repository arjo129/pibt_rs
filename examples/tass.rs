use std::collections::HashMap;

use macroquad::prelude::*;

struct TreeNode {
    position: (f32, f32),
    neighbours: Vec<usize>,
}

fn draw_nodes(nodes: &Vec<TreeNode>) {
    for node in nodes {
        for &neighbour in &node.neighbours {
            let (end_x, end_y) = nodes[neighbour].position;
            let (start_x, start_y) = node.position;
            let start_x = start_x * 40.0;
            let start_y = start_y * 40.0;
            let end_x = end_x * 40.0;
            let end_y = end_y * 40.0;
            draw_line(start_x, start_y, end_x, end_y, 2.0, BLACK);
        }
    }

    for node in nodes {
        let (start_x, start_y) = node.position;
        let start_x = start_x * 40.0;
        let start_y = start_y * 40.0;
        draw_circle(start_x, start_y, 18.0, BLACK);
        draw_circle(start_x, start_y, 16.0, WHITE);
    }
}

fn draw_agents(nodes: &Vec<TreeNode>, agent: &Vec<usize>) {
    for (a_id, position) in agent.iter().enumerate() {
        let node = &nodes[*position];
        draw_text(
            &format!("{}", a_id).as_str(),
            node.position.0 * 40.0,
            node.position.1 * 40.0,
            20.0,
            BLACK,
        );
    }
}

fn potential_moves(map: &Vec<TreeNode>, agent_state: &Vec<usize>) -> Vec<Vec<(usize, usize)>> {
    let mut pos_to_agent = vec![None; map.len()];
    for (id, &agent_pos) in agent_state.iter().enumerate() {
        pos_to_agent[agent_pos] = Some(id);
    }

    let mut moves = vec![];
    for (agent, &agent_pos) in agent_state.iter().enumerate() {
        for &n in &map[agent_pos].neighbours {
            if pos_to_agent[n].is_none() {
                moves.push(vec![(agent, n)]);
            }
        }
    }

    return moves;
}
/*
fn apply_moves(map: &Vec<TreeNode>, agent_state: &Vec<usize>, moves: &Vec<(usize, usize)>) {
    for movement in movements {}
    for agent in agent_state {}
}*/

fn solve(map: &Vec<TreeNode>, start: &Vec<usize>, goal: &Vec<usize>) {}

#[macroquad::main("Movement Visualization")]
async fn main() {
    let tree = vec![
        TreeNode {
            //0
            position: (1.0, 1.0),
            neighbours: vec![1],
        },
        TreeNode {
            //1
            position: (2.0, 1.0),
            neighbours: vec![0, 2],
        },
        TreeNode {
            //2
            position: (3.0, 1.0),
            neighbours: vec![1, 3, 4],
        },
        TreeNode {
            //3
            position: (4.0, 1.0),
            neighbours: vec![2],
        },
        TreeNode {
            //4
            position: (3.0, 2.0),
            neighbours: vec![2, 5],
        },
        TreeNode {
            //5
            position: (3.0, 3.0),
            neighbours: vec![4],
        },
    ];

    loop {
        clear_background(WHITE);
        draw_nodes(&tree);
        draw_agents(&tree, &vec![0, 1, 3]);
        next_frame().await;
    }
}
