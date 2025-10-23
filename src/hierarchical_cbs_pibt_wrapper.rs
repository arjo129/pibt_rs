use crate::HeterogenousAgent;
use crate::pibt_with_constraints::hierarchical_cbs_pibt;

pub struct HierarchicalCbsPibtWrapper {
    pub trajectories: Vec<Vec<Vec<(i64, i64)>>>,
    agent_to_graph: Vec<(usize, usize)>,
}

impl HierarchicalCbsPibtWrapper {
    pub fn init_solver(
        _base_obstacles: &Vec<Vec<bool>>,
        graph_scale: Vec<f32>,
        grid_bounds: Vec<(usize, usize)>,
        agents: Vec<HeterogenousAgent>,
    ) -> Self {
        let starts: Vec<(usize, usize, usize)> = agents
            .iter()
            .map(|agent| (agent.graph_id, agent.start.0, agent.start.1))
            .collect();
        let ends: Vec<(usize, usize)> = agents.iter().map(|agent| agent.end).collect();

        let trajectories = hierarchical_cbs_pibt(starts, ends, grid_bounds, graph_scale);

        let mut agent_to_graph = vec![(0, 0); agents.len()];
        let mut agent_in_graph_count = vec![0; 5];
        for (agent_id, agent) in agents.iter().enumerate() {
            agent_to_graph[agent_id] = (agent.graph_id, agent_in_graph_count[agent.graph_id]);
            agent_in_graph_count[agent.graph_id] += 1;
        }

        Self {
            trajectories,
            agent_to_graph,
        }
    }

    pub fn get_trajectories(&self, time_step: usize) -> Vec<Option<(usize, usize, usize)>> {
        let mut positions = vec![None; self.agent_to_graph.len()];
        for (agent_id, &(graph_id, agent_in_graph_id)) in self.agent_to_graph.iter().enumerate() {
            if time_step < self.trajectories[graph_id].len() {
                let (x, y) = self.trajectories[graph_id][time_step][agent_in_graph_id];
                positions[agent_id] = Some((graph_id, x as usize, y as usize));
            }
        }
        positions
    }
}
