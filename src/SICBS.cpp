#include "SICBS.h"

Solution SICBS::run() {
  HLNode root;
  root.constraints = {};
  root.solution = getInitialSolution();
  root.cost = calculateCost(root.solution);
  findConflicts(root.solution, root.conflicts);

  open_list.push(root);
  while (!open_list.empty()) {
    auto curr_node = open_list.top();
    open_list.pop();

    if (curr_node.conflicts.empty()) {
      return curr_node.solution;
    }

    auto conflict = curr_node.conflicts[0];
    vector<int> agent_ids = {get<0>(conflict), get<1>(conflict)};
    for (const auto& agent_id : agent_ids) {
      auto new_node = curr_node;
      new_node.constraints.emplace_back(agent_id, env.radii[agent_id], get<2>(conflict));
      constraint_table.updateConstraints(new_node.constraints);
      new_node.solution[agent_id] = low_level_planners[agent_id].run();
      if (new_node.solution[agent_id].empty()) continue;
      new_node.cost = calculateCost(new_node.solution);
      findConflicts(new_node.solution, new_node.conflicts);
      open_list.push(new_node);
    }
  }
  cout << "No solution" << endl;
  return {};
}

Solution SICBS::getInitialSolution() {
  Solution solution;
  for (int agent_id = 0; agent_id < env.num_of_robots; ++agent_id) {
    auto path = low_level_planners[agent_id].run();
    solution.emplace_back(path);
  }
  return solution;
}

double SICBS::calculateCost(const Solution& solution) {
  double cost = 0.0;
  for (const auto& path : solution) {
    cost += get<1>(path.back());
  }
  return cost;
}

void SICBS::findConflicts(const Solution& solution, vector<Conflict>& conflicts) const {
  for (int agent1_id = 0; agent1_id < env.num_of_robots; ++agent1_id) {
    for (int agent2_id = agent1_id + 1; agent2_id < env.num_of_robots; ++agent2_id) {
      assert(agent1_id != agent2_id);
      assert(!solution[agent1_id].empty() && !solution[agent2_id].empty());

      auto [prev_point1, prev_time1] = solution[agent1_id][0];
      auto [next_point1, next_time1] = solution[agent1_id][1];
      auto [prev_point2, prev_time2] = solution[agent2_id][0];
      auto [next_point2, next_time2] = solution[agent2_id][1];

      const double min_path_time = min(get<1>(solution[agent1_id].back()), get<1>(solution[agent2_id].back()));

      bool is_safe = true;
      double collision_start_time = 0.0;

      const auto timesteps = static_cast<int>(floor(min_path_time / env.velocities[agent1_id])) + 1;

      for (int timestep = 0; timestep < timesteps; ++timestep) {
        // update prev_point1, next_point1, prev_point2, next_point2
        if (timestep >= next_time1) {
          prev_point1 = next_point1;
          prev_time1 = next_time1;
          next_point1 = get<0>(solution[agent1_id][min(static_cast<int>(solution[agent1_id].size()) - 1, timestep + 1)]);
          next_time1 = get<1>(solution[agent1_id][min(static_cast<int>(solution[agent1_id].size()) - 1, timestep + 1)]);
        }
        if (timestep >= next_time2) {
          prev_point2 = next_point2;
          prev_time2 = next_time2;
          next_point2 = get<0>(solution[agent2_id][min(static_cast<int>(solution[agent2_id].size()) - 1, timestep + 1)]);
          next_time2 = get<1>(solution[agent2_id][min(static_cast<int>(solution[agent2_id].size()) - 1, timestep + 1)]);
        }

        const auto agent1_theta =
            atan2(get<1>(next_point1) - get<1>(prev_point1), get<0>(next_point1) - get<0>(prev_point1));
        const auto agent2_theta =
            atan2(get<1>(next_point2) - get<1>(prev_point2), get<0>(next_point2) - get<0>(prev_point2));

        auto agent1_point = prev_point1;
        auto agent2_point = prev_point2;
        if (agent1_theta != 0.0) {
          agent1_point = make_tuple(get<0>(prev_point1) + env.velocities[agent1_id] * cos(agent1_theta) * timestep,
                                    get<1>(prev_point1) + env.velocities[agent1_id] * sin(agent1_theta) * timestep);
        }
        if (agent2_theta != 0.0) {
          agent2_point = make_tuple(get<0>(prev_point2) + env.velocities[agent2_id] * cos(agent2_theta) * timestep,
                                    get<1>(prev_point2) + env.velocities[agent2_id] * sin(agent2_theta) * timestep);
        }

        if (calculateDistance(agent1_point, agent2_point) < env.radii[agent1_id] + env.radii[agent2_id]) {
          if (is_safe) {
            is_safe = false;
            collision_start_time = timestep;
          }
        } else if (calculateDistance(agent1_point, agent2_point) >= env.radii[agent1_id] + env.radii[agent2_id] &
                   !is_safe) {
          is_safe = true;
          assert(collision_start_time < timestep);
          conflicts.emplace_back(agent1_id, agent2_id, make_tuple(collision_start_time, timestep));
          cout << "Conflict : "
               << "(Agent " << agent1_id << ", Agent " << agent2_id << "), "
               << "Interval : "
               << "[" << collision_start_time << ", " << timestep << ")" << endl;
        }
      }
    }
  }
}