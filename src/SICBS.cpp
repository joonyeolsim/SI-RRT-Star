#include "SICBS.h"

Solution SICBS::run() {
  HLNode root;
  root.constraints = {};
  root.solution = getInitialSolution();
  root.cost = 0.0;

  open_list.push(root);
  while (!open_list.empty()) {
    auto curr_node = open_list.top();
    open_list.pop();

    vector<Conflict> conflicts;
    findConflicts(curr_node.solution, conflicts);
    if (conflicts.empty()) {
      return curr_node.solution;
    }

    auto conflict = conflicts[0];
    vector<int> agent_ids = {get<0>(conflict), get<1>(conflict)};
    for (const auto& agent_id : agent_ids) {
      auto new_node = curr_node;
      new_node.constraints.emplace_back(agent_id, env.radii[agent_id], get<2>(conflict));
      new_node.solution[agent_id] = low_level_planners[agent_id].run();
      if (new_node.solution.empty()) continue;
      new_node.cost = calculateCost(new_node.solution);
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

double SICBS::calculateCost(const Solution& solution) const {
  double cost = 0.0;
  for (const auto& path : solution) {
    cost += get<1>(path.back());
  }
  return cost;
}