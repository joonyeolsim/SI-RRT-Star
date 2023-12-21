#include "SICBS.h"

Solution SICBS::run() {
  HLNode root;
  root.constraints = {};
  root.solution = getInitialSolution(env.start_points, env.goal_points, env.radii, env.obstacles);
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
      new_node.solution[agent_id] = findPath(agent_id);
      if (new_node.solution.empty()) continue;
      new_node.cost = calculateCost(new_node.solution);
      open_list.push(new_node);
    }
  }
}
