#include "SIRRT.h"

Path SIRRT::run() {
  release();
  SafeIntervalTable safe_interval_table(env);

  // initialize start and goal safe intervals
  vector<Interval> start_safe_intervals;
  constraint_table.getSafeIntervalTableConstraint(agent_id, start_point, env.radii[agent_id], start_safe_intervals);
  assert(!start_safe_intervals.empty());
  safe_interval_table.table[start_point] = start_safe_intervals;

  vector<Interval> goal_safe_intervals;
  constraint_table.getSafeIntervalTableConstraint(agent_id, goal_point, env.radii[agent_id], goal_safe_intervals);
  assert(!goal_safe_intervals.empty());
  safe_interval_table.table[goal_point] = goal_safe_intervals;

  // initialize start node
  const auto start_node = make_shared<LLNode>(start_point);
  start_node->earliest_arrival_time = 0;
  start_node->intervals = {{0, min(numeric_limits<double>::max(), get<1>(start_safe_intervals[0]))}};
  nodes.push_back(start_node);

  // the earliest timestep that the agent can hold its goal location.
  double earliest_goal_arrival_time = 0.0;
  for (auto& interval : goal_safe_intervals) {
    earliest_goal_arrival_time = max(earliest_goal_arrival_time, get<0>(interval));
  }

  int iteration = env.iterations[agent_id];
  while (iteration--) {
    Point random_point = generateRandomPoint();
    const shared_ptr<LLNode> nearest_node = getNearestNode(random_point);
    auto new_node = steer(nearest_node, random_point, safe_interval_table);
    if (new_node == nullptr) {
      continue;
    }

    // SIRRT*
    vector<shared_ptr<LLNode>> neighbors;
    getNeighbors(new_node, neighbors);
    assert(!neighbors.empty());
    const bool success = chooseParent(new_node, neighbors, safe_interval_table);
    if (!success) {
      continue;
    }
    assert(new_node->parent.lock() != nullptr);
    rewire(new_node, neighbors, safe_interval_table);

    // check goal
    if (calculateDistance(new_node->point, goal_point) < env.threshold) {
      // SIRRTPP
      // if (constraint_table.targetConstrained(agent_id, new_node->point, new_node->earliest_arrival_time,
      //                                        env.radii[agent_id]))
      //   continue;

      // SICBS
      // goal_node = new_node;
      // cout << "Goal node updated: " << goal_node->earliest_arrival_time << endl;
      // break;
      for (auto& interval : new_node->intervals) {
        if (get<0>(interval) < earliest_goal_arrival_time) continue;
        if (goal_node == nullptr || get<0>(interval) < goal_node->earliest_arrival_time) {
          goal_node = new_node;
          goal_node->earliest_arrival_time = get<0>(interval);
        }
      }
    } else {
      nodes.push_back(new_node);
    }
  }

  if (goal_node != nullptr) {
    nodes.push_back(goal_node);
    path = updatePath(goal_node, 0);
    assert(calculateDistance(get<0>(path.front()), start_point) < env.threshold);
    assert(calculateDistance(get<0>(path.back()), goal_point) < env.threshold);
    // assert velocity always be 1.0m/s
    for (int j = 0; j < path.size() - 1; ++j) {
      const double distance = calculateDistance(get<0>(path[j]), get<0>(path[j + 1]));
      const double time_diff = get<1>(path[j + 1]) - get<1>(path[j]);
      assert(distance / time_diff < env.velocities[agent_id] + env.threshold);
    }
    cout << "Path found!" << endl;
    return path;
  }

  cout << "No solution found!" << endl;
  return path;
}

Point SIRRT::generateRandomPoint() {
  // Fixed seed for consistent results across runs
  if (dis_100(env.gen) < env.goal_sample_rates[agent_id]) {
    return make_tuple(get<0>(goal_point), get<1>(goal_point));
  } else {
    return make_tuple(dis_width(env.gen), dis_height(env.gen));
  }
}

shared_ptr<LLNode> SIRRT::getNearestNode(const Point& point) const {
  double min_distance = numeric_limits<double>::max();
  shared_ptr<LLNode> nearest_node;
  for (const auto& node : nodes) {
    const double distance = calculateDistance(node->point, point);
    if (distance < min_distance) {
      min_distance = distance;
      nearest_node = node;
    }
  }
  return nearest_node;
}

shared_ptr<LLNode> SIRRT::steer(const shared_ptr<LLNode>& from_node, const Point& random_point,
                                SafeIntervalTable& safe_interval_table) const {
  const double expand_distance =
      min(env.max_expand_distances[agent_id], calculateDistance(from_node->point, random_point));
  const double theta =
      atan2(get<1>(random_point) - get<1>(from_node->point), get<0>(random_point) - get<0>(from_node->point));
  const double expand_time = expand_distance / env.velocities[agent_id];
  const Point to_point = make_tuple(get<0>(from_node->point) + env.velocities[agent_id] * cos(theta) * expand_time,
                                    get<1>(from_node->point) + env.velocities[agent_id] * sin(theta) * expand_time);

  if (constraint_table.obstacleConstrained(agent_id, from_node->point, to_point, env.radii[agent_id])) return nullptr;
  auto new_node = make_shared<LLNode>(to_point);

  vector<Interval> safe_intervals;
  constraint_table.getSafeIntervalTableConstraint(agent_id, to_point, env.radii[agent_id], safe_intervals);
  if (safe_intervals.empty()) return nullptr;
  safe_interval_table.table[to_point] = safe_intervals;

  // SIRRT
  // for (int i = 0; i < from_node->intervals.size(); ++i) {
  //   const double lower_bound = get<0>(from_node->intervals[i]) + expand_time;
  //   const double upper_bound = get<1>(from_node->intervals[i]) + expand_time;
  //   assert(lower_bound > 0);
  //   assert(lower_bound < upper_bound);
  //   for (auto& safe_interval : safe_intervals) {
  //     if (lower_bound >= get<1>(safe_interval)) continue;
  //     if (upper_bound <= get<0>(safe_interval)) break;
  //
  //     // check move constraint
  //     // TODO : from time, to time should be real time
  //     const double to_time = max(get<0>(safe_interval), lower_bound);
  //     const double from_time = to_time - expand_time;
  //     if (constraint_table.constrained(agent_id, from_node->point, to_point, from_time, to_time,
  //     env.radii[agent_id]))
  //       continue;
  //     new_node->earliest_arrival_time = to_time;
  //     assert(to_time < min(get<1>(safe_interval), upper_bound));
  //     new_node->intervals.emplace_back(to_time, min(get<1>(safe_interval), upper_bound));
  //     new_node->parent_interval_indicies.emplace_back(i);
  //   }
  // }
  // if (new_node->intervals.empty()) {
  //   return nullptr;
  // }
  // new_node->parent = from_node;

  return new_node;
}

Path SIRRT::updatePath(const shared_ptr<LLNode>& goal_node, const int interval_index) const {
  Path path;
  shared_ptr<LLNode> curr_node = goal_node;
  int current_interval_index = interval_index;
  while (curr_node->parent.lock() != nullptr) {
    const auto prev_node = curr_node->parent.lock();
    const auto prev_time = get<0>(prev_node->intervals[curr_node->parent_interval_indicies[current_interval_index]]);
    const auto curr_time = get<0>(curr_node->intervals[current_interval_index]);
    assert(prev_time < curr_time);

    const auto expand_time = calculateDistance(prev_node->point, curr_node->point) / env.velocities[agent_id];
    path.emplace_back(curr_node->point, curr_time);
    if (prev_time + expand_time + env.threshold < curr_time) {
      path.emplace_back(prev_node->point, curr_time - expand_time);
      cout << "path is not continuous" << endl;
    }
    if (curr_node && !curr_node->parent_interval_indicies.empty()) {
      current_interval_index = curr_node->parent_interval_indicies[current_interval_index];
    }
    curr_node = curr_node->parent.lock();
  }
  path.emplace_back(curr_node->point, 0);
  reverse(path.begin(), path.end());

  return path;
}

void SIRRT::getNeighbors(const shared_ptr<LLNode>& new_node, vector<shared_ptr<LLNode>>& neighbors) const {
  assert(!nodes.empty());
  assert(neighbors.empty());
  // const double connection_radius =
  //     min(env.max_expand_distances[agent_id], 50 * sqrt(log(nodes.size()) / nodes.size())) + env.threshold;
  const double connection_radius = env.max_expand_distances[agent_id] + env.threshold;
  for (const auto& node : nodes) {
    const double distance = calculateDistance(node->point, new_node->point);
    if (distance < connection_radius) {
      neighbors.emplace_back(node);
    }
  }
}

bool SIRRT::chooseParent(const shared_ptr<LLNode>& new_node, const vector<shared_ptr<LLNode>>& neighbors,
                         SafeIntervalTable& safe_interval_table) {
  assert(!neighbors.empty());
  assert(new_node->parent.lock() == nullptr);

  shared_ptr<LLNode> parent_node;

  double earliest_arrival_time = numeric_limits<double>::max();
  vector<Interval> intervals;
  vector<int> parent_interval_indicies;

  for (const auto& neighbor : neighbors) {
    if (constraint_table.obstacleConstrained(agent_id, neighbor->point, new_node->point, env.radii[agent_id])) continue;
    const double expand_time = calculateDistance(neighbor->point, new_node->point) / env.velocities[agent_id];

    for (int i = 0; i < neighbor->intervals.size(); ++i) {
      const double lower_bound = get<0>(neighbor->intervals[i]) + expand_time;
      const double upper_bound = get<1>(neighbor->intervals[i]) + expand_time;
      assert(lower_bound > 0);
      assert(lower_bound < upper_bound);

      auto safe_intervals = safe_interval_table.table[new_node->point];
      for (auto& safe_interval : safe_intervals) {
        if (lower_bound + env.threshold >= get<1>(safe_interval)) continue;
        if (upper_bound - env.threshold <= get<0>(safe_interval)) break;

        // check move constraint
        const double to_time = max(get<0>(safe_interval), lower_bound);
        const double from_time = to_time - expand_time;
        if (constraint_table.constrained(agent_id, neighbor->point, new_node->point, from_time, to_time,
                                         env.radii[agent_id]))
          continue;
        assert(to_time < min(get<1>(safe_interval), upper_bound));
        if (to_time < earliest_arrival_time) {
          earliest_arrival_time = to_time;
          parent_node = neighbor;
          intervals.clear();
          parent_interval_indicies.clear();
        }

        if (parent_node == neighbor) {
          intervals.emplace_back(to_time, min(get<1>(safe_interval), upper_bound));
          assert(get<0>(intervals.back()) + env.threshold < get<1>(intervals.back()));
          parent_interval_indicies.emplace_back(i);
        }
      }
    }
  }

  // it fails to find parent because of constraint
  if (parent_node == nullptr) {
    return false;
  }
  new_node->intervals = intervals;
  new_node->parent_interval_indicies = parent_interval_indicies;
  new_node->parent = parent_node;
  parent_node->children.emplace_back(new_node);
  assert(new_node->earliest_arrival_time - parent_node->earliest_arrival_time >= 0);
  new_node->earliest_arrival_time = earliest_arrival_time;
  return true;
}

void SIRRT::rewire(const shared_ptr<LLNode>& new_node, const vector<shared_ptr<LLNode>>& neighbors,
                   SafeIntervalTable& safe_interval_table) {
  assert(!neighbors.empty());
  for (auto& neighbor : neighbors) {
    // Skip if neighbor is the parent of new_node
    if (neighbor == new_node->parent.lock()) continue;

    if (constraint_table.obstacleConstrained(agent_id, new_node->point, neighbor->point, env.radii[agent_id])) continue;
    const double expand_time = calculateDistance(new_node->point, neighbor->point) / env.velocities[agent_id];

    vector<Interval> intervals;
    vector<int> parent_interval_indices;
    double earliest_arrival_time = numeric_limits<double>::max();

    for (int i = 0; i < new_node->intervals.size(); ++i) {
      const double lower_bound = get<0>(new_node->intervals[i]) + expand_time;
      const double upper_bound = get<1>(new_node->intervals[i]) + expand_time;
      assert(lower_bound > 0);
      assert(lower_bound < upper_bound);

      auto safe_intervals = safe_interval_table.table[neighbor->point];
      for (auto& safe_interval : safe_intervals) {
        if (lower_bound + env.threshold >= get<1>(safe_interval)) continue;
        if (upper_bound - env.threshold <= get<0>(safe_interval)) break;

        // check move constraint
        const double to_time = max(get<0>(safe_interval), lower_bound);
        const double from_time = to_time - expand_time;
        if (constraint_table.constrained(agent_id, new_node->point, neighbor->point, from_time, to_time,
                                         env.radii[agent_id]))
          continue;
        if (to_time < neighbor->earliest_arrival_time) {
          earliest_arrival_time = to_time;
        }
        assert(to_time < min(get<1>(safe_interval), upper_bound));

        intervals.emplace_back(to_time, min(get<1>(safe_interval), upper_bound));
        assert(get<0>(intervals.back()) + env.threshold < get<1>(intervals.back()));
        parent_interval_indices.emplace_back(i);
      }
    }

    if (earliest_arrival_time < neighbor->earliest_arrival_time) {
      neighbor->earliest_arrival_time = earliest_arrival_time;
      neighbor->intervals = intervals;
      neighbor->parent_interval_indicies = parent_interval_indices;
      // update parent
      neighbor->parent.lock()->children.erase(
          remove(neighbor->parent.lock()->children.begin(), neighbor->parent.lock()->children.end(), neighbor),
          neighbor->parent.lock()->children.end());
      neighbor->parent = new_node;
      new_node->children.emplace_back(neighbor);
      propagateCostToSuccessor(neighbor, safe_interval_table);
      assert(neighbor->earliest_arrival_time - new_node->earliest_arrival_time >= 0);
    }
  }
}

void SIRRT::propagateCostToSuccessor(const shared_ptr<LLNode>& node, SafeIntervalTable& safe_interval_table) {
  assert(node->earliest_arrival_time >= 0);
  for (const auto& child : node->children) {
    child->intervals.clear();
    child->parent_interval_indicies.clear();
    const double expand_time = calculateDistance(node->point, child->point) / env.velocities[agent_id];

    for (int i = 0; i < node->intervals.size(); ++i) {
      const double lower_bound = get<0>(node->intervals[i]) + expand_time;
      const double upper_bound = get<1>(node->intervals[i]) + expand_time;
      assert(lower_bound > 0);
      assert(lower_bound < upper_bound);
      auto safe_intervals = safe_interval_table.table[child->point];
      for (auto& safe_interval : safe_intervals) {
        if (lower_bound + env.threshold >= get<1>(safe_interval)) continue;
        if (upper_bound - env.threshold <= get<0>(safe_interval)) break;

        const double to_time = max(get<0>(safe_interval), lower_bound);
        const double from_time = to_time - expand_time;
        if (constraint_table.constrained(agent_id, node->point, child->point, from_time, to_time, env.radii[agent_id]))
          continue;
        assert(to_time < min(get<1>(safe_interval), upper_bound));

        child->earliest_arrival_time = min(child->earliest_arrival_time, to_time);
        child->intervals.emplace_back(to_time, min(get<1>(safe_interval), upper_bound));
        assert(get<0>(child->intervals.back()) + env.threshold < get<1>(child->intervals.back()));
        child->parent_interval_indicies.emplace_back(i);
      }
    }
    propagateCostToSuccessor(child, safe_interval_table);
  }
}

void SIRRT::release() {
  nodes.clear();
  path.clear();
  goal_node = nullptr;
}
