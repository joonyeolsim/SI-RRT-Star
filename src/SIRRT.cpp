#include "SIRRT.h"

Path SIRRT::run() {
  release();
  SafeIntervalTable safe_interval_table(env);

  // initialize start and goal safe intervals
  vector<Interval> start_safe_intervals;
  constraint_table.getSafeIntervalTable(agent_id, start_point, env.radii[agent_id], start_safe_intervals);
  assert(!start_safe_intervals.empty());
  safe_interval_table.table[start_point] = start_safe_intervals;

  vector<Interval> goal_safe_intervals;
  constraint_table.getSafeIntervalTable(agent_id, goal_point, env.radii[agent_id], goal_safe_intervals);
  assert(!goal_safe_intervals.empty());
  safe_interval_table.table[goal_point] = goal_safe_intervals;

  // initialize start node
  const auto start_node = make_shared<LLNode>(start_point);
  nodes.push_back(start_node);

  // the earliest timestep that the agent can hold its goal location.
  double earliest_goal_arrival_time = 0.0;
  for (auto& interval : goal_safe_intervals) {
    earliest_goal_arrival_time = max(earliest_goal_arrival_time, get<0>(interval));
  }

  double best_earliest_arrival_time = numeric_limits<double>::infinity();
  int best_num_of_soft_conflicts = numeric_limits<int>::max();
  int iteration = env.iterations[agent_id];
  while (iteration--) {
    Point random_point = generateRandomPoint();
    const shared_ptr<LLNode> nearest_node = getNearestNode(random_point);
    Point new_point = steer(nearest_node, random_point, safe_interval_table);
    if (new_point == make_tuple(-1.0, -1.0)) {
      continue;
    }

    // SIRRT*
    vector<shared_ptr<LLNode>> neighbors;
    getNeighbors(new_point, neighbors);
    assert(!neighbors.empty());
    auto new_nodes = chooseParent(new_point, neighbors, safe_interval_table);
    if (new_nodes.empty()) {
      continue;
    }
    // rewire(new_node, neighbors, safe_interval_table);

    // check goal
    for (auto& new_node : new_nodes) {
      if (calculateDistance(new_node->point, goal_point) < env.epsilon) {
        if (get<0>(new_node->interval) < earliest_goal_arrival_time) continue;
        // if (goal_node == nullptr || new_node->soft_conflicts[i] < best_min_soft_conflict) {
        //   goal_node = new_node;
        //   best_earliest_arrival_time = get<0>(new_node->intervals[i]);
        //   best_min_soft_conflict = new_node->soft_conflicts[i];
        //   best_interval_index = i;
        // }
        if (goal_node == nullptr || get<0>(new_node->interval) < best_earliest_arrival_time) {
          goal_node = new_node;
          best_earliest_arrival_time = get<0>(new_node->interval);
          best_num_of_soft_conflicts = new_node->num_of_soft_conflicts;
          // TODO: interval이 rewire로 사라질 경우.
        }
      } else {
        nodes.push_back(new_node);
      }
    }
  }

  if (goal_node != nullptr) {
    nodes.push_back(goal_node);
    path = updatePath(goal_node);
    assert(calculateDistance(get<0>(path.front()), start_point) < env.epsilon);
    assert(calculateDistance(get<0>(path.back()), goal_point) < env.epsilon);
    // assert velocity always be 1.0m/s
    for (int j = 0; j < path.size() - 1; ++j) {
      const double distance = calculateDistance(get<0>(path[j]), get<0>(path[j + 1]));
      const double time_diff = get<1>(path[j + 1]) - get<1>(path[j]);
      assert(distance / time_diff < env.velocities[agent_id] + env.epsilon);
    }
    // cout << "Path found!" << endl;
    return path;
  }

  cout << "No solution found!" << endl;
  return path;
}

Point SIRRT::generateRandomPoint() {
  const bool selectGoalPoint = dis_100(env.gen) < env.goal_sample_rates[agent_id];

  return selectGoalPoint ? make_tuple(get<0>(goal_point), get<1>(goal_point))
                         : make_tuple(dis_width(env.gen), dis_height(env.gen));
}

shared_ptr<LLNode> SIRRT::getNearestNode(const Point& point) const {
  if (nodes.empty()) {
    return nullptr;
  }

  double min_distance = numeric_limits<double>::max();
  shared_ptr<LLNode> nearest_node = nullptr;

  for (const auto& node : nodes) {
    const double distance = calculateDistance(node->point, point);
    if (distance < min_distance) {
      min_distance = distance;
      nearest_node = node;
    }
  }

  return nearest_node;
}

Point SIRRT::steer(const shared_ptr<LLNode>& from_node, const Point& random_point,
                   SafeIntervalTable& safe_interval_table) const {
  const double expand_distance =
      min(env.max_expand_distances[agent_id], calculateDistance(from_node->point, random_point));
  const double theta =
      atan2(get<1>(random_point) - get<1>(from_node->point), get<0>(random_point) - get<0>(from_node->point));
  const double expand_time = expand_distance / env.velocities[agent_id];
  const Point to_point = make_tuple(get<0>(from_node->point) + env.velocities[agent_id] * cos(theta) * expand_time,
                                    get<1>(from_node->point) + env.velocities[agent_id] * sin(theta) * expand_time);

  if (constraint_table.obstacleConstrained(agent_id, from_node->point, to_point, env.radii[agent_id])) {
    return make_tuple(-1, -1);
  }

  vector<Interval> safe_intervals;
  constraint_table.getSafeIntervalTable(agent_id, to_point, env.radii[agent_id], safe_intervals);
  if (safe_intervals.empty()) {
    return make_tuple(-1.0, -1.0);
  }
  safe_interval_table.table[to_point] = safe_intervals;

  return to_point;
}

Path SIRRT::updatePath(const shared_ptr<LLNode>& goal_node) const {
  Path path;
  shared_ptr<LLNode> curr_node = goal_node;
  while (curr_node->parent.lock() != nullptr) {
    const auto prev_node = curr_node->parent.lock();
    const auto prev_time = get<0>(prev_node->interval);
    const auto curr_time = get<0>(curr_node->interval);
    assert(prev_time < curr_time);

    const auto expand_time = calculateDistance(prev_node->point, curr_node->point) / env.velocities[agent_id];
    path.emplace_back(curr_node->point, curr_time);
    if (prev_time + expand_time + env.epsilon < curr_time) {
      path.emplace_back(prev_node->point, curr_time - expand_time);
      cout << "path is not continuous" << endl;
    }
    curr_node = curr_node->parent.lock();
  }
  path.emplace_back(curr_node->point, 0);
  reverse(path.begin(), path.end());

  return path;
}

void SIRRT::getNeighbors(Point point, vector<shared_ptr<LLNode>>& neighbors) const {
  assert(!nodes.empty());
  assert(neighbors.empty());
  // const double connection_radius =
  //     min(env.max_expand_distances[agent_id], 50 * sqrt(log(nodes.size()) / nodes.size())) + env.threshold;
  const double connection_radius = env.max_expand_distances[agent_id] + env.epsilon;
  for (const auto& node : nodes) {
    const double distance = calculateDistance(node->point, point);
    if (distance < connection_radius) {
      neighbors.emplace_back(node);
    }
  }
}

vector<shared_ptr<LLNode>> SIRRT::chooseParent(const Point& new_point, const vector<shared_ptr<LLNode>>& neighbors,
                                               SafeIntervalTable& safe_interval_table) const {
  assert(!neighbors.empty());

  shared_ptr<LLNode> best_parent = nullptr;
  double best_earliest_arrival_time = numeric_limits<double>::infinity();
  double best_num_of_soft_conflicts = numeric_limits<int>::max();
  vector<shared_ptr<LLNode>> best_new_nodes;

  // 이웃 노드들을 순회하면서 가장 낮은 earliest arrival time을 가지는 이웃을 찾는다.
  for (const auto& neighbor : neighbors) {
    // 만약 이웃 노드로부터 새로운 노드로 갈 때 정적인 장애물과 충돌한다면 continue
    if (constraint_table.obstacleConstrained(agent_id, neighbor->point, new_point, env.radii[agent_id])) continue;

    // 이웃 노드로부터 새로운 노드로 갈 때 이동하는 시간
    const double expand_time = calculateDistance(neighbor->point, new_point) / env.velocities[agent_id];

    // 이웃 노드의 safe interval들을 순회하면서
    // 이웃 노드로부터 새로운 노드로 갈 때의 lower bound와 upper bound
    const double lower_bound = get<0>(neighbor->interval) + expand_time;
    const double upper_bound = get<1>(neighbor->interval) + expand_time;
    assert(lower_bound > 0);
    assert(lower_bound < upper_bound);

    vector<shared_ptr<LLNode>> new_nodes;
    // new node의 safe interval들을 순회하면서
    auto safe_intervals = safe_interval_table.table[new_point];
    for (auto& safe_interval : safe_intervals) {
      // lower bound와 upper bound가 safe interval과 겹치지 않는다면 continue
      if (lower_bound >= get<1>(safe_interval)) continue;
      if (upper_bound <= get<0>(safe_interval)) break;

      const double earliest_arrival_time = constraint_table.getEarliestArrivalTime(
          agent_id, neighbor->point, new_point, expand_time, max(get<0>(safe_interval), lower_bound),
          min(get<1>(safe_interval), upper_bound), env.radii[agent_id]);
      // there is no way to reach new node in this safe interval
      if (earliest_arrival_time < 0.0) continue;

      shared_ptr<LLNode> new_node = make_shared<LLNode>(new_point);
      new_node->interval = make_tuple(earliest_arrival_time, get<1>(safe_interval));
      new_node->num_of_soft_conflicts = neighbor->num_of_soft_conflicts;
      new_nodes.emplace_back(new_node);

      if (earliest_arrival_time < best_earliest_arrival_time) {
        best_parent = neighbor;
        best_earliest_arrival_time = earliest_arrival_time;
        // TODO: This value should be updated.
        best_num_of_soft_conflicts = neighbor->num_of_soft_conflicts;
        best_new_nodes = move(new_nodes);
      }

      // new node로의 이동이 soft constraint를 위반한다면 soft_conflict를 1 증가시킨다.
      // const double earliest_arrival_time_soft = constraint_table.getEarliestArrivalTimeSoft(
      //     agent_id, neighbor->point, new_node->point, expand_time, earliest_arrival_time,
      //     min(get<1>(safe_interval), upper_bound), env.radii[agent_id]);
      // if (earliest_arrival_time == earliest_arrival_time_soft) {
      //   // no collision
      //   candidate_interval = make_tuple(earliest_arrival_time, get<1>(safe_interval));
      //   candidate_num_of_soft_conflicts = neighbor->num_of_soft_conflicts;
      // } else if (earliest_arrival_time_soft < 0.0) {
      //   // all collision
      //   candidate_interval = make_tuple(earliest_arrival_time, get<1>(safe_interval));
      //   candidate_num_of_soft_conflicts = neighbor->num_of_soft_conflicts + 1;
      // } else {
      //   // collision
      //   candidate_interval = make_tuple(earliest_arrival_time, earliest_arrival_time_soft);
      //   candidate_num_of_soft_conflicts = neighbor->num_of_soft_conflicts + 1;
      //
      //   // no collision
      //   candidate_interval = make_tuple(earliest_arrival_time_soft, get<1>(safe_interval));
      //   candidate_num_of_soft_conflicts = neighbor->num_of_soft_conflicts;
      // }
    }
  }

  // 만약 parent node가 nullptr이라면 그 어떠한 이웃 노드로부터도 새로운 노드로 갈 수 없다.
  if (best_parent == nullptr) {
    return {};
  }

  for (auto& new_node : best_new_nodes) {
    new_node->parent = best_parent;
    best_parent->children.emplace_back(new_node);
  }
  return best_new_nodes;
}

// void SIRRT::rewire(const shared_ptr<LLNode>& new_node, const vector<shared_ptr<LLNode>>& neighbors,
//                    SafeIntervalTable& safe_interval_table) {
//   assert(!neighbors.empty());
//   for (auto& neighbor : neighbors) {
//     // Skip if neighbor is the parent of new_node
//     if (neighbor == new_node->parent.lock()) continue;
//     if (constraint_table.obstacleConstrained(agent_id, new_node->point, neighbor->point, env.radii[agent_id]))
//     continue;
//
//     const double expand_time = calculateDistance(new_node->point, neighbor->point) / env.velocities[agent_id];
//     vector<int> candidate_soft_conflicts;
//     vector<Interval> candidate_intervals;
//     vector<int> candidate_parent_interval_indices;
//
//     for (int i = 0; i < new_node->intervals.size(); ++i) {
//       const double lower_bound = get<0>(new_node->intervals[i]) + expand_time;
//       const double upper_bound = get<1>(new_node->intervals[i]) + expand_time;
//       assert(lower_bound > 0);
//       assert(lower_bound < upper_bound);
//
//       auto safe_intervals = safe_interval_table.table[neighbor->point];
//       for (auto& safe_interval : safe_intervals) {
//         if (lower_bound >= get<1>(safe_interval)) continue;
//         if (upper_bound <= get<0>(safe_interval)) break;
//
//         const double earliest_arrival_time = constraint_table.getEarliestArrivalTime(
//             agent_id, new_node->point, neighbor->point, expand_time, max(get<0>(safe_interval), lower_bound),
//             min(get<1>(safe_interval), upper_bound), env.radii[agent_id]);
//         if (earliest_arrival_time < 0.0) continue;
//
//         const double earliest_arrival_time_soft = constraint_table.getEarliestArrivalTimeSoft(
//             agent_id, new_node->point, neighbor->point, expand_time, earliest_arrival_time,
//             min(get<1>(safe_interval), upper_bound), env.radii[agent_id]);
//         if (earliest_arrival_time == earliest_arrival_time_soft) {
//           // no collision
//           candidate_intervals.emplace_back(earliest_arrival_time, get<1>(safe_interval));
//           candidate_soft_conflicts.emplace_back(new_node->soft_conflicts[i]);
//           candidate_parent_interval_indices.emplace_back(i);
//           assert(new_node->soft_conflicts[i] >= 0);
//         } else if (earliest_arrival_time_soft < 0.0) {
//           // all collision
//           candidate_intervals.emplace_back(earliest_arrival_time, get<1>(safe_interval));
//           candidate_soft_conflicts.emplace_back(new_node->soft_conflicts[i] + 1);
//           candidate_parent_interval_indices.emplace_back(i);
//           assert(new_node->soft_conflicts[i] + 1 >= 0);
//         } else {
//           // collision
//           candidate_intervals.emplace_back(earliest_arrival_time, earliest_arrival_time_soft);
//           candidate_soft_conflicts.emplace_back(new_node->soft_conflicts[i] + 1);
//           candidate_parent_interval_indices.emplace_back(i);
//           assert(new_node->soft_conflicts[i] + 1 >= 0);
//
//           // no collision
//           candidate_intervals.emplace_back(earliest_arrival_time_soft, get<1>(safe_interval));
//           candidate_soft_conflicts.emplace_back(new_node->soft_conflicts[i]);
//           candidate_parent_interval_indices.emplace_back(i);
//           assert(new_node->soft_conflicts[i] >= 0);
//         }
//       }
//     }
//
//     if (candidate_intervals.empty()) continue;
//
//     double earliest_arrival_time = numeric_limits<double>::infinity();
//     int min_soft_conflict = numeric_limits<int>::max();
//     for (const auto& interval : candidate_intervals) {
//       earliest_arrival_time = min(earliest_arrival_time, get<0>(interval));
//     }
//     for (const auto& soft_conflict : candidate_soft_conflicts) {
//       min_soft_conflict = min(min_soft_conflict, soft_conflict);
//     }
//
//     // update if new_node is better than neighbor
//     // if (min_soft_conflict < neighbor->min_soft_conflict) {
//     //   neighbor->earliest_arrival_time = earliest_arrival_time;
//     //   neighbor->min_soft_conflict = min_soft_conflict;
//     //   neighbor->intervals = move(candidate_intervals);
//     //   neighbor->soft_conflicts = move(candidate_soft_conflicts);
//     //   neighbor->parent_interval_indices = move(candidate_parent_interval_indices);
//     //   // update parent
//     //   auto old_parent = neighbor->parent.lock();
//     //   if (old_parent) {
//     //     old_parent->children.erase(remove(old_parent->children.begin(), old_parent->children.end(), neighbor),
//     //                                old_parent->children.end());
//     //   }
//     //
//     //   neighbor->parent = new_node;
//     //   new_node->children.emplace_back(neighbor);
//     //   propagateCostToSuccessor(neighbor, safe_interval_table);
//     //   assert(neighbor->earliest_arrival_time - new_node->earliest_arrival_time >= 0);
//     //   assert(neighbor->min_soft_conflict - new_node->min_soft_conflict >= 0);
//     // }
//     if (earliest_arrival_time < neighbor->earliest_arrival_time) {
//       neighbor->earliest_arrival_time = earliest_arrival_time;
//       neighbor->min_soft_conflict = min_soft_conflict;
//       neighbor->intervals = move(candidate_intervals);
//       neighbor->soft_conflicts = move(candidate_soft_conflicts);
//       neighbor->parent_interval_indices = move(candidate_parent_interval_indices);
//       // update parent
//       auto old_parent = neighbor->parent.lock();
//       if (old_parent) {
//         old_parent->children.erase(remove(old_parent->children.begin(), old_parent->children.end(), neighbor),
//                                    old_parent->children.end());
//       }
//
//       neighbor->parent = new_node;
//       new_node->children.emplace_back(neighbor);
//       propagateCostToSuccessor(neighbor, safe_interval_table);
//       assert(neighbor->earliest_arrival_time - new_node->earliest_arrival_time >= 0);
//       assert(neighbor->min_soft_conflict - new_node->min_soft_conflict >= 0);
//     }
//   }
// }
//
// void SIRRT::propagateCostToSuccessor(const shared_ptr<LLNode>& node, SafeIntervalTable& safe_interval_table) {
//   auto iter = node->children.begin();
//   while (iter != node->children.end()) {
//     auto child = *iter;
//     const double expand_time = calculateDistance(node->point, child->point) / env.velocities[agent_id];
//     vector<Interval> intervals;
//     vector<int> soft_conflicts;
//     vector<int> parent_interval_indices;
//
//     for (int i = 0; i < node->intervals.size(); ++i) {
//       const double lower_bound = get<0>(node->intervals[i]) + expand_time;
//       const double upper_bound = get<1>(node->intervals[i]) + expand_time;
//       assert(lower_bound > 0);
//       assert(lower_bound < upper_bound);
//       auto safe_intervals = safe_interval_table.table[child->point];
//       for (auto& safe_interval : safe_intervals) {
//         if (lower_bound >= get<1>(safe_interval)) continue;
//         if (upper_bound <= get<0>(safe_interval)) break;
//
//         const double earliest_arrival_time = constraint_table.getEarliestArrivalTime(
//             agent_id, node->point, child->point, expand_time, max(get<0>(safe_interval), lower_bound),
//             min(get<1>(safe_interval), upper_bound), env.radii[agent_id]);
//         if (earliest_arrival_time < 0.0) continue;
//
//         const double earliest_arrival_time_soft = constraint_table.getEarliestArrivalTimeSoft(
//             agent_id, node->point, child->point, expand_time, earliest_arrival_time,
//             min(get<1>(safe_interval), upper_bound), env.radii[agent_id]);
//         if (earliest_arrival_time == earliest_arrival_time_soft) {
//           // no collision
//           intervals.emplace_back(earliest_arrival_time, get<1>(safe_interval));
//           soft_conflicts.emplace_back(node->soft_conflicts[i]);
//           parent_interval_indices.emplace_back(i);
//           assert(node->soft_conflicts[i] >= 0);
//         } else if (earliest_arrival_time_soft < 0.0) {
//           // all collision
//           intervals.emplace_back(earliest_arrival_time, get<1>(safe_interval));
//           soft_conflicts.emplace_back(node->soft_conflicts[i] + 1);
//           parent_interval_indices.emplace_back(i);
//           assert(node->soft_conflicts[i] + 1 >= 0);
//         } else {
//           // collision
//           intervals.emplace_back(earliest_arrival_time, earliest_arrival_time_soft);
//           soft_conflicts.emplace_back(node->soft_conflicts[i] + 1);
//           parent_interval_indices.emplace_back(i);
//           assert(node->soft_conflicts[i] + 1 >= 0);
//
//           // no collision
//           intervals.emplace_back(earliest_arrival_time_soft, get<1>(safe_interval));
//           soft_conflicts.emplace_back(node->soft_conflicts[i]);
//           parent_interval_indices.emplace_back(i);
//           assert(node->soft_conflicts[i] >= 0);
//         }
//       }
//     }
//
//     // 업데이트로 interval이 없어진 child는 삭제한다.
//     if (intervals.empty()) {
//       child->parent.reset();
//       iter = node->children.erase(iter);
//       continue;
//     }
//
//     double earliest_arrival_time = numeric_limits<double>::infinity();
//     int min_soft_conflict = numeric_limits<int>::max();
//     for (const auto& interval : child->intervals) {
//       earliest_arrival_time = min(earliest_arrival_time, get<0>(interval));
//     }
//     for (const auto& soft_conflict : child->soft_conflicts) {
//       min_soft_conflict = min(min_soft_conflict, soft_conflict);
//     }
//
//     child->earliest_arrival_time = earliest_arrival_time;
//     child->min_soft_conflict = min_soft_conflict;
//     child->intervals = move(intervals);
//     child->soft_conflicts = move(soft_conflicts);
//     child->parent_interval_indices = move(parent_interval_indices);
//
//     propagateCostToSuccessor(child, safe_interval_table);
//     ++iter;
//   }
// }

void SIRRT::release() {
  nodes.clear();
  path.clear();
  goal_node = nullptr;
}
