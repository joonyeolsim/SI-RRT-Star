#include "SIRRT.h"

Path SIRRT::run() {
  release();
  SafeIntervalTable safe_interval_table(env);

  // initialize start and goal safe intervals
  if (env.algorithm == "pp")
    constraint_table.getSafeIntervalTablePath(agent_id, start_point, env.radii[agent_id],
                                              safe_interval_table.table[start_point]);
  else if (env.algorithm == "cbs")
    constraint_table.getSafeIntervalTable(agent_id, start_point, env.radii[agent_id],
                                          safe_interval_table.table[start_point]);
  if (safe_interval_table.table[start_point].empty()) {
    throw std::runtime_error("Safe interval table is empty for the start point.");
  }

  if (env.algorithm == "pp")
    constraint_table.getSafeIntervalTablePath(agent_id, goal_point, env.radii[agent_id],
                                              safe_interval_table.table[goal_point]);
  else if (env.algorithm == "cbs")
    constraint_table.getSafeIntervalTable(agent_id, goal_point, env.radii[agent_id],
                                          safe_interval_table.table[goal_point]);
  if (safe_interval_table.table[goal_point].empty()) {
    throw std::runtime_error("Safe interval table is empty for the goal point.");
  }

  // initialize start node
  auto start_node = make_shared<LLNode>(start_point);
  start_node->interval = make_pair(0, safe_interval_table.table[start_point].front().second);
  nodes.push_back(start_node);

  // initialize goal node
  auto goal_node = make_shared<LLNode>(goal_point);
  goal_node->interval = make_pair(safe_interval_table.table[goal_point].back().first, numeric_limits<int>::max() / 2);

  int best_earliest_arrival_time_step = numeric_limits<int>::max() / 2;
  int iteration = env.iterations[agent_id];
  while (iteration--) {
    Point random_point = generateRandomPoint();
    const shared_ptr<LLNode> nearest_node = getNearestNode(random_point);
    Point new_point = steer(nearest_node, random_point, safe_interval_table);
    if (new_point == make_tuple(-1.0, -1.0)) {
      continue;
    }

    // TODO: Handling Goal node (remove target conflict)
    // SIRRT*
    vector<shared_ptr<LLNode>> neighbors;
    getNeighbors(new_point, neighbors);
    assert(!neighbors.empty());
    vector<shared_ptr<LLNode>> new_nodes = chooseParent(new_point, neighbors, safe_interval_table);
    if (new_nodes.empty()) {
      continue;
    }
    // for (auto& new_node : new_nodes) {
    //   rewire(new_node, neighbors, safe_interval_table);
    // }

    // check goal
    for (auto& new_node : new_nodes) {
      if (calculateDistance(new_node->point, goal_point) < env.epsilon) {
        if (safe_interval_table.table[goal_point].back().first <= new_node->interval.first &&
          new_node->interval.first < best_earliest_arrival_time_step) {
          goal_node = new_node;
          best_earliest_arrival_time_step = goal_node->interval.first;
        }
      } else {
        nodes.push_back(new_node);
      }
    }
  }

  if (goal_node != nullptr && best_earliest_arrival_time_step < numeric_limits<int>::max() / 2) {
    nodes.push_back(goal_node);
    path = updatePath(goal_node);
    return path;
  }

  // cout << "No path found!" << endl;
  return path;
}

Point SIRRT::generateRandomPoint() {
  if (dis_100(env.gen) < env.goal_sample_rates[agent_id]) {
    return goal_point;
  }
  return make_tuple(dis_width(env.gen), dis_height(env.gen));
}

shared_ptr<LLNode> SIRRT::getNearestNode(const Point& point) const {
  if (nodes.empty()) {
    return nullptr;
  }

  double min_distance = numeric_limits<double>::max() / 2.0;
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
  // dv = ds / dt
  // dt = 1 because time step is 1
  // dv = ds
  const double velocity = expand_distance;

  const Point to_point = make_tuple(get<0>(from_node->point) + velocity * cos(theta),
                                    get<1>(from_node->point) + velocity * sin(theta));

  if (constraint_table.obstacleConstrained(to_point, env.radii[agent_id])) {
    return make_tuple(-1, -1);
  }

  vector<Interval> safe_intervals;
  if (env.algorithm == "pp")
    constraint_table.getSafeIntervalTablePath(agent_id, to_point, env.radii[agent_id], safe_intervals);
  else if (env.algorithm == "cbs")
    constraint_table.getSafeIntervalTable(agent_id, to_point, env.radii[agent_id], safe_intervals);
  if (safe_intervals.empty()) {
    return make_tuple(-1, -1);
  }
  safe_interval_table.table[to_point] = safe_intervals;

  return to_point;
}

Path SIRRT::updatePath(const shared_ptr<LLNode>& goal_node) const {
  Path path;
  shared_ptr<LLNode> curr_node = goal_node;

  // Collect the path from goal to start and expand if needed
  while (curr_node->parent != nullptr) {
    // Add the current node to the path
    path.emplace_back(curr_node->point, curr_node->interval.first);
    int time_diff = curr_node->interval.first - curr_node->parent->interval.first;
    for (int i = 1; i < time_diff; ++i) {
      path.emplace_back(curr_node->point, curr_node->interval.first - i);
    }
    curr_node = curr_node->parent;
  }
  // Add the start node to the path
  path.emplace_back(curr_node->point, curr_node->interval.first);

  // Reverse the path to get it from start to goal
  reverse(path.begin(), path.end());

  assert(calculateDistance(get<0>(path.front()), start_point) < env.epsilon);
  assert(calculateDistance(get<0>(path.back()), goal_point) < env.epsilon);

  return path;
}

void SIRRT::getNeighbors(Point point, vector<shared_ptr<LLNode>>& neighbors) const {
  assert(!nodes.empty());
  assert(neighbors.empty());

  const double connection_radius = env.max_expand_distances[agent_id] + env.epsilon;
  for (const auto& node : nodes) {
    if (calculateDistance(node->point, point) < connection_radius) {
      neighbors.emplace_back(node);
    }
  }
}

vector<shared_ptr<LLNode>> SIRRT::chooseParent(const Point& new_point, const vector<shared_ptr<LLNode>>& neighbors,
                                       SafeIntervalTable& safe_interval_table) const {
  assert(!neighbors.empty());

  auto new_nodes = vector<shared_ptr<LLNode>>();
  auto safe_intervals = safe_interval_table.table[new_point];

  for (auto& safe_interval : safe_intervals) {
    shared_ptr<LLNode> new_node = make_shared<LLNode>(new_point);
    // set the safe interval of the new node of the current safe interval
    new_node->interval = make_pair(safe_interval.first, safe_interval.second);

    for (const auto& neighbor : neighbors) {
      // lower_bound is the earliest arrival time of the new node from the neighbor node
      const int lower_bound = neighbor->interval.first + 1;
      // upper_bound is the latest arrival time of the new node from the neighbor node
      const int upper_bound = neighbor->interval.second + 1;
      assert(lower_bound > 0);
      assert(lower_bound < upper_bound);

      // safe interval간의 overlap이 없다면, neighbor로부터 new_node까지 이동할 수 없음.
      if (lower_bound >= safe_interval.second) continue;
      if (upper_bound <= safe_interval.first) continue;

      int earliest_arrival_time = max(safe_interval.first, lower_bound);
      if (earliest_arrival_time >= min(safe_interval.second, upper_bound)) continue;

      // 이미 new_node의 부모가 있고, lower_bound가 new_node의 earliest arrival time보다 크다면, 더 이상 확인할 필요가 없음
      if (new_node->parent && earliest_arrival_time >= new_node->interval.first) continue;

      // lower_bound를 높여가면서 중간에 충돌이 발생하는지 확인. 만약 충돌이 발생하지 않는다면, 해당 시간을 earliest arrival time으로 설정
      // const int earliest_arrival_time = constraint_table.getEarliestArrivalTime(
      //     agent_id, neighbor->point, new_node->point, max(safe_interval.first, lower_bound),
      //     min(safe_interval.second, upper_bound), env.radii[agent_id]);
      // if (earliest_arrival_time < 0) continue;

      // new_node의 부모가 없거나, earliest arrival time이 더 빠르다면, new_node의 부모를 neighbor로 설정
      new_node->interval.first = earliest_arrival_time;
      new_node->parent = neighbor;
    }

    // new_node의 부모가 있다면 new_nodes에 추가
    if (new_node->parent) {
      new_nodes.push_back(new_node);
    }
  }

  return new_nodes;
}

// shared_ptr<LLNode> SIRRT::chooseParent(const Point& new_point, const vector<shared_ptr<LLNode>>& neighbors,
//                                        SafeIntervalTable& safe_interval_table) const {
//   assert(!neighbors.empty());
//
//   shared_ptr<LLNode> new_node = make_shared<LLNode>(new_point);
//
//   for (const auto& neighbor : neighbors) {
//     // lower_bound is the earliest arrival time of the new node from the neighbor node
//     const int lower_bound = neighbor->interval.first + 1;
//     // upper_bound is the latest arrival time of the new node from the neighbor node
//     const int upper_bound = neighbor->interval.second + 1;
//     assert(lower_bound > 0);
//     assert(lower_bound < upper_bound);
//
//     auto safe_intervals = safe_interval_table.table[new_node->point];
//     for (auto& safe_interval : safe_intervals) {
//       if (lower_bound >= safe_interval.second) continue;
//       if (upper_bound <= safe_interval.first) break;
//
//       // lower_bound를 높여가면서 중간에 충돌이 발생하는지 확인. 만약 충돌이 발생하지 않는다면, 해당 시간을 earliest arrival time으로 설정
//       const int earliest_arrival_time = constraint_table.getEarliestArrivalTime(
//           agent_id, neighbor->point, new_node->point, max(safe_interval.first, lower_bound),
//           min(safe_interval.second, upper_bound), env.radii[agent_id]);
//       if (earliest_arrival_time < 0) continue;
//
//       if (new_node->parent == nullptr || earliest_arrival_time < new_node->interval.first) {
//         new_node->interval = make_pair(earliest_arrival_time, safe_interval.second);
//         new_node->parent = neighbor;
//       }
//     }
//   }
//
//   if (new_node->parent == nullptr) {
//     return nullptr;
//   }
//   return new_node;
// }

void SIRRT::rewire(const shared_ptr<LLNode>& new_node, const vector<shared_ptr<LLNode>>& neighbors,
                   SafeIntervalTable& safe_interval_table) {
  assert(!neighbors.empty());

  for (auto& neighbor : neighbors) {
    if (neighbor == new_node->parent) continue;

    // lower_bound is the earliest arrival time of the neighbor node from the new node
    const int lower_bound = get<0>(new_node->interval) + 1;
    // upper_bound is the latest arrival time of the neighbor node from the new node
    const int upper_bound = get<1>(new_node->interval) + 1;
    assert(lower_bound > 0);
    assert(lower_bound < upper_bound);

    auto safe_intervals = safe_interval_table.table[neighbor->point];
    for (auto& safe_interval : safe_intervals) {
      if (lower_bound >= safe_interval.second) continue;
      if (upper_bound <= safe_interval.first) break;

      const int earliest_arrival_time = constraint_table.getEarliestArrivalTime(
          agent_id, new_node->point, neighbor->point, max(safe_interval.first, lower_bound),
          min(safe_interval.second, upper_bound), env.radii[agent_id]);
      if (earliest_arrival_time < 0) continue;

      if (earliest_arrival_time < neighbor->interval.first) {
        if (safe_interval.second <= neighbor->interval.first) {
          shared_ptr<LLNode> new_neighbor_node = make_shared<LLNode>(neighbor->point);
          new_neighbor_node->interval = make_pair(earliest_arrival_time, safe_interval.second);
          new_neighbor_node->parent = new_node;
          nodes.push_back(new_neighbor_node);
        } else {
          neighbor->interval = make_pair(earliest_arrival_time, safe_interval.second);
          neighbor->parent = new_node;
        }
        break;
      }
    }
  }
}

void SIRRT::release() {
  nodes.clear();
  path.clear();
  goal_node = nullptr;
}
