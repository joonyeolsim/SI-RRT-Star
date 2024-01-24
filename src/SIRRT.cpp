#include "SIRRT.h"

Path SIRRT::run() {
  release();
  SafeIntervalTable safe_interval_table(env);

  // initialize start and goal safe intervals
  vector<Interval> start_safe_intervals;
  constraint_table.getSafeIntervalTablePath(agent_id, start_point, env.radii[agent_id], start_safe_intervals);
  assert(!start_safe_intervals.empty());
  safe_interval_table.table[start_point] = start_safe_intervals;

  vector<Interval> goal_safe_intervals;
  constraint_table.getSafeIntervalTablePath(agent_id, goal_point, env.radii[agent_id], goal_safe_intervals);
  assert(!goal_safe_intervals.empty());
  safe_interval_table.table[goal_point] = goal_safe_intervals;

  // initialize start node
  const auto start_node = make_shared<LLNode>(start_point);
  start_node->interval = make_tuple(0.0, get<1>(start_safe_intervals[0]));
  nodes.push_back(start_node);

  // the earliest timestep that the agent can hold its goal location.
  double earliest_goal_arrival_time = 0.0;
  for (auto& interval : goal_safe_intervals) {
    earliest_goal_arrival_time = max(earliest_goal_arrival_time, get<0>(interval));
  }

  double best_earliest_arrival_time = numeric_limits<double>::infinity();
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
    shared_ptr<LLNode> new_node = chooseParent(new_point, neighbors, safe_interval_table);
    if (new_node == nullptr) {
      continue;
    }
    rewire(new_node, neighbors, safe_interval_table);

    // check goal
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
      }
    } else {
      nodes.push_back(new_node);
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

  // cout << "No path found!" << endl;
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
  constraint_table.getSafeIntervalTablePath(agent_id, to_point, env.radii[agent_id], safe_intervals);
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
      // cout << "path is not continuous" << endl;
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

shared_ptr<LLNode> SIRRT::chooseParent(const Point& new_point, const vector<shared_ptr<LLNode>>& neighbors,
                                       SafeIntervalTable& safe_interval_table) const {
  assert(!neighbors.empty());

  shared_ptr<LLNode> new_node = make_shared<LLNode>(new_point);

  // 이웃 노드들을 순회하면서 to point로 갈 수 있는 가장 낮은 earliest arrival time을 가지는 이웃을 찾는다.
  for (const auto& neighbor : neighbors) {
    // 만약 이웃 노드로부터 새로운 노드로 갈 때 정적인 장애물과 충돌한다면 continue
    if (constraint_table.obstacleConstrained(agent_id, neighbor->point, new_point, env.radii[agent_id])) continue;

    // 이웃 노드로부터 새로운 노드로 갈 때 이동하는 시간
    const double expand_time = calculateDistance(neighbor->point, new_point) / env.velocities[agent_id];

    // 이웃 노드로부터 새로운 노드로 갈 때의 lower bound와 upper bound
    const double lower_bound = get<0>(neighbor->interval) + expand_time;
    const double upper_bound = get<1>(neighbor->interval) + expand_time;
    assert(lower_bound > 0);
    assert(lower_bound < upper_bound);

    // new node의 safe interval들을 순회하면서
    auto safe_intervals = safe_interval_table.table[new_point];
    for (auto& safe_interval : safe_intervals) {
      // lower bound와 upper bound가 safe interval과 겹치지 않는다면 continue
      if (lower_bound >= get<1>(safe_interval)) continue;
      if (upper_bound <= get<0>(safe_interval)) break;

      // neighbor노드로부터 new node로 갈 때의 가장 빠른 도착 시간을 구한다.
      // 가장 빠른 도착 시간이란 neighbor노드로부터 new node로 갈 때 hard constraint와 충돌하지 않는 가장 빠른 시간을
      // 의미한다.
      const double earliest_arrival_time = constraint_table.getEarliestArrivalTime(
          agent_id, neighbor->point, new_point, expand_time, max(get<0>(safe_interval), lower_bound),
          min(get<1>(safe_interval), upper_bound), env.radii[agent_id]);
      // 현재 safe interval에서 neighbor노드에서 new node로 갈 수 있는 경로가 없으면 continue
      if (earliest_arrival_time < 0.0) continue;

      if (new_node->parent.lock() == nullptr || earliest_arrival_time < get<0>(new_node->interval)) {
        new_node->interval = make_tuple(earliest_arrival_time, get<1>(safe_interval));
        new_node->parent = neighbor;
      }
    }
  }

  // 만약 parent node가 nullptr이라면 그 어떠한 이웃 노드로부터도 새로운 노드로 갈 수 없다.
  if (new_node->parent.lock() == nullptr) {
    return nullptr;
  }

  // new node를 parent node의 자식 노드로 추가한다.
  new_node->parent.lock()->children.emplace_back(new_node);
  return new_node;
}

void SIRRT::rewire(const shared_ptr<LLNode>& new_node, const vector<shared_ptr<LLNode>>& neighbors,
                   SafeIntervalTable& safe_interval_table) {
  assert(!neighbors.empty());
  for (auto& neighbor : neighbors) {
    // cout << get<0>(neighbor->point) << " " << get<1>(neighbor->point) << endl;
    // 만약 neighbor가 new_node의 부모라면 continue
    if (neighbor == new_node->parent.lock()) continue;

    // 만약 새로운 노드로부터 이웃 노드로 갈 때 정적인 장애물과 충돌한다면 continue
    if (constraint_table.obstacleConstrained(agent_id, new_node->point, neighbor->point, env.radii[agent_id])) continue;

    // new_node로부터 neighbor로 갈 때 이동하는 시간
    const double expand_time = calculateDistance(new_node->point, neighbor->point) / env.velocities[agent_id];

    // new node로부터 neighbor로 갈 때의 lower bound와 upper bound
    const double lower_bound = get<0>(new_node->interval) + expand_time;
    const double upper_bound = get<1>(new_node->interval) + expand_time;
    assert(lower_bound > 0);
    assert(lower_bound < upper_bound);

    // neighbor의 safe interval들을 순회하면서
    auto safe_intervals = safe_interval_table.table[neighbor->point];
    for (auto& safe_interval : safe_intervals) {
      // lower bound와 upper bound가 safe interval과 겹치지 않는다면 continue
      if (lower_bound >= get<1>(safe_interval)) continue;
      if (upper_bound <= get<0>(safe_interval)) break;

      // new node로부터 neighbor로 갈 때의 가장 빠른 도착 시간을 구한다.
      // 가장 빠른 도착 시간이란 new node로부터 neighbor로 갈 때 hard constraint와 충돌하지 않는 가장 빠른 시간을
      // 의미한다.
      const double earliest_arrival_time = constraint_table.getEarliestArrivalTime(
          agent_id, new_node->point, neighbor->point, expand_time, max(get<0>(safe_interval), lower_bound),
          min(get<1>(safe_interval), upper_bound), env.radii[agent_id]);
      // 현재 safe interval에서 new node에서 neighbor로 갈 수 있는 경로가 없으면 continue
      if (earliest_arrival_time < 0.0) continue;

      // 가장 빠른 도착 시간이 neighbor의 interval보다 작다면 neighbor의 interval을 업데이트한다.
      // if (earliest_arrival_time < get<0>(neighbor->interval)) {
      //   // earliest arrival time이 neighbor의 interval의 upper bound보다 크다면 새로운 노드 생성
      //   if (get<1>(safe_interval) <= get<0>(neighbor->interval)) {
      //     shared_ptr<LLNode> new_neighbor_node = make_shared<LLNode>(neighbor->point);
      //     new_neighbor_node->interval = make_tuple(earliest_arrival_time, get<1>(safe_interval));
      //     new_neighbor_node->parent = new_node;
      //     new_node->children.emplace_back(neighbor);
      //     nodes.push_back(new_neighbor_node);
      //   } else {
      //     neighbor->interval = make_tuple(earliest_arrival_time, get<1>(safe_interval));
      //     neighbor->parent = new_node;
      //     new_node->children.emplace_back(neighbor);
      //   }
      //   break;
      // }
      if (earliest_arrival_time < get<0>(neighbor->interval)) {
        neighbor->interval = make_tuple(earliest_arrival_time, get<1>(safe_interval));
        neighbor->parent.lock()->children.erase(
            remove(neighbor->parent.lock()->children.begin(), neighbor->parent.lock()->children.end(), neighbor),
            neighbor->parent.lock()->children.end());
        neighbor->parent = new_node;
        new_node->children.emplace_back(neighbor);
        // propagateCostToSuccessor(neighbor, safe_interval_table);
        break;
      }
    }
  }
}

void SIRRT::propagateCostToSuccessor(const shared_ptr<LLNode>& node, SafeIntervalTable& safe_interval_table) {
  auto children = node->children;
  for (auto& child : children) {
    // 만약 부모 노드로부터 자식 노드로 갈 때 정적인 장애물과 충돌한다면 child는 삭제한다.
    if (constraint_table.obstacleConstrained(agent_id, node->point, child->point, env.radii[agent_id])) {
      pruneNode(child);
      continue;
    }

    // 부모 노드로부터 자식 노드로 갈 때 이동하는 시간
    const double expand_time = calculateDistance(node->point, child->point) / env.velocities[agent_id];

    // 부모 노드로부터 자식 노드로 갈 때의 lower bound와 upper bound
    const double lower_bound = get<0>(node->interval) + expand_time;
    const double upper_bound = get<1>(node->interval) + expand_time;
    assert(lower_bound > 0);
    assert(lower_bound < upper_bound);

    // child의 safe interval들을 순회하면서
    bool updated = false;
    auto safe_intervals = safe_interval_table.table[child->point];
    for (auto& safe_interval : safe_intervals) {
      // lower bound와 upper bound가 safe interval과 겹치지 않는다면 continue
      if (lower_bound >= get<1>(safe_interval)) continue;
      if (upper_bound <= get<0>(safe_interval)) break;

      // node로부터 child로 갈 때의 가장 빠른 도착 시간을 구한다.
      // 가장 빠른 도착 시간이란 node로부터 child로 갈 때 hard constraint와 충돌하지 않는 가장 빠른 시간을 의미한다.
      const double earliest_arrival_time = constraint_table.getEarliestArrivalTime(
          agent_id, node->point, child->point, expand_time, max(get<0>(safe_interval), lower_bound),
          min(get<1>(safe_interval), upper_bound), env.radii[agent_id]);
      if (earliest_arrival_time < 0.0) {
        continue;
      }

      child->interval = make_tuple(earliest_arrival_time, get<1>(safe_interval));
      updated = true;
      break;
    }

    // 업데이트로 interval이 없어진 child는 삭제한다.
    if (!updated) {
      pruneNode(child);
    } else {
      propagateCostToSuccessor(child, safe_interval_table);
    }
  }
}

void SIRRT::pruneNode(shared_ptr<LLNode>& node) {
  // cout << "Pruning node: " << get<0>(node->point) << " " << get<1>(node->point) << endl;
  while (!node->children.empty()) {
    auto child = node->children.back();
    node->children.pop_back();
    pruneNode(child);
  }
  const auto parentPtr = node->parent.lock();
  assert(parentPtr != nullptr);
  parentPtr->children.erase(remove(parentPtr->children.begin(), parentPtr->children.end(), node),
                            parentPtr->children.end());
  node->parent.reset();
  const auto node_iter = find(nodes.begin(), nodes.end(), node);
  if (node_iter != nodes.end()) {
    nodes.erase(node_iter);
  }
}

void SIRRT::release() {
  nodes.clear();
  path.clear();
  goal_node = nullptr;
}
