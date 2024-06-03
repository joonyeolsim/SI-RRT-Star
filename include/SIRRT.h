#ifndef SIRRT_H
#define SIRRT_H

#include "ConstraintTable.h"
#include "LLNode.h"
#include "SafeIntervalTable.h"
#include "common.h"

class SIRRT {
 public:
  std::uniform_real_distribution<> dis_width;
  std::uniform_real_distribution<> dis_height;
  std::uniform_real_distribution<> dis_velocity;
  std::uniform_real_distribution<> dis_100;
  vector<shared_ptr<LLNode>> nodes;
  shared_ptr<LLNode> goal_node;
  Point start_point;
  Point goal_point;
  Path path;
  int agent_id;
  SharedEnv& env;
  ConstraintTable& constraint_table;
  double max_velocity;
  const double position_weight = 0.9;
  const double velocity_weight = 0.1;

  SIRRT(int agent_id, SharedEnv& env, ConstraintTable& constraint_table)
      : dis_width(env.radii[agent_id], env.width - env.radii[agent_id]),
        dis_height(env.radii[agent_id], env.height - env.radii[agent_id]),
        dis_100(0.0, 100.0),
        env(env),
        constraint_table(constraint_table),
        agent_id(agent_id),
        start_point(env.start_points[agent_id]),
        goal_point(env.goal_points[agent_id]) {
    max_velocity = env.max_expand_distances[agent_id];
    dis_velocity = std::uniform_real_distribution<>(-max_velocity, max_velocity);
  }
  ~SIRRT() = default;
  Path run();
  Point generateRandomPoint();
  double generateRandomVelocity();
  shared_ptr<LLNode> getNearestNode(const Point& point, double velocity) const;
  pair<Point, double> steer(const shared_ptr<LLNode>& from_node, const Point& sample_point, double sample_velocity,
                   SafeIntervalTable& safe_interval_table) const;
  Path updatePath(const shared_ptr<LLNode>& goal_node) const;
  void getNeighbors(Point point, double velocity, vector<shared_ptr<LLNode>>& neighbors) const;
  vector<shared_ptr<LLNode>> chooseParent(const Point& new_point, double new_velocity, const vector<shared_ptr<LLNode>>& neighbors,
                                       SafeIntervalTable& safe_interval_table) const;
  void rewire(const shared_ptr<LLNode>& new_node, const vector<shared_ptr<LLNode>>& neighbors);
  void release();
};

#endif  // SIRRT_H
