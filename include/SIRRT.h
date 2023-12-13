//
// Created by joonyeol on 23. 12. 7.
//

#ifndef SIRRT_H
#define SIRRT_H

#include "ConstraintTable.h"
#include "LLNode.h"
#include "common.h"

class SIRRT {
 public:
  std::uniform_real_distribution<> dis_10;
  std::uniform_real_distribution<> dis_100;
  vector<shared_ptr<LLNode>> nodes;
  Point start_point;
  Point goal_point;
  Path path;
  int agent_id;
  SharedEnv& env;
  ConstraintTable& constraint_table;

  SIRRT(SharedEnv& env, ConstraintTable& constraint_table, int agent_id)
      : dis_10(0.0, 10.0),
        dis_100(0.0, 100.0),
        env(env),
        constraint_table(constraint_table),
        agent_id(agent_id),
        start_point(env.start_points[agent_id]),
        goal_point(env.goal_points[agent_id]) {}
  ~SIRRT() = default;
  Path run();
  Point generateRandomPoint();
  shared_ptr<LLNode> getNearestNode(Point point);
  shared_ptr<LLNode> steer(const shared_ptr<LLNode>& from_node, Point to_point);
  Path updatePath(const shared_ptr<LLNode>& goal_node);
  void release();
};

#endif  // SIRRT_H
