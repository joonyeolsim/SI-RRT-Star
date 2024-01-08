//
// Created by joonyeol on 23. 12. 7.
//

#ifndef COMMON_H
#define COMMON_H

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <boost/functional/hash.hpp>
#include <boost/heap/fibonacci_heap.hpp>
#include <cassert>
#include <chrono>
#include <climits>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <random>
#include <utility>
#include <vector>

using namespace std;

typedef tuple<double, double> Point;
typedef vector<tuple<Point, double>> Path;
typedef tuple<double, double> Interval;               // (from_time, to_time)
typedef tuple<int, int, tuple<Path, Path>> Conflict;  // (agent1, agent2, (trajectory1, trajectory2))
typedef tuple<double, Path> Constraint;               // (occupied_radius, occupied_trajectory)
typedef vector<Path> Solution;

void savePath(const Path& path, const string& filename);
void saveSolution(const Solution& solution, const string& filename);
void saveData(double cost, double makespan, double duration, const string& filename);
double calculateDistance(Point point1, Point point2);

struct PointHash {
  size_t operator()(const Point& point) const {
    auto [x, y] = point;
    size_t seed = 0;
    boost::hash_combine(seed, x);
    boost::hash_combine(seed, y);
    return seed;
  }
};

class Obstacle {
 public:
  Point point;
  Obstacle(double x, double y) : point(make_tuple(x, y)) {}
  virtual ~Obstacle() = default;
  virtual bool constrained(const Point& other_point, const double other_radius) = 0;
};

class RectangularObstacle : public Obstacle {
 public:
  double width, height;
  RectangularObstacle(double x, double y, double width, double height) : Obstacle(x, y), width(width), height(height) {}
  bool constrained(const Point& other_point, const double other_radius) override {
    const auto& [agentX, agentY] = other_point;
    const double agentR = other_radius;
    const auto& [x, y] = point;

    double rectLeft = x - width / 2;
    double rectRight = x + width / 2;
    double rectTop = y - height / 2;
    double rectBottom = y + height / 2;

    if (agentX >= rectLeft && agentX <= rectRight && agentY >= rectTop && agentY <= rectBottom) {
      return true;
    }

    double closestX = (agentX <= rectLeft) ? rectLeft : (agentX >= rectRight) ? rectRight : agentX;
    double closestY = (agentY <= rectTop) ? rectTop : (agentY >= rectBottom) ? rectBottom : agentY;

    double distX = agentX - closestX;
    double distY = agentY - closestY;

    return (distX * distX + distY * distY) <= (agentR * agentR);
  }
};

class CircularObstacle : public Obstacle {
 public:
  double radius;
  CircularObstacle(double x, double y, double radius) : Obstacle(x, y), radius(radius) {}
  bool constrained(const Point& other_point, const double other_radius) override {
    return (calculateDistance(point, other_point) <= radius + other_radius);
  }
};

#endif  // COMMON_H
