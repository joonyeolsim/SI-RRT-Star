//
// Created by joonyeol on 23. 12. 7.
//

#ifndef COMMON_H
#define COMMON_H

#include <yaml-cpp/yaml.h>

#include <algorithm>
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
typedef tuple<double, double> Interval;           // (from_time, to_time)
typedef tuple<int, int, Interval> Conflict;       // (agent1, agent2, interval)
typedef tuple<int, double, Interval> Constraint;  // (agent, counter radius, interval)
typedef vector<Path> Solution;

void savePath(const Path& path, const string& filename);
void saveSolution(const Solution& solution, const string& filename);
double calculateDistance(Point point1, Point point2);

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
    const double x = get<0>(point);
    const double y = get<1>(point);
    const double other_x = get<0>(other_point);
    const double other_y = get<1>(other_point);
    return (x - width / 2 - other_radius <= other_x && other_x <= x + width / 2 + other_radius &&
            y - height / 2 - other_radius <= other_y && other_y <= y + height / 2 + other_radius);
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
