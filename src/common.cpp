#include "common.h"

void savePath(const Path& path, const string& filename) {
  ofstream file(filename, ios::out);
  if (!file.is_open()) {
    cerr << "Error opening file: " << filename << endl;
    return;
  }

  for (const auto& point_time : path) {
    const auto& point = get<0>(point_time);
    double time = get<1>(point_time);
    file << "(" << get<0>(point) << "," << get<1>(point) << "," << time << ")->";
  }
  file << endl;
  file.close();
}

void saveSolution(const Solution& solution, const string& filename) {
  ofstream file(filename, ios::out);
  if (!file.is_open()) {
    cerr << "Error opening file: " << filename << endl;
    return;
  }

  for (size_t i = 0; i < solution.size(); ++i) {
    file << "Agent " << i << ":";
    for (const auto& point_time : solution[i]) {
      const auto& point = get<0>(point_time);
      double time = get<1>(point_time);
      file << "(" << get<0>(point) << "," << get<1>(point) << "," << time << ")->";
    }
    file << endl;
  }

  file.close();
}

void saveData(double cost, double makespan, double duration, const string& filename) {
  ofstream file(filename, ios::out);
  if (!file.is_open()) {
    cerr << "Error opening file: " << filename << endl;
    return;
  }

  file << cost << "," << makespan << "," << duration << endl;

  file.close();
}

double calculateDistance(Point point1, Point point2) {
  return sqrt(pow(get<0>(point1) - get<0>(point2), 2) + pow(get<1>(point1) - get<1>(point2), 2));
}