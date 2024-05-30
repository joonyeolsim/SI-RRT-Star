#include "common.h"

namespace fs = std::filesystem;

void openFile(ofstream& file, const string& filename) {
  file.open(filename, ios::out);
  if (!file.is_open()) {
    cerr << "Error opening file: " << filename << endl;
  }
}

void writePath(ofstream& file, const Path& path) {
  for (const auto& point_time : path) {
    const auto& point = get<0>(point_time);
    double time = get<1>(point_time);
    file << "(" << get<0>(point) << "," << get<1>(point) << "," << time << ")->";
  }
  file << endl;
}

void savePath(const Path& path, const string& filename) {
  ofstream file;
  openFile(file, filename);
  if (!file.is_open()) return;

  writePath(file, path);
  file.close();

  if (!fs::exists(filename)) {
    cerr << "Failed to write file: " << filename << endl;
  }
}

void saveSolution(const Solution& solution, const string& filename) {
  ofstream file;
  openFile(file, filename);
  if (!file.is_open()) return;

  for (size_t i = 0; i < solution.size(); ++i) {
    file << "Agent " << i << ":";
    writePath(file, solution[i]);
  }
  file.close();

  if (!fs::exists(filename)) {
    cerr << "Failed to write file: " << filename << endl;
  }
}

void saveData(double cost, double makespan, double duration, const string& filename) {
  ofstream file;
  openFile(file, filename);
  if (!file.is_open()) return;

  file << cost << "," << makespan << "," << duration << endl;
  file.close();

  if (!fs::exists(filename)) {
    cerr << "Failed to write file: " << filename << endl;
  }
}

double calculateDistance(Point point1, Point point2) {
  return hypot(get<0>(point1) - get<0>(point2), get<1>(point1) - get<1>(point2));
}

bool checkConflicts(const Solution &solution, const vector<double> &radii) {
  for (size_t i = 0; i < solution.size(); ++i) {
    const Path &path1 = solution[i];
    double radius1 = radii[i];

    for (size_t j = i + 1; j < solution.size(); ++j) {
      const Path &path2 = solution[j];
      double radius2 = radii[j];
      size_t max_length = max(path1.size(), path2.size());

      for (size_t t = 0; t < max_length; ++t) {
        Point point1 = (t < path1.size()) ? get<0>(path1[t]) : get<0>(path1.back());
        Point point2 = (t < path2.size()) ? get<0>(path2[t]) : get<0>(path2.back());

        if (calculateDistance(point1, point2) <= (radius1 + radius2)) {
          return true; // Conflict detected
        }
      }
    }
  }
  return false; // No conflict detected
}

