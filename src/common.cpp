#include "common.h"

void savePath(const Path& path, const string& filename) {
  ofstream file(filename, ios::out);
  if (!file.is_open()) {
    cerr << "Error opening file: " << filename << endl;
    return;
  }

  for (const auto& point : path) {
    file << "(" << get<0>(point) << "," << get<1>(point) << ")->";
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
    for (const auto& point : solution[i]) {
      file << "(" << get<0>(point) << "," << get<1>(point) << ")->";
    }
    file << endl;
  }

  file.close();
}

double calculateDistance(Point point1, Point point2) {
  return sqrt(pow(get<0>(point1) - get<0>(point2), 2) + pow(get<1>(point1) - get<1>(point2), 2));
}