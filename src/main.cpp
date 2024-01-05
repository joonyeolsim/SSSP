#include <SSSP.h>

#include "RRT.h"
#include "SharedEnv.h"

int main() {
  int num_of_agents = 30;
  int width = 32;
  int height = 32;
  vector<Point> start_points;
  vector<Point> goal_points;
  vector<double> radii;
  vector<double> max_expand_distances;
  vector<double> velocities;
  vector<double> thresholds;
  vector<int> iterations;
  vector<double> goal_sample_rates;
  for (int i = 0; i < num_of_agents; ++i) {
    radii.emplace_back(0.5);
    max_expand_distances.emplace_back(5.0);
    velocities.emplace_back(0.5);
    thresholds.emplace_back(0.01);
    iterations.emplace_back(500);
    goal_sample_rates.emplace_back(10.0);
  }
  vector<shared_ptr<Obstacle>> obstacles;
  // obstacles.emplace_back(make_shared<CircularObstacle>(5, 10, 1.5));
  // obstacles.emplace_back(make_shared<CircularObstacle>(10, 10, 2));
  // obstacles.emplace_back(make_shared<CircularObstacle>(15, 15, 1));
  // obstacles.emplace_back(make_shared<CircularObstacle>(25, 25, 3));

  SharedEnv env = SharedEnv(num_of_agents, width, height, start_points, goal_points, radii, max_expand_distances,
                            velocities, iterations, goal_sample_rates, obstacles);
  env.generateRandomInstance();
  Solution soluiton;
  auto start = std::chrono::high_resolution_clock::now();

  // SI-CBS
  SSSP sssp(env);
  soluiton = sssp.run();

  auto stop = std::chrono::high_resolution_clock::now();
  chrono::duration<double, std::ratio<1>> duration = stop - start;
  cout << "Time taken by function: " << duration.count() << " seconds" << endl;
  saveSolution(soluiton, "solution.txt");

  std::ofstream outfile("radii.txt");  // 파일 쓰기 객체 생성
  if (!outfile.is_open()) {
    std::cerr << "Failed to open file for writing." << std::endl;
    return 1;
  }

  for (const auto &radius : radii) {
    outfile << radius << std::endl;  // 각 반지름 값을 파일에 씁니다.
  }

  outfile.close(); // 파일 쓰기 완료 후 파일 닫기

  return 0;
}