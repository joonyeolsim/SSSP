#include <SSSP.h>

#include "RRT.h"
#include "SharedEnv.h"

int main(int argc, char* argv[]) {
  string mapname;
  string obs;
  string robotnum;
  string testnum;
  for (int i = 1; i < argc; ++i) {
    if (strcmp(argv[i], "-m") == 0 && i + 1 < argc) {
      mapname = argv[i + 1];
    } else if (strcmp(argv[i], "-o") == 0 && i + 1 < argc) {
      obs = argv[i + 1];
    } else if (strcmp(argv[i], "-r") == 0 && i + 1 < argc) {
      robotnum = argv[i + 1];
    } else if (strcmp(argv[i], "-t") == 0 && i + 1 < argc) {
      testnum = argv[i + 1];
    }
  }

  string benchmarkPath = "benchmark/" + mapname + "_" + obs + "/agents" + robotnum + "/" + mapname + "_" + obs + "_" +
                         robotnum + "_" + testnum + ".yaml";
  string solutionPath = "solution/" + mapname + "_" + obs + "/agents" + robotnum + "/" + mapname + "_" + obs + "_" +
                        robotnum + "_" + testnum + "_solution.txt";
  string dataPath = "data/" + mapname + "_" + obs + "/agents" + robotnum + "/" + mapname + "_" + obs + "_" + robotnum +
                    "_" + testnum + "_data.txt";
  YAML::Node config = YAML::LoadFile(benchmarkPath);

  vector<shared_ptr<Obstacle>> obstacles;
  for (size_t i = 0; i < config["obstacles"].size(); ++i) {
    if (mapname == "CircleEnv") {
      auto center = config["obstacles"][i]["center"].as<std::vector<double>>();
      auto radius = config["obstacles"][i]["radius"].as<double>();
      obstacles.emplace_back(make_shared<CircularObstacle>(center[0], center[1], radius));
    } else {
      auto center = config["obstacles"][i]["center"].as<std::vector<double>>();
      auto height = config["obstacles"][i]["height"].as<double>();
      auto width = config["obstacles"][i]["width"].as<double>();
      obstacles.emplace_back(make_shared<RectangularObstacle>(center[0], center[1], width, height));
    }
  }
  vector<Point> start_points;
  vector<Point> goal_points;
  for (size_t i = 0; i < config["startPoints"].size(); ++i) {
    auto start = config["startPoints"][i].as<std::vector<double>>();
    auto goal = config["goalPoints"][i].as<std::vector<double>>();
    start_points.emplace_back(start[0], start[1]);
    goal_points.emplace_back(goal[0], goal[1]);
  }

  int num_of_agents = config["agentNum"].as<int>();
  int width = 40;
  int height = 40;
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
    iterations.emplace_back(1000);
    goal_sample_rates.emplace_back(10.0);
  }

  SharedEnv env = SharedEnv(num_of_agents, width, height, start_points, goal_points, radii, max_expand_distances,
                            velocities, iterations, goal_sample_rates, obstacles);
  Solution soluiton;
  auto start = std::chrono::high_resolution_clock::now();

  // SI-CBS
  SSSP sssp(env);
  soluiton = sssp.run();

  auto stop = std::chrono::high_resolution_clock::now();
  chrono::duration<double, std::ratio<1>> duration = stop - start;

  cout << "sum of cost: " << sssp.sum_of_costs << endl;
  cout << "makespan: " << sssp.makespan << endl;
  cout << "computation time: " << duration.count() << endl;
  saveSolution(soluiton, solutionPath);
  saveData(sssp.sum_of_costs, sssp.makespan, duration.count(), dataPath);

  return 0;
}