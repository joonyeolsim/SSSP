#include "RRT.h"
#include "SharedEnv.h"
#include "common.h"
#include "HLNode.h"

class SSSP {
public:
  int num_of_sampling = 6;
  double random_sampling_rate = 1;
  double threshold = 1;
  double decay_rate = 0.99;
  vector<RRT> roadmap_constructors;
  SharedEnv& env;
  typedef vector<shared_ptr<LLNode>> Roadmap;
  vector<Roadmap> roadmaps;
  default_random_engine gen;
  std::uniform_real_distribution<> dis_100;

  SSSP(SharedEnv& env) : env(env), dis_100(0.0, 100.0), gen(env.seed) {
    roadmap_constructors.reserve(env.num_of_robots);
    for (int agent_id = 0; agent_id < env.num_of_robots; ++agent_id) {
      roadmap_constructors.emplace_back(agent_id, env);
    }
  }
  ~SSSP() = default;
  Solution run();
  void initRoadmaps();
  bool agentConstrained(int agent_id, const Point& from_point, const Point& to_point,
                        const vector<shared_ptr<LLNode>>& nodes, double radius) const;
  void dijkstra(Roadmap roadmap, shared_ptr<LLNode> source_node);
  Solution updatePath(const shared_ptr<HLNode>& goal_node) const;
};