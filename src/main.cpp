#include "ConstraintTable.h"
#include "SICBS.h"
#include "SIRRT.h"
#include "SharedEnv.h"

int main() {
  int num_of_agents = 10;
  int width = 10;
  int height = 10;
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
    iterations.emplace_back(1000);
    goal_sample_rates.emplace_back(10.0);
  }
  vector<shared_ptr<Obstacle>> obstacles;
  obstacles.emplace_back(make_shared<CircularObstacle>(5, 5, 2));

  SharedEnv env = SharedEnv(num_of_agents, width, height, start_points, goal_points, radii, max_expand_distances,
                            velocities, iterations, goal_sample_rates, obstacles);
  env.generateRandomInstance();
  ConstraintTable constraint_table(env, num_of_agents);
  Solution soluiton;
  for (int agent_id = 0; agent_id < num_of_agents; ++agent_id) {
    SIRRT sirrt(env, constraint_table, agent_id);
    auto path = sirrt.run();
    constraint_table.insertPathToConstraint(agent_id, path);
    soluiton.emplace_back(path);
  }
  saveSolution(soluiton, "../solution.txt");
  return 0;
}