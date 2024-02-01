#include <yaml-cpp/yaml.h>

#include "ConstraintTable.h"
#include "SICBS.h"
#include "SIRRT.h"
#include "SharedEnv.h"

// Function to split a string by a delimiter
std::vector<std::string> split(const std::string &s, char delimiter) {
  std::vector<std::string> tokens;
  std::string token;
  std::istringstream tokenStream(s);
  while (std::getline(tokenStream, token, delimiter)) {
    tokens.push_back(token);
  }
  return tokens;
}

// Function to parse a single point
Point parsePoint(const std::string &s) {
  std::vector<std::string> pointData = split(s, ',');
  double x = std::stod(pointData[0]);
  double y = std::stod(pointData[1]);
  return std::make_tuple(x, y);
}

// Updated function to parse a path
Path parsePath(const std::string &s) {
  std::vector<std::string> pathData = split(s, '-');  // Splitting using "->" instead of ">"
  Path path;
  for (const std::string &pointString : pathData) {
    size_t start = pointString.find('(') + 1;
    size_t end = pointString.find(')', start);
    if (start == std::string::npos || end == std::string::npos) {
      continue;  // Invalid format, skip this entry
    }
    std::string pointData = pointString.substr(start, end - start);
    Point point = parsePoint(pointData);

    size_t zStart = pointString.rfind(',', end) + 1;
    if (zStart == std::string::npos || zStart >= end) {
      continue;  // Invalid format, skip this entry
    }
    std::string zData = pointString.substr(zStart, end - zStart);

    double z;
    try {
      z = std::stod(zData);
    } catch (const std::invalid_argument &e) {
      std::cerr << "Invalid argument for stod: " << zData << '\n';
      continue;  // Invalid z value, skip this entry
    }

    path.push_back(std::make_tuple(point, z));
  }
  return path;
}

// Updated function to read and parse the file
Solution parseFile(const std::string &fileName) {
  std::ifstream file(fileName);
  Solution solution;
  std::string line;
  while (std::getline(file, line)) {
    size_t labelEnd = line.find(':');  // Find the end of the agent label
    if (labelEnd != std::string::npos) {
      std::string pathString = line.substr(labelEnd + 1);  // Extract the path string after the label
      Path path = parsePath(pathString);
      solution.push_back(path);
    }
  }
  return solution;
}

int main(int argc, char *argv[]) {
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
  // srand(time(0));
  for (int i = 0; i < num_of_agents; ++i) {
    // double randomValue = 0.25 + static_cast<double>(rand()) / (static_cast<double>(RAND_MAX / (1 - 0.25)));
    radii.emplace_back(0.5);
    max_expand_distances.emplace_back(5.0);
    velocities.emplace_back(0.5);
    thresholds.emplace_back(0.01);
    iterations.emplace_back(1500);
    goal_sample_rates.emplace_back(10.0);
  }

  SharedEnv env = SharedEnv(num_of_agents, width, height, start_points, goal_points, radii, max_expand_distances,
                            velocities, iterations, goal_sample_rates, obstacles);
  // env.generateRandomInstance();
  ConstraintTable constraint_table(env);
  Solution soluiton;
  auto start = std::chrono::high_resolution_clock::now();

  // SI-CBS
  // SICBS sicbs(env, constraint_table);
  // soluiton = sicbs.run();

  // SI-RRT PP
  double sum_of_costs = 0.0;
  double makespan = 0.0;
  // for (int agent_id = 0; agent_id < num_of_agents; ++agent_id) {
  //   SIRRT sirrt(agent_id, env, constraint_table);
  //   auto path = sirrt.run();
  //   while (path.empty()) {
  //     // cout << "Replanning for agent " << agent_id << endl;
  //     path = sirrt.run();
  //   }
  //   // cout << "Agent " << agent_id << " found a solution" << endl;
  //   soluiton.emplace_back(path);
  //   sum_of_costs += get<1>(path.back());
  //   makespan = max(makespan, get<1>(path.back()));
  //   constraint_table.path_table[agent_id] = path;
  // }

  // for (int agent_id = 0; agent_id < 30; ++agent_id) {
  //   SIRRT sirrt(agent_id, env, constraint_table);
  //   auto path = sirrt.run();
  //   while (path.empty()) {
  //     cout << "Replanning for agent " << agent_id << endl;
  //     path = sirrt.run();
  //   }
  //   cout << "Agent " << agent_id << " found a solution" << endl;
  //   soluiton.emplace_back(path);
  //   sum_of_costs += get<1>(path.back());
  //   makespan = max(makespan, get<1>(path.back()));
  //   constraint_table.path_table[agent_id] = path;
  // }
  std::string fileName = "dynamicFree.txt";
  Solution solution = parseFile(fileName);
  for (int agent_id = 0; agent_id < solution.size(); ++agent_id) {
    constraint_table.path_table[agent_id] = solution[agent_id];
  }

  // Test SI-RRT* cost
  start_points[30] = make_tuple(1.0, 1.0);
  goal_points[30] = make_tuple(39.0, 39.0);
  goal_sample_rates[30] = 5.0;
  env.start_points[30] = start_points[30];
  env.goal_points[30] = goal_points[30];
  env.iterations[30] = 1000000;
  env.goal_sample_rates[30] = goal_sample_rates[30];
  SIRRT sirrt(30, env, constraint_table);
  auto path = sirrt.run();
  soluiton.emplace_back(path);

  auto stop = std::chrono::high_resolution_clock::now();
  chrono::duration<double, std::ratio<1>> duration = stop - start;

  cout << "sum of cost: " << sum_of_costs << endl;
  cout << "makespan: " << makespan << endl;
  cout << "computation time: " << duration.count() << endl;
  saveSolution(soluiton, solutionPath);
  saveData(sum_of_costs, makespan, duration.count(), dataPath);

  // cout << "sum of cost: " << sicbs.sum_of_costs << endl;
  // cout << "makespan: " << sicbs.makespan << endl;
  // cout << "computation time: " << duration.count() << endl;
  // saveSolution(soluiton, solutionPath);
  // saveData(sicbs.sum_of_costs, sicbs.makespan, duration.count(), dataPath);

  return 0;
}