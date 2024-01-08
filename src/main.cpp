#include <yaml-cpp/yaml.h>

#include "ConstraintTable.h"
#include "SICBS.h"
#include "SIRRT.h"
#include "SharedEnv.h"

int main(int argc, char* argv[]) {
  string benchmarkPath;
  string testnum;
  for (int i = 1; i < argc; ++i) {
    if (strcmp(argv[i], "-n") == 0 && i + 1 < argc) {
      testnum = argv[++i];
    }
  }

  benchmarkPath = "maps/map_tetris_404020_" + testnum + ".yaml";
  YAML::Node config = YAML::LoadFile(benchmarkPath);

  vector<shared_ptr<Obstacle>> obstacles;
  // for (size_t i = 0; i < config["obstacles"].size(); ++i) {
  //   auto center = config["obstacles"][i]["center"].as<std::vector<double>>();
  //   auto height = config["obstacles"][i]["height"].as<double>();
  //   auto width = config["obstacles"][i]["width"].as<double>();
  //   obstacles.emplace_back(make_shared<RectangularObstacle>(center[0], center[1], width, height));
  // }

  int num_of_agents = 30;
  int width = 40;
  int height = 40;
  vector<Point> start_points;
  vector<Point> goal_points;
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
  env.generateRandomInstance();
  ConstraintTable constraint_table(env);
  Solution soluiton;
  auto start = std::chrono::high_resolution_clock::now();

  // SI-CBS
  SICBS sicbs(env, constraint_table);
  soluiton = sicbs.run();
  cout << "solution cost: " << sicbs.sum_of_costs << endl;

  // SI-RRT PP
  // double sum_of_costs = 0.0;
  // for (int agent_id = 0; agent_id < num_of_agents; ++agent_id) {
  //   SIRRT sirrt(agent_id, env, constraint_table);
  //   auto path = sirrt.run();
  //   constraint_table.insertPathToConstraint(agent_id, path);
  //   soluiton.emplace_back(path);
  //   sum_of_costs += get<1>(path.back());
  // }
  // cout << "solution cost: " << sum_of_costs << endl;

  auto stop = std::chrono::high_resolution_clock::now();
  chrono::duration<double, std::ratio<1>> duration = stop - start;
  cout << "Time taken by function: " << duration.count() << " seconds" << endl;
  saveSolution(soluiton, "solution" + testnum + ".txt");

  std::ofstream outfile("radii" + testnum + ".txt");  // 파일 쓰기 객체 생성
  if (!outfile.is_open()) {
    std::cerr << "Failed to open file for writing." << std::endl;
    return 1;
  }

  for (const auto& radius : radii) {
    outfile << radius << std::endl;  // 각 반지름 값을 파일에 씁니다.
  }

  outfile.close(); // 파일 쓰기 완료 후 파일 닫기

  return 0;
}