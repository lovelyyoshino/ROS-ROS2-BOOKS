/**
 * @file lpa_star.cpp
 * @author vss2sn
 * @brief Contains the LPAStar class
 */

#include "path_planning/lpa_star.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>
#include <tuple>

#ifdef BUILD_INDIVIDUAL
#include <random>
#endif  // BUILD_INDIVIDUAL

constexpr int pause_time = 500;  // milliseconds

void LPAStar::SetDynamicObstacles(
    const bool create_random_obstacles,
    const std::unordered_map<int, std::vector<Node>>&
        time_discovered_obstacles) {
  create_random_obstacles_ = create_random_obstacles;
  time_discovered_obstacles_ = time_discovered_obstacles;
}

bool LPAStar::IsObstacle(const Node& n) const { return grid_[n.x_][n.y_] == 1; }

double LPAStar::H(const Node& n1, const Node& n2) {
  return std::sqrt(std::pow(n1.x_ - n2.x_, 2) + std::pow(n1.y_ - n2.y_, 2));
}

std::vector<Node> LPAStar::GetNeighbours(const Node& u) const {
  std::vector<Node> neighbours;
  for (const auto& m : motions_) {
    if (const auto neighbour = u + m;
        !checkOutsideBoundary(neighbour, grid_.size())) {
      neighbours.push_back(neighbour);
    }
  }
  return neighbours;
}

std::vector<Node> LPAStar::GetPred(const Node& u) const {
  return GetNeighbours(u);
}

std::vector<Node> LPAStar::GetSucc(const Node& u) const {
  return GetNeighbours(u);
}

double LPAStar::C(const Node& s1, const Node& s2) const {
  if (IsObstacle(s1) || IsObstacle(s2)) {
    return std::numeric_limits<double>::max();
  }
  const Node delta{s2.x_ - s1.x_, s2.y_ - s1.y_};
  return std::find_if(std::begin(motions_), std::end(motions_),
                      [&delta](const Node& motion) {
                        return CompareCoordinates(motion, delta);
                      })
      ->cost_;
}

Key LPAStar::CalculateKey(const Node& s) const {
  return Key{std::min(g_[s.x_][s.y_], rhs_[s.x_][s.y_]) + H(s, goal_),
             std::min(g_[s.x_][s.y_], rhs_[s.x_][s.y_])};
}

std::vector<std::vector<double>> LPAStar::CreateGrid() {
  return std::vector<std::vector<double>>(
      n_, std::vector<double>(n_, std::numeric_limits<double>::max()));
}

void LPAStar::Initialize() {
  motions_ = GetMotion();
  time_step_ = 0;
  U_.clear();
  rhs_ = CreateGrid();
  g_ = CreateGrid();
  rhs_[start_.x_][start_.y_] = 0;
  U_.insert(NodeKeyPair{start_, CalculateKey(start_)});
}

void LPAStar::UpdateVertex(const Node& u) {
  if (grid_[u.x_][u.y_] == 0) {
    grid_[u.x_][u.y_] = 2;
  }
  if (!CompareCoordinates(u, start_)) {
    rhs_[u.x_][u.y_] = std::numeric_limits<double>::max();
    const auto predecessors = GetPred(u);
    for (const auto& sprime : predecessors) {
      rhs_[u.x_][u.y_] =
          std::min(rhs_[u.x_][u.y_], g_[sprime.x_][sprime.y_] + C(sprime, u));
    }
  }
  if (U_.isElementInStruct({u, {}})) {
    U_.remove(NodeKeyPair{u, Key()});
  }
  if (rhs_[u.x_][u.y_] != g_[u.x_][u.y_]) {
    U_.insert(NodeKeyPair{u, CalculateKey(u)});
  }
}

void LPAStar::ComputeShortestPath() {
  while ((!U_.empty() && U_.top().key < CalculateKey(goal_)) ||
         (rhs_[goal_.x_][goal_.y_] != g_[goal_.x_][goal_.y_])) {
    const Node u = U_.top().node;
    U_.pop();
    if (g_[u.x_][u.y_] > rhs_[u.x_][u.y_]) {
      g_[u.x_][u.y_] = rhs_[u.x_][u.y_];
      for (const auto& s : GetSucc(u)) {
        UpdateVertex(s);
      }
    } else {
      g_[u.x_][u.y_] = std::numeric_limits<double>::max();
      for (const auto& s : GetSucc(u)) {
        UpdateVertex(s);
      }
      UpdateVertex(u);
    }
  }
}

std::vector<Node> LPAStar::DetectChanges() {
  std::vector<Node> obstacles;
  if (time_discovered_obstacles_.find(time_step_) !=
      time_discovered_obstacles_.end()) {
    const auto discovered_obstacles_at_time =
        time_discovered_obstacles_[time_step_];
    for (const auto& discovered_obstacle_at_time :
         discovered_obstacles_at_time) {
      if (!((start_.x_ == discovered_obstacle_at_time.x_ &&
             start_.y_ == discovered_obstacle_at_time.y_) ||
            (goal_.x_ == discovered_obstacle_at_time.x_ &&
             goal_.y_ == discovered_obstacle_at_time.y_))) {
        grid_[discovered_obstacle_at_time.x_][discovered_obstacle_at_time.y_] =
            1;
        obstacles.push_back(discovered_obstacle_at_time);
      }
    }
  }
  if (create_random_obstacles_ && rand() > 1.0 / static_cast<double>(n_)) {
    const int x = rand() % n_;
    const int y = rand() % n_;
    if (!((start_.x_ == x && start_.y_ == y) ||
          (goal_.x_ == x && goal_.y_ == y))) {
      grid_[x][y] = 1;
      obstacles.emplace_back(Node(x, y));
    }
  }
  return obstacles;
}

void LPAStar::ClearPathDisplay(const std::vector<Node>& path) {
  for (const auto& node : path) {
    if (grid_[node.x_][node.y_] ==
        3) {  // Don't update if it's a discovered obstacle
      grid_[node.x_][node.y_] = 2;  // It's been explored, but no longer a path
    }
  }
  grid_[start_.x_][start_.y_] = 3;
}

void LPAStar::UpdatePathDisplay(const std::vector<Node>& path) {
  for (const auto& node : path) {
    grid_[node.x_][node.y_] = 3;
  }
}

std::vector<Node> LPAStar::GetNewPath() {
  auto current = goal_;
  std::vector<Node> path;
  path.push_back(current);
  while (!CompareCoordinates(current, start_)) {
    auto min_cost = std::numeric_limits<double>::max();
    Node min_node;
    for (const auto& motion : motions_) {
      auto new_node = current + motion;
      if (!checkOutsideBoundary(new_node, n_) &&
          g_[new_node.x_][new_node.y_] < min_cost) {
        min_cost = g_[new_node.x_][new_node.y_];
        min_node = new_node;
      }
    }
    min_node.id_ = min_node.x_ * n_ + min_node.y_;
    path.back().pid_ = min_node.id_;
    current = min_node;
    path.push_back(current);
  }

  const auto path_cost = path.back().cost_;
  for (auto& node : path) {
    node.cost_ = path_cost - node.cost_;
  }
  path.back().pid_ = path.back().id_;
  return path;
}

std::tuple<bool, std::vector<Node>> LPAStar::Plan(const Node& start,
                                                  const Node& goal) {
  grid_ = original_grid_;
  start_ = start;
  goal_ = goal;
  std::vector<Node> path;
  path.push_back(start_);
  grid_[start_.x_][start_.y_] = 3;
  PrintGrid(grid_);
  Initialize();
  while (time_step_ < max_time_step_) {
    ComputeShortestPath();
    if (g_[goal_.x_][goal_.y_] == std::numeric_limits<double>::max()) {
      ClearPathDisplay(path);
      PrintGrid(grid_);
      std::cout << "No path exists" << '\n';
      return {false, {}};
    }
    ClearPathDisplay(path);
    path = GetNewPath();
    UpdatePathDisplay(path);
    PrintGrid(grid_);
    time_step_++;

#ifndef RUN_TESTS
    std::this_thread::sleep_for(std::chrono::milliseconds(pause_time));
#endif  // RUN_TESTS

    if (const auto changed_nodes = DetectChanges(); !changed_nodes.empty()) {
      for (const auto node : changed_nodes) {
        UpdateVertex(node);
      }
    }
  }
  for (const auto& node : path) {
    node.PrintStatus();
  }
  PrintGrid(grid_);
  return {true, path};
}

#ifdef BUILD_INDIVIDUAL
/**
 * @brief Script main function. Generates start and end nodes as well as grid,
 * then creates the algorithm object and calls the main algorithm function.
 * @return 0
 */
int main() {
  constexpr int n = 11;
  std::vector<std::vector<int>> grid(n, std::vector<int>(n, 0));
  MakeGrid(grid);

  std::random_device rd;   // obtain a random number from hardware
  std::mt19937 eng(rd());  // seed the generator
  std::uniform_int_distribution<int> distr(0, n - 1);  // define the range

  Node start(distr(eng), distr(eng), 0, 0, 0, 0);
  Node goal(distr(eng), distr(eng), 0, 0, 0, 0);

  start.id_ = start.x_ * n + start.y_;
  start.pid_ = start.x_ * n + start.y_;
  goal.id_ = goal.x_ * n + goal.y_;
  start.h_cost_ = abs(start.x_ - goal.x_) + abs(start.y_ - goal.y_);

  // Make sure start and goal are not obstacles and their ids are correctly
  // assigned.
  grid[start.x_][start.y_] = 0;
  grid[goal.x_][goal.y_] = 0;

  start.PrintStatus();
  goal.PrintStatus();

  PrintGrid(grid);

  const bool create_random_obstacles = false;
  const std::unordered_map<int, std::vector<Node>> time_discovered_obstacles{
      {1, {Node(1, 1)}},
      {2, {Node(2, 2)}},
      {3, {Node(5, 5)}},
      {4,
       {Node(6, 6), Node(7, 7), Node(8, 8), Node(9, 9), Node(10, 10),
        Node(7, 6)}}};

  LPAStar lpa_star(grid);
  lpa_star.SetDynamicObstacles(create_random_obstacles,
                               time_discovered_obstacles);
  const auto [found_path, path_vector] = lpa_star.Plan(start, goal);
  PrintPath(path_vector, start, goal, grid);
  return 0;
}
#endif  // BUILD_INDIVIDUAL
