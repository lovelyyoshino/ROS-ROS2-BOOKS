/**
 * @file main.cpp
 * @author vss2sn
 * @brief Main file where all the algorithms can be used and tested.
 */

#include <iostream>
#include <random>

#include "path_planning/a_star.hpp"
#include "path_planning/ant_colony.hpp"
#include "path_planning/d_star_lite.hpp"
#include "path_planning/dijkstra.hpp"
#include "path_planning/genetic_algorithm.hpp"
#include "path_planning/jump_point_search.hpp"
#include "path_planning/lpa_star.hpp"
#include "path_planning/rrt.hpp"
#include "path_planning/rrt_star.hpp"

int main() {
  constexpr int n = 21;
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
  PrintGrid(grid);

  // Store points after algorithm has run
  std::vector<std::vector<int>> main_grid = grid;

  // Variables for RRT and RRTStar
  constexpr double threshold = 2;
  constexpr int max_iter_x_factor = 20;

  // Variables for Ant Colony Optimization
  constexpr int n_ants = 10;
  constexpr int iterations = 50;
  constexpr double alpha = 1;
  constexpr double beta = 0.7;
  constexpr double evap_rate = 0.3;
  constexpr double Q = 10;

  // Variables for Genetic Algorithm
  constexpr int generations = 10000;
  constexpr int popsize = 30;
  constexpr double c = 1.05;
  constexpr bool shorten_chromosome = true;
  constexpr int path_length_x_factor = 4;

  // Resetting grid
  // Create object for the algorithm
  // Run algorithm
  // Print the final grid using the path_vector

  // clang-format off
  std::cout << "--------------------------------------------------------------" << '\n'
            << "--------------------- ALGORITHM: DIJKSTRA ---------------------" << '\n'
            << "--------------------------------------------------------------" << '\n';
  // clang-format on
  grid = main_grid;
  Dijkstra dijkstra(grid);
  {
    const auto [path_found, path_vector] = dijkstra.Plan(start, goal);
    PrintPath(path_vector, start, goal, grid);
  }

  // clang-format off
  std::cout << "--------------------------------------------------------" << '\n'
            << "--------------------- ALGORITHM: A* ---------------------" << '\n'
            << "--------------------------------------------------------" << '\n';
  // clang-format on
  grid = main_grid;
  AStar a_star(grid);
  {
    const auto [path_found, path_vector] = a_star.Plan(start, goal);
    PrintPath(path_vector, start, goal, grid);
  }

  // clang-format off
  std::cout << "-----------------------------------------------------------------------" << '\n';
  std::cout << "--------------------- ALGORITHM: Jump Point Search ---------------------" << '\n';
  std::cout << "-----------------------------------------------------------------------" << '\n';
  // clang-format on
  grid = main_grid;
  JumpPointSearch jump_point_search(grid);
  {
    const auto [path_found, path_vector] = jump_point_search.Plan(start, goal);
    PrintPath(path_vector, start, goal, grid);
  }

  // clang-format off
  std::cout << "--------------------------------------------------------------------------" << '\n'
            << "--------------------- ALGORITHM: Lifelong Planning A* ---------------------" << '\n'
            << "--------------------------------------------------------------------------" << '\n';
  // clang-format on
  grid = main_grid;
  LPAStar lpa_star(grid);
  {
    const auto [path_found, path_vector] = lpa_star.Plan(start, goal);
    PrintPath(path_vector, start, goal, grid);
  }
  // NOTE: The PrintPath function will not be to show the updated grid for the
  //       live run of LPA* including obstacles discovered during execution.


  // clang-format off
  std::cout << "---------------------------------------------------------" << '\n'
            << "--------------------- ALGORITHM: RRT ---------------------" << '\n'
            << "---------------------------------------------------------" << '\n';
  // clang-format on
  grid = main_grid;
  RRT rrt(grid);
  rrt.SetParams(threshold, max_iter_x_factor);
  {
    const auto [path_found, path_vector] = rrt.Plan(start, goal);
    PrintPath(path_vector, start, goal, grid);
  }

  // clang-format off
  std::cout << "----------------------------------------------------------" << '\n';
  std::cout << "--------------------- ALGORITHM: RRT* ---------------------" << '\n';
  std::cout << "----------------------------------------------------------" << '\n';
  // clang-format on
  grid = main_grid;
  RRTStar rrt_star(grid);
  rrt_star.SetParams(threshold, max_iter_x_factor);
  {
    const auto [path_found, path_vector] = rrt_star.Plan(start, goal);
    PrintPath(path_vector, start, goal, grid);
  }

  // clang-format off
  std::cout << "-------------------------------------------------------------" << '\n'
            << "--------------------- ALGORITHM: D* Lite ---------------------" << '\n'
            << "-------------------------------------------------------------" << '\n';
  // clang-format on
  grid = main_grid;
  DStarLite d_star_lite(grid);
  {
    const auto [path_found, path_vector] = d_star_lite.Plan(start, goal);
    PrintPath(path_vector, start, goal, grid);
  }
  // NOTE: The PrintPath function will not be to show the updated grid for the
  //       live run of D* Lite including obstacles discovered during execution.

  // clang-format off
  std::cout << "-----------------------------------------------------------------------------" << '\n'
            << "--------------------- ALGORITHM: Ant Colony Optimization ---------------------" << '\n'
            << "-----------------------------------------------------------------------------" << '\n';
  // clang-format on
  grid = main_grid;
  AntColony ant_colony(grid);
  ant_colony.SetParams(n_ants, alpha, beta, evap_rate, iterations, Q);
  {
    const auto [path_found, path_vector] = ant_colony.Plan(start, goal);
    PrintPath(path_vector, start, goal, grid);
  }

  // clang-format off
  std::cout << "-----------------------------------------------------------------------" << '\n'
            << "--------------------- ALGORITHM: Genetic Algorithm ---------------------" << '\n'
            << "-----------------------------------------------------------------------" << '\n';
  // clang-format on
  grid = main_grid;
  GeneticAlgorithm genetic_algorithm(grid);
  genetic_algorithm.SetParams(generations, popsize, c, shorten_chromosome,
    static_cast<int>(path_length_x_factor * start.h_cost_));
  {
    const auto [path_found, path_vector] = genetic_algorithm.Plan(start, goal);
    PrintPath(path_vector, start, goal, grid);
  }

  return 0;
}
