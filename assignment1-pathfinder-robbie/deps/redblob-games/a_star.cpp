#include "implementation.hpp"

int main() {
  GridWithWeights grid = make_diagram4();
  GridLocation start{ 1, 4 };
  GridLocation goal{ 8, 5 };
  std::unordered_map<GridLocation, GridLocation> came_from;
  std::unordered_map<GridLocation, double> cost_so_far;
  a_star_search(grid, start, goal, came_from, cost_so_far);
  draw_grid(grid, 2, nullptr, &came_from);
  std::cout << '\n';
  draw_grid(grid, 3, &cost_so_far, nullptr);
  std::cout << '\n';
  std::vector<GridLocation> path = reconstruct_path(start, goal, came_from);
  draw_grid(grid, 3, nullptr, nullptr, &path);
}
