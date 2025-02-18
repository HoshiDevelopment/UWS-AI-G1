#include "vec.hpp"
#include "draw-triangle-pro.hpp"
#include "raylib-cpp.hpp"
#include "graph.hpp"
#include "graph-utils.hpp"
#include <iostream>
#include <vector>
#include <string>
#include <unordered_map>

std::vector<node_t> astar_pathfind(const Graph& g, node_t start, node_t goal)
{
  std::unordered_map<node_t, node_t> came_from;
  std::unordered_map<node_t, double> cost_so_far;
  a_star_search(g, start, goal, came_from, cost_so_far);
  std::vector<node_t> path = reconstruct_path(start, goal, came_from);
  return path;
}

unsigned int path_cost(const std::vector<node_t>& path)
{
  double dcost = 0;
  if (path.size() >= 2) // then we have some lines to draw
  {
    const int num_edges = path.size() - 1;
    for (int i = 0; i < num_edges; i++)
    {
      dcost = dcost + edge_info[std::pair{path[i], path[i+1]}];
    }
  }

  return static_cast<unsigned int>(dcost);
}

int main()
{
    //Screen size
  const int w{ 2280/2 }, h{ 1620/2 }, half_w{ w/2 }, half_h{ h/2 }, gap{ w/8 };
  raylib::Window window{ w, h, "Pathfinder" };

  SetTargetFPS(60);


  Graph g;
  add_node(g, 'A', { half_w - gap, half_h });
  add_node(g, 'B', { half_w, half_h });
  add_node(g, 'C', { half_w, half_h - gap });
  add_node(g, 'D', { half_w, half_h + gap });
  add_node(g, 'E', { half_w + gap, half_h + gap });
  add_node(g, 'F', { half_w + gap, half_h });
  add_node(g, 'G', { half_w + (2 * gap), half_h - gap });

  //testing nodes
  //left
  add_node(g, 'H', { half_w - (2 * gap), half_h });
  //right
  add_node(g, 'I', { half_w + (2 * gap), half_h });
  //down
  add_node(g, 'J', { half_w, half_h + (2 * gap) });
  //up
  add_node(g, 'K', { half_w, half_h - (2 * gap) });

  add_double_edge(g, 'A', 'B');
  add_double_edge(g, 'B', 'C');
  add_double_edge(g, 'B', 'D');
  add_double_edge(g, 'C', 'A');
  add_double_edge(g, 'D', 'E');
  add_double_edge(g, 'D', 'A');
  add_double_edge(g, 'E', 'B');
  add_double_edge(g, 'B', 'F');
  add_double_edge(g, 'C', 'F');
  add_double_edge(g, 'C', 'G');
  add_double_edge(g, 'F', 'G');

  int t{60}; // time
  std::vector<node_t> player_path{};
  node_t start = 'A';
  node_t end   = 'G';
  int tokens{2000}, score{}, high_score{}; // try with more/less tokens?

  while (!window.ShouldClose()) // Detect window close button or ESC key
  {
    BeginDrawing();

    ClearBackground(LIGHTGRAY);

    //Numbers are X, Y, font size
    //Need to find out what type of time their after
    DrawText(TextFormat("Score: %08i", score), 10, 10, 20, RED);
    DrawText(TextFormat("Tokens: %08i", tokens), 190, 10, 20, ORANGE);
    DrawText(TextFormat("High_score: %08i", high_score), 380, 10, 20, PURPLE);
    DrawText(TextFormat("Timer: %02.02f ms", GetTime() * 1000, t), 610, 10, 20, BLACK);
    DrawText(TextFormat("T: %08i", t), 780, 10, 20, PURPLE);

    draw_graph(g);

    if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON))
    {
      if (auto opt = get_nearby_node(GetMousePosition()))
      {
        // *opt is a node_t
          DrawText(TextFormat("Score: %08i", score), 110, 110, 20, RED);
          add_node(g, 'Z', { half_w, half_h });
      }
    }

    EndDrawing();
  }

  return 0;
}

