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
  const int w{ 2880/2 }, h{ 1620/2 }, half_w{ w/2 }, half_h{ h/2 }, gap{ w/8 };
  raylib::Window window{ w, h, "Pathfinder" };
  
// laoding the sound effect task4 -jack
Sound clickSound = LoadSound("raylib/resources/audio/sound.wav");  

  SetTargetFPS(60);

  Graph g;
  add_node(g, 'A', { half_w - gap, half_h });
  add_node(g, 'B', { half_w, half_h });
  add_node(g, 'C', { half_w, half_h - gap });
  add_node(g, 'D', { half_w, half_h + gap });
  add_node(g, 'E', { half_w + gap, half_h + gap });
  add_node(g, 'F', { half_w + gap, half_h });
  add_node(g, 'G', { half_w + (2 * gap), half_h - gap });


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

  //Task 14 - Robbie

  node_t start = 'A' + GetRandomValue(0, 6);
  node_t end;
  //node_t end   = 'A' + GetRandomValue(0, 6);

  do {
      end = 'A' + GetRandomValue(0, 6);

  } while (end == start);
  std::vector<node_t> playerpath;
  player_path.push_back(start);
  player_path.push_back(end);

  int tokens{2000}, score{}, high_score{}; // try with more/less tokens?
  int frames = 0;
  

  // Display the player's path
  std::cout << "Player's Path: ";
  for (char Goals : player_path) {
      std::cout << Goals << ' ';
  }
  std::cout << std::endl;
 

  while (!window.ShouldClose()) // Detect window close button or ESC key
  {
    BeginDrawing();

    ClearBackground(LIGHTGRAY);

    //Zac T1
    //Numbers are X, Y, font size
    DrawText(TextFormat("Score: %08i", score), 10, 10, 20, RED);
    DrawText(TextFormat("Tokens: %08i", tokens), 190, 10, 20, ORANGE);
    DrawText(TextFormat("High_score: %08i", high_score), 380, 10, 20, PURPLE);
    DrawText(TextFormat("Timer: %02.02f ms", GetTime() * 1000, t), 610, 10, 20, BLACK);
    DrawText(TextFormat("T: %08i", t), 780, 10, 20, PURPLE);
    //T1

    draw_graph(g);
    //Task 5 - Robbie
    // Draw the player's path in correct order
    for (size_t i = 1; i < player_path.size(); i++) {
        node_t current = player_path[i - 1];
        node_t next = player_path[i];

        // Ensure we're only drawing valid edges
        if (edge_info.find({ current, next }) != edge_info.end()) {
            Vector2 startPos = node_info[current];
            Vector2 endPos = node_info[next];

            DrawLineEx(startPos, endPos, 5, BLUE); // Draw thick blue line for path
        }
    }

    //Task 2 - Robbie
    DrawCircle(node_info[start].x, node_info[start].y, 10, GREEN);
    DrawCircle(node_info[end].x, node_info[end].y, 10, RED);

    // Task 12 - Robbie
    frames++; // increase the frames int by 1 every frame
    if (frames == 60) {
        t--; //Decrease time every 60 frames
        frames = 0;
        
    }

    // task 13 - jack
    // check if game is over 
    if (t <= 0 || tokens < 0) {

      if (score > high_score) {
        high_score = score; // will update high score if needed
      }

      // reset score, tokens levl etc
      score = 0;
      tokens = 2000;

      // restart level
      player_path.clear();
      player_path.push_back(start);

      t = 60; // reset timer

    }


    if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON))
    {
       
      if (auto opt = get_nearby_node(GetMousePosition()))
      {
          //writes in the cmd window what node it's on with each click
          DrawText(TextFormat("Score: %08i", score), 10, 30, 20, RED);
          std::cout << "Player's Path: ";
          for (char Live : player_path) {
              std::cout << Live << ' ';
          }
          std::cout << std::endl;
         
         
          
       
        // TASK 6 - Jack

        std::vector<node_t> start_neighbors = g.neighbors(start); // neighbours start of the node

        if(player_path.empty())
        {
          
           
         
          if (std::find(start_neighbors.begin(), start_neighbors.end(), *opt) != start_neighbors.end())
          {
        // *opt is a node_t
          // Zac T3

          player_path.push_back(*opt); // add node if valid neighbour
          // playsound
          // T3

          }
        }
        else // task 7, 8, 9, 10 - charlie
        {
          node_t last_node = player_path.back(); // get the last node in path 

          // TASK 10
          if (*opt == end && last_node == end) // reached end node
          {
            // calculates the ideal node, get cost
            std::vector<node_t> ideal_path = astar_pathfind(g, start, end);
            int ideal_cost = path_cost(ideal_path);

            // update score 
            score += ideal_cost;

            // give extra tokens
            tokens += 500;

            // reset game 
            player_path.clear();
            start = 'A' + GetRandomValue(0, 6);
            end = 'A' + GetRandomValue(0, 6);
            player_path.push_back(start);
          } // TASK 10 END




          // TASK 9:
          if (*opt == last_node && player_path.size() > 1) // not the start node
          {
            // refund tokens
            node_t prev_node = player_path[player_path.size() - 2];
            tokens += edge_info[{prev_node, last_node}];

            player_path.pop_back(); // remove node
          } // END OF TASK 9


          if (edge_info.find({ last_node, *opt }) != edge_info.end()) // check new node is neighbor of the last node
          {

            int cost = edge_info[{last_node, * opt}]; // get edge cost
            tokens -= cost; 

            // end of task 8
            player_path.push_back(*opt);
          }
        }
      }
    }

    EndDrawing();
  }

  return 0;
}

