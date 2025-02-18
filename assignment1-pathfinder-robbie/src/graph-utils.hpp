#ifndef _GRAPH_UTILS_HPP_
#define _GRAPH_UTILS_HPP_

#include <numeric>   // std::accumulate
#include <cmath>     // std::sin
#include <optional>  // std::optional
#include <random>    // std::random_device, std::mt19937
#include <algorithm> // std::sample
#include <iterator>  // std::back_inserter
#include "graph.hpp"

auto num_nodes(const Graph& g)
{
  return std::distance(g.nodes.cbegin(), g.nodes.cend());
}

auto num_edges(const Graph& g)
{
  auto f = [](const auto&x, const auto& y) { return x + y.second.size(); };
  return std::accumulate(g.nodes.cbegin(), g.nodes.cend(), std::size_t{0}, f);
}

void add_node(Graph& g, const node_t& n, const coord_t& xy)
{
  g.nodes[n] = {};
  node_info[n] = xy;
}

// Add two edges to the Graph g: one between n1 and n2; and the other
// between n2 and n1. The costs are set at cost1 and cost2 respectively.
void add_double_edge(Graph& g, const node_t& n1, const node_t& n2,
                     const double cost1, const double cost2)
{
  g.nodes[n1].push_back(n2);
  g.nodes[n2].push_back(n1);
  edge_info[{n1,n2}] = cost1;
  edge_info[{n2,n1}] = cost2;
}

// Add two edges to the Graph g: one between n1 and n2; and the other
// between n2 and n1. The edge costs default to the euclidean distance.
void add_double_edge(Graph& g, const node_t& n1, const node_t& n2)
{
  const double cost = (node_info[n1]-node_info[n2]).length();
  add_double_edge(g, n1, n2, cost, cost);
}

std::optional<node_t> get_nearby_node(coord_t coord2)
{
  for (const auto& [node, coord1] : node_info)
  {
    if ((coord1 - coord2).length() < node_radius)
    {
      return node;
    }
  }

  return std::nullopt;
}

void draw_cost_line(const node_t& n1, const node_t& n2, const unsigned int cost)
{
  const coord_t& coord_n1 = node_info[n1];
  const coord_t& coord_n2 = node_info[n2];

  float s = std::sin(static_cast<float>(GetTime()) * 10.0f);

  DrawLineEx(coord_n1, coord_n2, line_thickness, GRAY);

  const char* text = TextFormat("%d", cost);
  coord_t mid = (coord_n1 + coord_n2) / 2.0f;
  const Font font = GetFontDefault();
  const int spacing = 2;
  const float font_size = 45 + (s * 4);
  coord_t tsz = MeasureTextEx(font, text, font_size, spacing);
//  DrawTextEx(font, text, mid - (tsz / 2), font_size + s, spacing, DARKGREEN);
  const float rot = s;
  DrawTextPro(font, text, mid, tsz/2, rot, font_size, spacing, BROWN);
}

void draw_graph(Graph& g)
{
  for (const auto& [edge, dcost] : edge_info)
  {
    const auto& [n1, n2] = edge;
    const unsigned int cost = static_cast<unsigned int>(dcost);
    draw_cost_line(n1, n2, cost);
  }

  for (const auto& [node, coord] : node_info)
  {
    DrawCircleV(coord, node_radius, BLACK);
  }
}

#endif // _GRAPH_UTILS_HPP_
