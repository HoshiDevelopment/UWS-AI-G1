#ifndef _GRAPH_HPP_
#define _GRAPH_HPP_

#include <numeric>
#include <unordered_map>
#include <vector>
#include <utility>        // std::pair
#include <cmath>
#include <functional>

// We need this class template specialisation of std::hash before declaring
// std::unordered_map variables involving a std::pair; such as edge_t.

namespace std
{
  template <typename T, typename U>
  struct hash<std::pair<T,U>>
  {
    std::size_t operator()(const std::pair<T,U>& n) const
    {
      return std::hash<T>{}(n.first) ^ std::hash<U>{}(n.second);
    }
  };
}

using node_t  = char;
using edge_t  = std::pair<node_t, node_t>;
using coord_t = ai::Vector2;
std::unordered_map<node_t, coord_t> node_info;
std::unordered_map<edge_t, double>  edge_info;

const float line_thickness = 5.0f;
const float node_radius    = line_thickness * 2.0f;

inline double heuristic(node_t, node_t);
#include "implementation.hpp" // in redblob-games

// neighbors and cost are needed by a_star_search
struct Graph
{
  // std::unordered_map<node_t, std::pair<coord_t, std::vector<node_t>>> ?
  // That would remove the need for node_info. Less readable?
  std::unordered_map<node_t, std::vector<node_t>> nodes; // node + its n'bours

  std::vector<node_t> neighbors(node_t id)
  {
    return nodes[id];
  }

  double cost(node_t current, node_t next)
  {
    return edge_info[{current,next}];
  }
};

inline double heuristic(node_t next, node_t goal)
{
  const coord_t& npos = node_info[next];
  const coord_t& gpos = node_info[goal];
  return std::abs(npos.x - gpos.x) + std::abs(npos.y - gpos.y);
}

#endif // _GRAPH_HPP_
