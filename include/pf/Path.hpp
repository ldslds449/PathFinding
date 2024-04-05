// Copyright 2024 ldslds449

#ifndef INCLUDE_PF_PATH_HPP_
#define INCLUDE_PF_PATH_HPP_

#include <ostream>
#include <vector>

#include <pf/Vec3.hpp>

namespace pathfinding {

template <class TPos>
class Path {
 private:
  std::vector<TPos> nodes;

 public:
  friend std::ostream &operator<<(std::ostream &os, const Path<TPos> &path) {
    for (const auto &n : path.nodes) {
      os << n << std::endl;
    }
    return os;
  }

  inline void add(const TPos &pos) { nodes.push_back(pos); }
  inline void reverse() { std::reverse(nodes.begin(), nodes.end()); }
  inline std::size_t size() const { return nodes.size(); }
  inline const TPos &back() const { return nodes.back(); }
  inline void pop_back() { nodes.pop_back(); }
  inline const std::vector<TPos> &get() const { return nodes; }

  // only leave important point
  inline void refine(const std::size_t &max_interval) {
    std::vector<TPos> new_nodes;
    std::size_t interval = 0;
    for (std::size_t i = 0; i < nodes.size(); ++i) {
      if (i <= 1) {
        new_nodes.emplace_back(nodes[i]);
      } else {
        TPos prev_vec = nodes[i - 1] - nodes[i - 2];
        TPos now_vec = nodes[i] - nodes[i - 1];
        if (prev_vec == now_vec && interval + 1 < max_interval) {
          // ignore this point
          interval++;
        } else {
          // important point
          new_nodes.emplace_back(nodes[i]);
          interval = 0;
        }
      }
    }
    // exchange vector
    nodes.swap(new_nodes);
  }

  TPos &operator[](const std::size_t &idx) { return nodes[idx]; }
};

}  // namespace pathfinding

#endif  // INCLUDE_PF_PATH_HPP_
