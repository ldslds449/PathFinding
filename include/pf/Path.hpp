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

  TPos &operator[](const std::size_t &idx) { return nodes[idx]; }
};

}  // namespace pathfinding

#endif  // INCLUDE_PF_PATH_HPP_
