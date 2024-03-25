// Copyright 2024 ldslds449

#ifndef INCLUDE_PF_EVALUATE_OCTILE_HPP_
#define INCLUDE_PF_EVALUATE_OCTILE_HPP_

#include <algorithm>
#include <utility>

#include <pf/Evaluate/EvaluateBase.hpp>

namespace pathfinding {

namespace eval {

class Octile final : public EvaluateBase<Octile> {
 public:
  template <class TPos>
  static CostT evalImpl(const TPos &pos, const TPos &target) {
    constexpr double cost_3d = std::sqrt(3) - std::sqrt(2);
    constexpr double cost_2d = std::sqrt(2) - 1;
    const TPos delta = (target - pos).abs();

    // sort
    typename TPos::value_type val[3] = {delta.x, delta.y, delta.z};
    if (val[1] > val[2]) std::swap(val[1], val[2]);
    if (val[0] > val[1]) std::swap(val[0], val[1]);
    if (val[1] > val[2]) std::swap(val[1], val[2]);

    return (val[0] * cost_3d) + (val[1] * cost_2d) + val[2];
  }

  Octile() = delete;
};

}  // namespace eval

}  // namespace pathfinding

#endif  // INCLUDE_PF_EVALUATE_OCTILE_HPP_
