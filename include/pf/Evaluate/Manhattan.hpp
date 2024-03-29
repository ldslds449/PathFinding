// Copyright 2024 ldslds449

#ifndef INCLUDE_PF_EVALUATE_MANHATTAN_HPP_
#define INCLUDE_PF_EVALUATE_MANHATTAN_HPP_

#include <pf/Evaluate/EvaluateBase.hpp>

namespace pathfinding {

namespace eval {

class Manhattan final : public EvaluateBase<Manhattan> {
 public:
  template <class TPos>
  static CostT evalImpl(const TPos &pos, const TPos &target) {
    return static_cast<CostT>(pos.manhattanDist(target));
  }

  Manhattan() = delete;
};

}  // namespace eval

}  // namespace pathfinding

#endif  // INCLUDE_PF_EVALUATE_MANHATTAN_HPP_
