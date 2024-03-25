// Copyright 2024 ldslds449

#ifndef INCLUDE_PF_EVALUATE_CHEBYSHEV_HPP_
#define INCLUDE_PF_EVALUATE_CHEBYSHEV_HPP_

#include <algorithm>

#include <pf/Evaluate/EvaluateBase.hpp>

namespace pathfinding {

namespace eval {

class Chebyshev final : public EvaluateBase<Chebyshev> {
 public:
  template <class TPos>
  static CostT evalImpl(const TPos &pos, const TPos &target) {
    const TPos delta = (target - pos).abs();
    return delta.maxAxisVal();
  }

  Chebyshev() = delete;
};

}  // namespace eval

}  // namespace pathfinding

#endif  // INCLUDE_PF_EVALUATE_CHEBYSHEV_HPP_
