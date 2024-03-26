// Copyright 2024 ldslds449

#ifndef INCLUDE_PF_EVALUATE_EUCLIDEAN_HPP_
#define INCLUDE_PF_EVALUATE_EUCLIDEAN_HPP_

#include <cmath>

#include <pf/Evaluate/EvaluateBase.hpp>

namespace pathfinding {

namespace eval {

class Euclidean final : public EvaluateBase<Euclidean> {
 public:
  template <class TPos>
  static CostT evalImpl(const TPos &pos, const TPos &target) {
    return std::sqrt(pos.squaredEuclideanDist(target));
  }

  Euclidean() = delete;
};

}  // namespace eval

}  // namespace pathfinding

#endif  // INCLUDE_PF_EVALUATE_EUCLIDEAN_HPP_
