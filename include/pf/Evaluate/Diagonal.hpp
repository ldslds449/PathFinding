// Copyright 2024 ldslds449

#ifndef INCLUDE_PF_EVALUATE_DIAGONAL_HPP_
#define INCLUDE_PF_EVALUATE_DIAGONAL_HPP_

#include <pf/Evaluate/EvaluateBase.hpp>

namespace pathfinding {

namespace eval {

class Diagonal final : public EvaluateBase<Diagonal> {
 public:
  template <class TPos>
  static CostT evalImpl(const TPos &pos, const TPos &target) {
    return static_cast<CostT>(pos.diagonalDist(target));
  }

  Diagonal() = delete;
};

}  // namespace eval

}  // namespace pathfinding

#endif  // INCLUDE_PF_EVALUATE_DIAGONAL_HPP_
