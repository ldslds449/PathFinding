// Copyright 2024 ldslds449

#ifndef INCLUDE_PF_EVALUATE_EVALUATEBASE_HPP_
#define INCLUDE_PF_EVALUATE_EVALUATEBASE_HPP_

#include <pf/Type.hpp>
#include <pf/Vec3.hpp>

namespace pathfinding {

namespace eval {

template <class TDrived>
class EvaluateBase {
 public:
  template <class TPos>
  inline static CostT eval(const TPos &pos, const TPos &target) {
    return TDrived::evalImpl(pos, target);
  }

  template <class TPos>
  inline static CostT eval(const TPos &target) {
    return TDrived::evalImpl(TPos(0, 0, 0), target);
  }

 private:
  EvaluateBase() {}
  friend TDrived;
};

}  // namespace eval

}  // namespace pathfinding

#endif  // INCLUDE_PF_EVALUATE_EVALUATEBASE_HPP_
