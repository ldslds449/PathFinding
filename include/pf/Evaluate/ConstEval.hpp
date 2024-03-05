// Copyright 2024 ldslds449

#ifndef INCLUDE_PF_EVALUATE_CONSTEVAL_HPP_
#define INCLUDE_PF_EVALUATE_CONSTEVAL_HPP_

#include <pf/Evaluate/EvaluateBase.hpp>

namespace pathfinding {

namespace eval {

template <int Val = 0>
class ConstEval final : public EvaluateBase<ConstEval<Val>> {
 public:
  template <class TPos>
  static CostT evalImpl(const TPos &pos, const TPos &target) {
    return static_cast<CostT>(Val);
  }

  ConstEval() = delete;
};

}  // namespace eval

}  // namespace pathfinding

#endif  // INCLUDE_PF_EVALUATE_CONSTEVAL_HPP_
