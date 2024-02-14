#ifndef PATHFINDING_EVALUATE_CONSTEVAL_H_
#define PATHFINDING_EVALUATE_CONSTEVAL_H_

#include "EvaluateBase.hpp"

namespace pathfinding {

namespace eval {

template <int Val>
class ConstEval final : public EvaluateBase<ConstEval<Val>> {
 public:
  template <class TPos>
  static CostT evalImpl(const TPos &pos, const TPos &target) {
    return static_cast<CostT>(Val);
  };

  ConstEval() = delete;
};

}  // namespace eval

}  // namespace pathfinding

#endif  // PATHFINDING_EVALUATE_CONSTEVAL_H_