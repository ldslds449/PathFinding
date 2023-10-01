#ifndef PATHFINDING_EVALUATE_EVALUATEBASE_H_
#define PATHFINDING_EVALUATE_EVALUATEBASE_H_

#include "Vec3.hpp"

namespace pathfinding {

namespace eval {

template <class TDrived>
class EvaluateBase {
 public:
  template <class TPos>
  inline static U64 eval(const TPos &pos, const TPos &target) {
    return TDrived::evalImpl(pos, target);
  };

  template <class TPos>
  inline static U64 eval(const TPos &target) {
    return TDrived::evalImpl(TPos(), target);
  };

 private:
  EvaluateBase() {}
  friend TDrived;
};

}  // namespace eval

}  // namespace pathfinding

#endif  // PATHFINDING_EVALUATE_EVALUATEBASE_H_