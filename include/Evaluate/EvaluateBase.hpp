#ifndef PATHFINDING_EVALUATE_EVALUATEBASE_H_
#define PATHFINDING_EVALUATE_EVALUATEBASE_H_

#include "Vec3.hpp"

namespace pathfinding {

namespace eval {

template <class TDrived>
class EvaluateBase {
 public:
  template <class TVec3>
  inline static int eval(const Vec3<TVec3> &pos, const Vec3<TVec3> &target) {
    return TDrived::evalImpl(pos, target);
  };

 private:
  EvaluateBase() {}
  friend TDrived;
};

}  // namespace eval

}  // namespace pathfinding

#endif  // PATHFINDING_EVALUATE_EVALUATEBASE_H_