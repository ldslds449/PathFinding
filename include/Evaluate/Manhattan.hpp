#ifndef PATHFINDING_EVALUATE_MANHATTAN_H_
#define PATHFINDING_EVALUATE_MANHATTAN_H_

#include "EvaluateBase.hpp"

namespace pathfinding {

namespace eval {

class Manhattan final : public EvaluateBase<Manhattan> {
 public:
  template <class TPos>
  static int evalImpl(const TPos &pos, const TPos &target) {
    return pos.manhattanDist(target);
  };
};

}  // namespace eval

}  // namespace pathfinding

#endif  // PATHFINDING_EVALUATE_MANHATTAN_H_