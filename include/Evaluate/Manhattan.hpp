#ifndef PATHFINDING_EVALUATE_MANHATTAN_H_
#define PATHFINDING_EVALUATE_MANHATTAN_H_

#include "EvaluateBase.hpp"

namespace pathfinding {

namespace eval {

class Manhattan final : public EvaluateBase<Manhattan> {
 public:
  template <class TPos>
  static CostT evalImpl(const TPos &pos, const TPos &target) {
    return static_cast<CostT>(pos.manhattanDist(target));
  };

  Manhattan() = delete;
};

}  // namespace eval

}  // namespace pathfinding

#endif  // PATHFINDING_EVALUATE_MANHATTAN_H_