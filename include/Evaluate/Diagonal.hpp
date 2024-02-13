#ifndef PATHFINDING_EVALUATE_DIAGONAL_H_
#define PATHFINDING_EVALUATE_DIAGONAL_H_

#include "EvaluateBase.hpp"

namespace pathfinding {

namespace eval {

class Diagonal final : public EvaluateBase<Diagonal> {
 public:
  template <class TPos>
  static CostT evalImpl(const TPos &pos, const TPos &target) {
    return static_cast<CostT>(pos.diagonalDist(target));
  };

  Diagonal() = delete;
};

}  // namespace eval

}  // namespace pathfinding

#endif  // PATHFINDING_EVALUATE_DIAGONAL_H_