#ifndef PATHFINDING_EVALUATE_EUCLIDEAN_H_
#define PATHFINDING_EVALUATE_EUCLIDEAN_H_

#include "EvaluateBase.hpp"

namespace pathfinding {

namespace eval {

class Euclidean final : public EvaluateBase<Euclidean> {
 public:
  template <class TPos>
  static int evalImpl(const TPos &pos, const TPos &target) {
    return pos.squaredEuclideanDist(target);
  };
};

}  // namespace eval

}  // namespace pathfinding

#endif  // PATHFINDING_EVALUATE_EUCLIDEAN_H_