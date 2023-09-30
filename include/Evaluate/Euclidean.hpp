#ifndef PATHFINDING_EVALUATE_EUCLIDEAN_H_
#define PATHFINDING_EVALUATE_EUCLIDEAN_H_

#include "EvaluateBase.hpp"

namespace pathfinding {

namespace eval {

class Euclidean final : public EvaluateBase<Euclidean> {
 public:
  template <class TPos>
  static U64 evalImpl(const TPos &pos, const TPos &target) {
    return pos.template squaredEuclideanDist<U64>(target);  // squared does not effect the relation for all pairs
  };
};

}  // namespace eval

}  // namespace pathfinding

#endif  // PATHFINDING_EVALUATE_EUCLIDEAN_H_