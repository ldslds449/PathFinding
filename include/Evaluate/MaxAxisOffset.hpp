#ifndef PATHFINDING_EVALUATE_MAXASIXOFFSET_H_
#define PATHFINDING_EVALUATE_MAXASIXOFFSET_H_

#include "EvaluateBase.hpp"

namespace pathfinding {

namespace eval {

class MaxAxisOffset final : public EvaluateBase<MaxAxisOffset> {
 public:
  template <class TPos>
  static int evalImpl(const TPos &pos, const TPos &target) {
    return pos.maxAxisOffset(target);
  };
};

}  // namespace eval

}  // namespace pathfinding

#endif  // PATHFINDING_EVALUATE_MAXASIXOFFSET_H_