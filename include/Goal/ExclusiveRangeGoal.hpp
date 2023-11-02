#ifndef PATHFINDING_GOAL_EXCLUSIVERANGEGOAL_H_
#define PATHFINDING_GOAL_EXCLUSIVERANGEGOAL_H_

#include "GoalBase.hpp"
#include "RangeGoal.hpp"
#include "Vec3.hpp"

namespace pathfinding {

namespace goal {

template <class TPos>
class ExclusiveRangeGoal : public RangeGoal<TPos> {
 public:
  virtual bool isSuitableGoal(const TPos &pos) const override {
    return !RangeGoal<TPos>::isSuitableGoal(pos);
  }

  ExclusiveRangeGoal(const TPos &_goalPos, const typename TPos::value_type &_x_tol,
            const typename TPos::value_type &_y_tol,
            const typename TPos::value_type &_z_tol)
      : RangeGoal<TPos>(_goalPos, _x_tol, _y_tol, _z_tol) {}
};

}  // namespace goal

};  // namespace pathfinding

#endif  // PATHFINDING_GOAL_EXCLUSIVERANGEGOAL_H_