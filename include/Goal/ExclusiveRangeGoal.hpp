#ifndef PATHFINDING_GOAL_EXCLUSIVERANGEGOAL_H_
#define PATHFINDING_GOAL_EXCLUSIVERANGEGOAL_H_

#include "GoalBase.hpp"
#include "RangeGoal.hpp"
#include "Vec3.hpp"

namespace pathfinding {

namespace goal {

template <class TPos>
class ExclusiveRangeGoal : public RangeGoal<TPos> {
  using T = typename TPos::value_type;

 public:
  virtual bool isSuitableGoal(const TPos &pos) const override {
    return !RangeGoal<TPos>::isSuitableGoal(pos);
  }

  ExclusiveRangeGoal(const TPos &_goalPos, const T &_x_tol, const T &_y_tol,
                     const T &_z_tol)
      : RangeGoal<TPos>(_goalPos, _x_tol, _y_tol, _z_tol) {}

  ExclusiveRangeGoal(const TPos &_goalPos, const T &_x_tol_pos,
                     const T &_y_tol_pos, const T &_z_tol_pos,
                     const T &_x_tol_neg, const T &_y_tol_neg,
                     const T &_z_tol_neg)
      : RangeGoal<TPos>(_goalPos, _x_tol_pos, _y_tol_pos, _z_tol_pos,
                        _x_tol_neg, _y_tol_neg, _z_tol_neg) {}
};

}  // namespace goal

};  // namespace pathfinding

#endif  // PATHFINDING_GOAL_EXCLUSIVERANGEGOAL_H_