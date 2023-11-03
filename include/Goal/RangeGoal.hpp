#ifndef PATHFINDING_GOAL_RANGEGOAL_H_
#define PATHFINDING_GOAL_RANGEGOAL_H_

#include "GoalBase.hpp"
#include "Vec3.hpp"

namespace pathfinding {

namespace goal {

template <class TPos>
class RangeGoal : public GoalBase<TPos> {
  using T = typename TPos::value_type;

 public:
  virtual bool isSuitableGoal(const TPos &pos) const override {
    return ((x_tol_neg < 0 && pos.x <= GoalBase<TPos>::goalPos.x) ||
            (x_tol_pos < 0 && GoalBase<TPos>::goalPos.x <= pos.x) ||
            ((GoalBase<TPos>::goalPos.x - x_tol_neg <= pos.x) &&
             (pos.x <= GoalBase<TPos>::goalPos.x + x_tol_pos))) &&
           ((y_tol_neg < 0 && pos.y <= GoalBase<TPos>::goalPos.y) ||
            (y_tol_pos < 0 && GoalBase<TPos>::goalPos.y <= pos.y) ||
            ((GoalBase<TPos>::goalPos.y - y_tol_neg <= pos.y) &&
             (pos.y <= GoalBase<TPos>::goalPos.y + y_tol_pos))) &&
           ((z_tol_neg < 0 && pos.z <= GoalBase<TPos>::goalPos.z) ||
            (z_tol_pos < 0 && GoalBase<TPos>::goalPos.z <= pos.z) ||
            ((GoalBase<TPos>::goalPos.z - z_tol_neg <= pos.z) &&
             (pos.z <= GoalBase<TPos>::goalPos.z + z_tol_pos)));
  }

  RangeGoal(const TPos &_goalPos, const T &_x_tol, const T &_y_tol,
            const T &_z_tol)
      : GoalBase<TPos>(_goalPos),
        x_tol_pos(_x_tol),
        x_tol_neg(_x_tol),
        y_tol_pos(_y_tol),
        y_tol_neg(_y_tol),
        z_tol_pos(_z_tol),
        z_tol_neg(_z_tol) {}

  RangeGoal(const TPos &_goalPos, const T &_x_tol_pos, const T &_y_tol_pos,
            const T &_z_tol_pos, const T &_x_tol_neg, const T &_y_tol_neg,
            const T &_z_tol_neg)
      : GoalBase<TPos>(_goalPos),
        x_tol_pos(_x_tol_pos),
        x_tol_neg(_x_tol_neg),
        y_tol_pos(_y_tol_pos),
        y_tol_neg(_y_tol_neg),
        z_tol_pos(_z_tol_pos),
        z_tol_neg(_z_tol_neg) {}

  T x_tol_pos, x_tol_neg;
  T y_tol_pos, y_tol_neg;
  T z_tol_pos, z_tol_neg;
};

}  // namespace goal

};  // namespace pathfinding

#endif  // PATHFINDING_GOAL_RANGEGOAL_H_