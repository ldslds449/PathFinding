#ifndef PATHFINDING_GOAL_RANGEGOAL_H_
#define PATHFINDING_GOAL_RANGEGOAL_H_

#include "GoalBase.hpp"
#include "Vec3.hpp"

namespace pathfinding {

namespace goal {

template <class TPos = Position>
class RangeGoal : public GoalBase<TPos> {
 public:
  virtual bool isSuitableGoal(const TPos &pos) const final override {
    return (x_tol < 0 || ((GoalBase<TPos>::goalPos.x - x_tol <= pos.x) &&
                          (pos.x <= GoalBase<TPos>::goalPos.x + x_tol))) &&
           (y_tol < 0 || ((GoalBase<TPos>::goalPos.y - y_tol <= pos.y) &&
                          (pos.y <= GoalBase<TPos>::goalPos.y + y_tol))) &&
           (z_tol < 0 || ((GoalBase<TPos>::goalPos.z - z_tol <= pos.z) &&
                          (pos.z <= GoalBase<TPos>::goalPos.z + z_tol)));
  }

  virtual std::vector<TPos> getPossibleGoalPosition() const final override {
    using Tval = typename TPos::value_type;
    std::vector<TPos> positions;
    for (Tval i = -x_tol; i <= x_tol; ++i) {
      for (Tval j = -y_tol; j <= y_tol; ++j) {
        for (Tval k = -z_tol; k <= z_tol; ++k) {
          positions.emplace_back(GoalBase<TPos>::goalPos.offset(i, j, k));
        }
      }
    }
    return positions;
  }

  RangeGoal(const TPos &_goalPos, const typename TPos::value_type &_x_tol,
            const typename TPos::value_type &_y_tol,
            const typename TPos::value_type &_z_tol)
      : GoalBase<TPos>(_goalPos), x_tol(_x_tol), y_tol(_y_tol), z_tol(_z_tol) {}

  typename TPos::value_type x_tol, y_tol, z_tol;
};

}  // namespace goal

};  // namespace pathfinding

#endif  // PATHFINDING_GOAL_RANGEGOAL_H_