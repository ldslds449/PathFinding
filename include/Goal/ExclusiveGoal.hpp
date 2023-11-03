#ifndef PATHFINDING_GOAL_EXCLUSIVEGOAL_H_
#define PATHFINDING_GOAL_EXCLUSIVEGOAL_H_

#include "GoalBase.hpp"
#include "Vec3.hpp"

namespace pathfinding {

namespace goal {

template <class TGoal>
class ExclusiveGoal : public GoalBase<TDeducedPos<TGoal>> {
  using TPos = TDeducedPos<TGoal>;

 public:
  virtual bool isSuitableGoal(const TPos &pos) const override {
    return !goal.isSuitableGoal(pos);
  }

  ExclusiveGoal(const TGoal &_goal)
      : GoalBase<TPos>(_goal.getGoalPosition()), goal(_goal) {}

  TGoal goal;
};

}  // namespace goal

};  // namespace pathfinding

#endif  // PATHFINDING_GOAL_EXCLUSIVEGOAL_H_