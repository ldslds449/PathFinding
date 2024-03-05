// Copyright 2024 ldslds449

#ifndef INCLUDE_PF_GOAL_EXCLUSIVEGOAL_HPP_
#define INCLUDE_PF_GOAL_EXCLUSIVEGOAL_HPP_

#include <pf/Goal/GoalBase.hpp>
#include <pf/Vec3.hpp>

namespace pathfinding {

namespace goal {

template <class TGoal>
class ExclusiveGoal : public GoalBase<TDeducedPos<TGoal>> {
  using TPos = TDeducedPos<TGoal>;

 public:
  bool isSuitableGoal(const TPos &pos) const override {
    return !goal.isSuitableGoal(pos);
  }

  explicit ExclusiveGoal(const TGoal &_goal)
      : GoalBase<TPos>(_goal.getGoalPosition()), goal(_goal) {}

  TGoal goal;
};

}  // namespace goal

};  // namespace pathfinding

#endif  // INCLUDE_PF_GOAL_EXCLUSIVEGOAL_HPP_
