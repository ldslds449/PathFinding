// Copyright 2024 ldslds449

#ifndef INCLUDE_PF_GOAL_COMBINEGOAL_HPP_
#define INCLUDE_PF_GOAL_COMBINEGOAL_HPP_

#include <tuple>

#include <pf/Goal/GoalBase.hpp>
#include <pf/Vec3.hpp>

namespace pathfinding {

namespace goal {

template <typename TMainGoal, typename... TGoals>
class CombineGoal : public GoalBase<TDeducedPos<TMainGoal>> {
  using TPos = TDeducedPos<TMainGoal>;

 public:
  bool isSuitableGoal(const TPos &pos) const override {
    auto checkOneGoal = [&](auto &&...goal) {
      return (goal.isSuitableGoal(pos) && ...);
    };
    return std::apply(checkOneGoal, goals);
  }

  CombineGoal(const TMainGoal &main_goal, const TGoals &..._goals)
      : GoalBase<TPos>(main_goal.getGoalPosition()),
        goals(main_goal, _goals...) {}

  const std::tuple<TMainGoal, TGoals...> goals;
};

}  // namespace goal

};  // namespace pathfinding

#endif  // INCLUDE_PF_GOAL_COMBINEGOAL_HPP_
