#ifndef PATHFINDING_GOAL_COMBINEGOAL_H_
#define PATHFINDING_GOAL_COMBINEGOAL_H_

#include <tuple>

#include "GoalBase.hpp"
#include "Vec3.hpp"

namespace pathfinding {

namespace goal {

template <typename TMainGoal, typename... TGoals>
class CombineGoal : public GoalBase<TDeducedPos<TMainGoal>> {
  using TPos = TDeducedPos<TMainGoal>;

 public:
  virtual bool isSuitableGoal(const TPos &pos) const override {
    auto checkOneGoal = [&](auto &&...goal) {
      return (goal.isSuitableGoal(pos) && ...);
    };
    return std::apply(checkOneGoal, goals);
  }

  CombineGoal(const TMainGoal &main_goal, const TGoals &..._goals)
      : GoalBase<TPos>(main_goal), goals(main_goal, _goals...) {}

  const std::tuple<TMainGoal, TGoals...> goals;
};

}  // namespace goal

};  // namespace pathfinding

#endif  // PATHFINDING_GOAL_COMBINEGOAL_H_