#ifndef PATHFINDING_GOAL_COMBINEGOAL_H_
#define PATHFINDING_GOAL_COMBINEGOAL_H_

#include <tuple>

#include "GoalBase.hpp"
#include "Vec3.hpp"

namespace pathfinding {

namespace goal {

template <typename... TGoals>
class CombineGoal : public GoalBase<TDeducedPos<TGoals...>> {
  using TPos = TDeducedPos<TGoals...>;

 public:
  virtual bool isSuitableGoal(const TPos &pos) const override {
    auto checkOneGoal = [&](auto &&...goal) {
      return (goal.isSuitableGoal(pos) && ...);
    };
    return std::apply(checkOneGoal, goals);
  }

  CombineGoal(const TPos &_goalPos, const TGoals &..._goals)
      : GoalBase<TPos>(_goalPos), goals(_goals...) {}

  const std::tuple<TGoals...> goals;
};

}  // namespace goal

};  // namespace pathfinding

#endif  // PATHFINDING_GOAL_COMBINEGOAL_H_