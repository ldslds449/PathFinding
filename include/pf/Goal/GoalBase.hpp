#ifndef PATHFINDING_GOAL_GOALBASE_H_
#define PATHFINDING_GOAL_GOALBASE_H_

#include <vector>

namespace pathfinding {

namespace goal {

template <class TPos>
class GoalBase {
 public:
  using pos_type = TPos;

  virtual bool isSuitableGoal(const TPos &pos) const = 0;

  const TPos &getGoalPosition() const { return goalPos; }

 protected:
  GoalBase(const TPos &_goalPos) : goalPos(_goalPos) {}

  TPos goalPos;
};

namespace {
  template <typename... TGoals>
  using TDeducedPos = typename std::tuple_element<0, std::tuple<TGoals...>>::type::pos_type;
}

};  // namespace goal

};  // namespace pathfinding

#endif  // PATHFINDING_GOAL_GOALBASE_H_