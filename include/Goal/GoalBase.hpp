#ifndef PATHFINDING_GOAL_GOALBASE_H_
#define PATHFINDING_GOAL_GOALBASE_H_

#include <vector>

namespace pathfinding {

namespace goal {

template <class TPos>
class GoalBase {
 public:
  virtual bool isSuitableGoal(const TPos &pos) const = 0;

  const TPos &getGoalPosition() const { return goalPos; }

  virtual std::vector<TPos> getPossibleGoalPosition() const = 0;

 protected:
  GoalBase(const TPos &_goalPos) : goalPos(_goalPos) {}

  TPos goalPos;
};

};  // namespace goal

};  // namespace pathfinding

#endif  // PATHFINDING_GOAL_GOALBASE_H_