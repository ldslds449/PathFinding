#ifndef PATHFINDING_GOAL_GOALBASE_H_
#define PATHFINDING_GOAL_GOALBASE_H_

namespace pathfinding {

namespace goal {

template <class TPos>
class GoalBase {
 public:
  virtual bool isGoal(const TPos &pos) const = 0;

  const TPos &getGoalPosition() const { return goalPos; }

 protected:
  GoalBase(const TPos &_goalPos) : goalPos(_goalPos) {}

  TPos goalPos;
};

};  // namespace goal

};  // namespace pathfinding

#endif  // PATHFINDING_GOAL_GOALBASE_H_