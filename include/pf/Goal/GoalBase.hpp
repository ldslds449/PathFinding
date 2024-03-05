// Copyright 2024 ldslds449

#ifndef INCLUDE_PF_GOAL_GOALBASE_HPP_
#define INCLUDE_PF_GOAL_GOALBASE_HPP_

#include <tuple>
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
  explicit GoalBase(const TPos &_goalPos) : goalPos(_goalPos) {}

  TPos goalPos;
};

template <typename... TGoals>
using TDeducedPos =
    typename std::tuple_element<0, std::tuple<TGoals...>>::type::pos_type;

};  // namespace goal

};  // namespace pathfinding

#endif  // INCLUDE_PF_GOAL_GOALBASE_HPP_
