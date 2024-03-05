// Copyright 2024 ldslds449

#ifndef INCLUDE_PF_GOAL_DISTANCEGOAL_HPP_
#define INCLUDE_PF_GOAL_DISTANCEGOAL_HPP_

#include <pf/Goal/GoalBase.hpp>
#include <pf/Goal/RangeGoal.hpp>
#include <pf/Vec3.hpp>

namespace pathfinding {

namespace goal {

template <class TPos, template <class> class TEval>
class DistanceGoal : public GoalBase<TPos> {
  using T = typename TPos::value_type;

 public:
  bool isSuitableGoal(const TPos &pos) const override {
    return TEval<TPos>::eval(pos) <= dist_tol;
  }

  DistanceGoal(const TPos &_goalPos, const T &_dist_tol)
      : GoalBase<TPos>(_goalPos), dist_tol(_dist_tol) {}

  T dist_tol;
};

}  // namespace goal

};  // namespace pathfinding

#endif  // INCLUDE_PF_GOAL_DISTANCEGOAL_HPP_
