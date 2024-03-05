#ifndef PATHFINDING_GOAL_DISTANCEGOAL_H_
#define PATHFINDING_GOAL_DISTANCEGOAL_H_

#include <pf/Goal/GoalBase.hpp>
#include <pf/Goal/RangeGoal.hpp>
#include <pf/Vec3.hpp>

namespace pathfinding {

namespace goal {

template <class TPos, template <class> class TEval>
class DistanceGoal : public GoalBase<TPos> {
  using T = typename TPos::value_type;

 public:
  virtual bool isSuitableGoal(const TPos &pos) const override {
    return TEval<TPos>::eval(pos) <= dist_tol;
  }

  DistanceGoal(const TPos &_goalPos, const T &_dist_tol)
      : GoalBase<TPos>(_goalPos), dist_tol(_dist_tol) {}

  T dist_tol;
};

}  // namespace goal

};  // namespace pathfinding

#endif  // PATHFINDING_GOAL_DISTANCEGOAL_H_