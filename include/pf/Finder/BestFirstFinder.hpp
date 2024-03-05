// Copyright 2024 ldslds449

#ifndef INCLUDE_PF_FINDER_BESTFIRSTFINDER_HPP_
#define INCLUDE_PF_FINDER_BESTFIRSTFINDER_HPP_

#include <pf/Finder/AstarFinder.hpp>

namespace pathfinding {

template <class TDrived, class TPos = Position,
          class Dummy1 = eval::ConstEval<>,
          class TEstimateEval = eval::Manhattan, class... Dummy2>
class BestFirstFinder
    : public AstarFinder<
          BestFirstFinder<TDrived, TPos, Dummy1, TEstimateEval, Dummy2...>,
          TPos, eval::ConstEval<>, TEstimateEval, weight::ConstWeighted<>> {
 private:
  using BASE = AstarFinder<
      BestFirstFinder<TDrived, TPos, Dummy1, TEstimateEval, Dummy2...>, TPos,
      eval::ConstEval<>, TEstimateEval, weight::ConstWeighted<>>;

 public:
  BestFirstFinder() = default;
  explicit BestFirstFinder(const FinderConfig &_config) : BASE(_config) {}
};

}  // namespace pathfinding

#endif  // INCLUDE_PF_FINDER_BESTFIRSTFINDER_HPP_
