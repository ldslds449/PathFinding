// Copyright 2024 ldslds449

#ifndef INCLUDE_PF_FINDER_IDBESTFIRSTFINDER_HPP_
#define INCLUDE_PF_FINDER_IDBESTFIRSTFINDER_HPP_

#include <pf/Finder/IDAstarFinder.hpp>

namespace pathfinding {

template <class TDrived, class TPos = Position,
          class Dummy1 = eval::ConstEval<>,
          class TEstimateEval = eval::Manhattan, class... Dummy2>
class IDBestFirstFinder
    : public IDAstarFinder<
          IDBestFirstFinder<TDrived, TPos, Dummy1, TEstimateEval, Dummy2...>,
          TPos, eval::ConstEval<>, TEstimateEval, weight::ConstWeighted<>> {
 private:
  using BASE = IDAstarFinder<
      IDBestFirstFinder<TDrived, TPos, Dummy1, TEstimateEval, Dummy2...>, TPos,
      eval::ConstEval<>, TEstimateEval, weight::ConstWeighted<>>;

 public:
  IDBestFirstFinder() = default;
  explicit IDBestFirstFinder(const FinderConfig &_config) : BASE(_config) {}
};

}  // namespace pathfinding

#endif  // INCLUDE_PF_FINDER_IDBESTFIRSTFINDER_HPP_
