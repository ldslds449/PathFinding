#ifndef PATHFINDING_FINDER_IDBESTFIRSTFINDER_H_
#define PATHFINDING_FINDER_IDBESTFIRSTFINDER_H_

#include "IDAstarFinder.hpp"

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
  IDBestFirstFinder(const FinderConfig &_config) : BASE(_config) {}
};

}  // namespace pathfinding

#endif  // PATHFINDING_FINDER_IDBESTFIRSTFINDER_H_