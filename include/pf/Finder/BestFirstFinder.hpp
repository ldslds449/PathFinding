#ifndef PATHFINDING_FINDER_BESTFIRSTFINDER_H_
#define PATHFINDING_FINDER_BESTFIRSTFINDER_H_

#include <pf/Finder/AstarFinder.hpp>

namespace pathfinding {

template <class TDrived, class TPos = Position,
          class Dummy1 = eval::ConstEval<>,
          class TEstimateEval = eval::Manhattan, class... Dummy2>
class BestFirstFinder
    : public AstarFinder<
          BestFirstFinder<TDrived, TPos, Dummy1, TEstimateEval, Dummy2...>, TPos,
          eval::ConstEval<>, TEstimateEval, weight::ConstWeighted<>> {
 private:
  using BASE =
      AstarFinder<BestFirstFinder<TDrived, TPos, Dummy1, TEstimateEval, Dummy2...>,
                  TPos, eval::ConstEval<>, TEstimateEval,
                  weight::ConstWeighted<>>;

 public:
  BestFirstFinder() = default;
  BestFirstFinder(const FinderConfig &_config) : BASE(_config) {}
};

}  // namespace pathfinding

#endif  // PATHFINDING_FINDER_BESTFIRSTFINDER_H_