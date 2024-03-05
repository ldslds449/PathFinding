#ifndef PATHFINDING_FINDER_UCSFINDER_H_
#define PATHFINDING_FINDER_UCSFINDER_H_

#include <pf/Finder/AstarFinder.hpp>

namespace pathfinding {

template <class TDrived, class TPos = Position,
          class TEdgeEval = eval::Manhattan, class... Dummy>
class UCSFinder
    : public AstarFinder<UCSFinder<TDrived, TPos, TEdgeEval, Dummy...>, TPos,
                         TEdgeEval, eval::ConstEval<>,
                         weight::ConstWeighted<>> {
 private:
  using BASE =
      AstarFinder<UCSFinder<TDrived, TPos, TEdgeEval, Dummy...>, TPos,
                  TEdgeEval, eval::ConstEval<>, weight::ConstWeighted<>>;

 public:
  UCSFinder() = default;
  UCSFinder(const FinderConfig &_config) : BASE(_config) {}
};

}  // namespace pathfinding

#endif  // PATHFINDING_FINDER_UCSFINDER_H_