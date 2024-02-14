#ifndef PATHFINDING_FINDER_IDUCSFINDER_H_
#define PATHFINDING_FINDER_IDUCSFINDER_H_

#include "IDAstarFinder.hpp"

namespace pathfinding {

template <class TDrived, class TPos = Position,
          class TEdgeEval = eval::Manhattan, class... Dummy>
class IDUCSFinder
    : public IDAstarFinder<IDUCSFinder<TDrived, TPos, TEdgeEval, Dummy...>,
                           TPos, TEdgeEval, eval::ConstEval<>,
                           weight::ConstWeighted<>> {
 private:
  using BASE =
      IDAstarFinder<IDUCSFinder<TDrived, TPos, TEdgeEval, Dummy...>, TPos,
                    TEdgeEval, eval::ConstEval<>, weight::ConstWeighted<>>;

 public:
  IDUCSFinder() = default;
  IDUCSFinder(const FinderConfig &_config) : BASE(_config) {}
};

}  // namespace pathfinding

#endif  // PATHFINDING_FINDER_IDUCSFINDER_H_