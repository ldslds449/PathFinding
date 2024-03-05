#ifndef PATHFINDING_FINDER_ILSFINDER_H_
#define PATHFINDING_FINDER_ILSFINDER_H_

#include <pf/Finder/IDAstarFinder.hpp>

namespace pathfinding {

template <class TDrived, class TPos = Position,
          class TEdgeEval = eval::Manhattan, class... Dummy>
class ILSFinder
    : public IDAstarFinder<ILSFinder<TDrived, TPos, TEdgeEval, Dummy...>,
                           TPos, TEdgeEval, eval::ConstEval<>,
                           weight::ConstWeighted<>> {
 private:
  using BASE =
      IDAstarFinder<ILSFinder<TDrived, TPos, TEdgeEval, Dummy...>, TPos,
                    TEdgeEval, eval::ConstEval<>, weight::ConstWeighted<>>;

 public:
  ILSFinder() = default;
  ILSFinder(const FinderConfig &_config) : BASE(_config) {}
};

}  // namespace pathfinding

#endif  // PATHFINDING_FINDER_ILSFINDER_H_