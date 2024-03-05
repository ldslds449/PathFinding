// Copyright 2024 ldslds449

#ifndef INCLUDE_PF_FINDER_ILSFINDER_HPP_
#define INCLUDE_PF_FINDER_ILSFINDER_HPP_

#include <pf/Finder/IDAstarFinder.hpp>

namespace pathfinding {

template <class TDrived, class TPos = Position,
          class TEdgeEval = eval::Manhattan, class... Dummy>
class ILSFinder
    : public IDAstarFinder<ILSFinder<TDrived, TPos, TEdgeEval, Dummy...>, TPos,
                           TEdgeEval, eval::ConstEval<>,
                           weight::ConstWeighted<>> {
 private:
  using BASE =
      IDAstarFinder<ILSFinder<TDrived, TPos, TEdgeEval, Dummy...>, TPos,
                    TEdgeEval, eval::ConstEval<>, weight::ConstWeighted<>>;

 public:
  ILSFinder() = default;
  explicit ILSFinder(const FinderConfig &_config) : BASE(_config) {}
};

}  // namespace pathfinding

#endif  // INCLUDE_PF_FINDER_ILSFINDER_HPP_
