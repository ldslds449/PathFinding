// Copyright 2024 ldslds449

#ifndef INCLUDE_PF_FINDER_UCSFINDER_HPP_
#define INCLUDE_PF_FINDER_UCSFINDER_HPP_

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
  explicit UCSFinder(const FinderConfig &_config) : BASE(_config) {}
};

}  // namespace pathfinding

#endif  // INCLUDE_PF_FINDER_UCSFINDER_HPP_
