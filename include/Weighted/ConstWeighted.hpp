#ifndef PATHFINDING_WEIGHTED_CONSTWEIGHTED_H_
#define PATHFINDING_WEIGHTED_CONSTWEIGHTED_H_

#include "Type.hpp"
#include "WeightedBase.hpp"

namespace pathfinding {

namespace weight {

template <int GWeight = 1, int HWeight = 1>
class ConstWeighted final
    : public WeightedBase<ConstWeighted<GWeight, HWeight>> {
 public:
  inline static CostT combineImpl(const CostT &g, const CostT &h) {
    if constexpr (GWeight == 1 && HWeight == 1) {
      return g + h;
    } else if constexpr (GWeight == 1) {
      return g + h * static_cast<CostT>(HWeight);
    } else if constexpr (HWeight == 1) {
      return g * static_cast<CostT>(GWeight) + h;
    }
  };

  ConstWeighted() = delete;
};

}  // namespace weight

}  // namespace pathfinding

#endif  // PATHFINDING_WEIGHTED_CONSTWEIGHTED_H_