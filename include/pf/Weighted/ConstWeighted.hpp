// Copyright 2024 ldslds449

#ifndef INCLUDE_PF_WEIGHTED_CONSTWEIGHTED_HPP_
#define INCLUDE_PF_WEIGHTED_CONSTWEIGHTED_HPP_

#include <pf/Type.hpp>
#include <pf/Weighted/WeightedBase.hpp>

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
  }

  ConstWeighted() = delete;
};

}  // namespace weight

}  // namespace pathfinding

#endif  // INCLUDE_PF_WEIGHTED_CONSTWEIGHTED_HPP_
