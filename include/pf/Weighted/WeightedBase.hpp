// Copyright 2024 ldslds449

#ifndef INCLUDE_PF_WEIGHTED_WEIGHTEDBASE_HPP_
#define INCLUDE_PF_WEIGHTED_WEIGHTEDBASE_HPP_

#include <pf/Type.hpp>

namespace pathfinding {

namespace weight {

template <class TDrived>
class WeightedBase {
 public:
  inline static CostT combine(const CostT &g, const CostT &h) {
    return TDrived::combineImpl(g, h);
  }

 private:
  WeightedBase() {}
  friend TDrived;
};

}  // namespace weight

}  // namespace pathfinding

#endif  // INCLUDE_PF_WEIGHTED_WEIGHTEDBASE_HPP_
