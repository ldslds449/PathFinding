#ifndef PATHFINDING_WEIGHTED_WEIGHTEDBASE_H_
#define PATHFINDING_WEIGHTED_WEIGHTEDBASE_H_

#include <pf/Type.hpp>

namespace pathfinding {

namespace weight {

template <class TDrived>
class WeightedBase {
 public:
  inline static CostT combine(const CostT &g, const CostT &h) {
    return TDrived::combineImpl(g, h);
  };

 private:
  WeightedBase() {}
  friend TDrived;
};

}  // namespace weight

}  // namespace pathfinding

#endif  // PATHFINDING_WEIGHTED_WEIGHTEDBASE_H_