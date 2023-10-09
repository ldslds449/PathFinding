#ifndef PATHFINDING_FINDER_IDASTARFINDER_H_
#define PATHFINDING_FINDER_IDASTARFINDER_H_

#include <chrono>
#include <limits>
#include <stack>
#include <unordered_set>

#include "Evaluate/Evaluate.hpp"
#include "Finder/FinderBase.hpp"
#include "Type.hpp"
#include "Vec3.hpp"

namespace pathfinding {

template <class TDrived, class TEstimateEval = eval::Manhattan,
          class TEdgeEval = eval::Euclidean, class TPos = Position>
class IDAstarFinder : public FinderBase<IDAstarFinder<TDrived>, TPos> {
 private:
  using BASE = FinderBase<IDAstarFinder<TDrived>, TPos>;

 public:
  virtual std::pair<PathResult, std::shared_ptr<Path<TPos>>> findPathImpl(
      const TPos &from, const goal::GoalBase<TPos> &goal,
      const U64 &timeLimit) const override {
    CostT costLimit = TEstimateEval::eval(from, goal.getGoalPosition());
    U64 nowTimeLimit = timeLimit;
    while (true) {
      auto start = std::chrono::steady_clock::now();
      auto r = AstarWithCostLimit(from, goal, nowTimeLimit, costLimit);
      auto end = std::chrono::steady_clock::now();
      if (std::get<0>(r) == PathResult::FOUND)
        return {PathResult::FOUND, std::get<1>(r)};  // found
      if (std::get<0>(r) == PathResult::NOT_FOUND &&
          std::get<2>(r) == costLimit)
        return {PathResult::NOT_FOUND, std::get<1>(r)};  // not found
      if (std::get<0>(r) == PathResult::TIME_LIMIT_EXCEED)
        return {PathResult::TIME_LIMIT_EXCEED, std::get<1>(r)};
      costLimit = std::get<2>(r);
      std::cout << "Path not found, set cost limit to " << costLimit << "\n";
      nowTimeLimit -=
          std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
              .count();
    }
  }

  struct finderConfig {
    bool moveDiagonally = true;
    float fallingDamageTolerance = 0.0;
  };

  IDAstarFinder() = default;
  IDAstarFinder(const finderConfig &_config) : config(_config) {}

 private:
  finderConfig config;

 protected:
  std::tuple<PathResult, std::shared_ptr<Path<TPos>>, CostT> AstarWithCostLimit(
      const TPos &from, const goal::GoalBase<TPos> &goal, const U64 &timeLimit,
      const CostT &costLimit) const {
    struct Node {
      TPos pos;
      short dirIdx;
      CostT gCost;
    };

    // direction to generate next node
    struct Direction {
      TPos offset;
      CostT cost, upCost;
    };

    const TPos &to = goal.getGoalPosition();

    // stack
    std::stack<Node> st;

    // time limit
    auto startTime = std::chrono::steady_clock::now();

    // visited table
    auto set_hash = [](const TPos &a) { return a.hash(); };
    std::unordered_set<TPos, decltype(set_hash)> visited(23, set_hash);

    // directions for selecting neighbours
    std::vector<Direction> directions;
    for (int x = -1; x <= 1; ++x) {
      for (int z = -1; z <= 1; ++z) {
        if (x == 0 && z == 0) continue;  // no move
        TPos offset{x, 0, z};
        if (!config.moveDiagonally && offset.abs().getXZ().sum() > 1) continue;
        directions.push_back({offset,
                              TEdgeEval::eval(offset),  // squared euclidean
                              TEdgeEval::eval(offset + TPos{0, 1, 0})});
      }
    }
    const CostT fallCost = TEdgeEval::eval(TPos{0, 1, 0});

    // add initial state
    st.push({from, 0, 0});
    visited.insert(from);

    // for loop to find a path to goal
    TPos last;
    bool found = false, timeUp = false;
    CostT minExceedCost = std::numeric_limits<CostT>::max();  // maximum
    while (!st.empty()) {
      auto now = st.top();
      st.pop();

      // check whether we reach the goal
      if (goal.isGoal(now.pos)) {
        last = now.pos;
        found = true;
        break;
      }

      if (BASE::isTimeUp(startTime, timeLimit)) {
        timeUp = true;
        break;
      }

      // whether we visit all children
      if (now.dirIdx >= directions.size()) {
        continue;
      } else {
        st.push({now.pos, static_cast<short>(now.dirIdx + 1), now.gCost});
      }

      // check jump
      bool canJump =
          BASE::getBlockType(now.pos + TPos{0, 3, 0}).is(BlockType::AIR);

      // get next neighbour
      const Direction &dir = directions[now.dirIdx];

      TPos newOffset = BASE::isAbleToWalkTo(now.pos, dir.offset,
                                            config.fallingDamageTolerance);
      if (newOffset.abs().sum() > 0) {
        const TPos newPos = now.pos + newOffset;
        // whether we visited it before
        if (visited.count(newPos) == 0) {
          CostT addGCost = dir.cost;
          if (newOffset.y > 0)
            addGCost = dir.upCost;
          else if (newOffset.y < 0)
            addGCost = fallCost * (-newOffset.y);

          CostT newGCost = now.gCost + addGCost;
          CostT newFCost = newGCost + TEstimateEval::eval(newPos, to);
          if (newFCost <= costLimit) {
            st.push({newPos, 0, newGCost});
            visited.insert(newPos);
          } else {
            minExceedCost = std::min(minExceedCost, newFCost);
          }
        }
      }
    }

    // back tracking to get the whole path
    std::shared_ptr<Path<TPos>> path = std::make_shared<Path<TPos>>();
    if (found) {
      path->add(last);
      while (!st.empty()) {
        const TPos &nowPos = st.top().pos;
        path->add(nowPos);
        st.pop();
      }
      path->reverse();
      return {PathResult::FOUND, path, 0};
    } else if (timeUp) {
      return {PathResult::TIME_LIMIT_EXCEED, path, 0};
    } else {
      return {
          PathResult::NOT_FOUND, path,
          (minExceedCost == std::numeric_limits<CostT>::max() ? costLimit
                                                              : minExceedCost)};
    }
  }
};

}  // namespace pathfinding

#endif  // PATHFINDING_FINDER_IDASTARFINDER_H_