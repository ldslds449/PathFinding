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
#include "Weighted/Weighted.hpp"

namespace pathfinding {

template <class TDrived, class TWeighted = weight::ConstWeighted<>,
          class TEstimateEval = eval::Manhattan,
          class TEdgeEval = eval::Euclidean, class TPos = Position>
class IDAstarFinder
    : public FinderBase<
          IDAstarFinder<TDrived, TWeighted, TEstimateEval, TEdgeEval, TPos>,
          TPos> {
 private:
  using BASE = FinderBase<
      IDAstarFinder<TDrived, TWeighted, TEstimateEval, TEdgeEval, TPos>, TPos>;

 public:
  virtual std::tuple<PathResult, std::shared_ptr<Path<TPos>>, U64> findPathImpl(
      const TPos &from, const goal::GoalBase<TPos> &goal, const U64 &timeLimit,
      const U64 &nodeLimit, const U64 &extraTimeLimit) const override {
    CostT costLimit = TEstimateEval::eval(from, goal.getGoalPosition());
    U64 nowTimeLimit = timeLimit, nowNodeLimit = nodeLimit;
    while (true) {
      auto start = std::chrono::steady_clock::now();
      auto r = AstarWithCostLimit(from, goal, nowTimeLimit, nowNodeLimit,
                                  extraTimeLimit, costLimit);
      auto end = std::chrono::steady_clock::now();
      if (std::get<0>(r) == PathResult::FOUND)
        return {PathResult::FOUND, std::get<1>(r), std::get<3>(r)};  // found
      else if (std::get<0>(r) == PathResult::NOT_FOUND &&
               std::get<2>(r) == costLimit)
        return {PathResult::NOT_FOUND, std::get<1>(r), std::get<3>(r)};  // not found
      else if (std::get<0>(r) == PathResult::TIME_LIMIT_EXCEED)
        return {PathResult::TIME_LIMIT_EXCEED, std::get<1>(r), std::get<3>(r)};
      else if (std::get<0>(r) == PathResult::NODE_SEARCH_LIMIT_EXCEED)
        return {PathResult::NODE_SEARCH_LIMIT_EXCEED, std::get<1>(r), std::get<3>(r)};
      costLimit = std::get<2>(r);
      std::cout << "Path not found, set cost limit to " << costLimit << "\n";
      nowTimeLimit -=
          std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
              .count();
      nowNodeLimit -= std::get<3>(r);
    }
  }

  struct finderConfig {
    bool moveDiagonally = true;
    float fallingDamageTolerance = 0.0;
  };

  IDAstarFinder() = default;
  IDAstarFinder(const finderConfig &_config) : config(_config) {}

 protected:
  finderConfig config;

  std::tuple<PathResult, std::shared_ptr<Path<TPos>>, CostT, U64>
  AstarWithCostLimit(const TPos &from, const goal::GoalBase<TPos> &goal,
                     const U64 &timeLimit, const U64 &nodeLimit,
                     const U64 &extraTimeLimit, const CostT &costLimit) const {
    struct Node {
      TPos pos;
      CostT gcost;
      short dirIdx;
      Node(const TPos &p, const CostT &g, const short &d)
          : pos(p), gcost(g), dirIdx(d) {}
      Node() = default;
    };

    // direction to generate next node
    struct Direction {
      TPos offset;
      CostT cost;
    };

    const TPos &to = goal.getGoalPosition();
    const bool goalExist = BASE::isGoalExist(goal);

    // stack
    std::stack<Node> st;

    // time limit
    auto startTime = std::chrono::steady_clock::now();
    decltype(startTime) extraStartTime;

    // stack table
    std::unordered_set<TPos> stackList;

    // directions for selecting neighbours
    std::vector<Direction> directions;
    for (int x = -1; x <= 1; ++x) {
      for (int z = -1; z <= 1; ++z) {
        TPos offset{x, 0, z};
        if (!config.moveDiagonally && offset.abs().getXZ().sum() > 1) continue;
        directions.push_back({offset, TEdgeEval::eval(offset)});
      }
    }
    const CostT fallCost = TEdgeEval::eval(TPos{0, 1, 0});
    const CostT climbCost = TEdgeEval::eval(TPos{0, 1, 0});

    // add initial state
    st.emplace(from, 0, 0);
    stackList.insert(from);

    // for loop to find a path to goal
    TPos last;
    bool found = false, foundSuitable = false;
    bool timeUp = false, nodeSearchExceed = false;
    CostT minExceedCost = std::numeric_limits<CostT>::max();  // maximum
    U64 nodeCount = 0;
    while (!st.empty()) {
      auto now = st.top();

      // check whether we reach the goal
      if (goal.isSuitableGoal(now.pos)) {
        if (now.pos == to) {
          last = now.pos;
          found = true;
          break;
        } else if (foundSuitable) {
          if (TEstimateEval::eval(now.pos) < TEstimateEval::eval(last.pos)) {
            last = now;
          }
        } else if (!goalExist) {
          foundSuitable = true;
          last = now;
          break;
        } else {
          foundSuitable = true;
          last = now;
          extraStartTime = std::chrono::steady_clock::now();
        }
      }

      if (BASE::isTimeUp(startTime, timeLimit)) {
        timeUp = true;
        break;
      }
      if (foundSuitable && BASE::isTimeUp(extraStartTime, extraTimeLimit)) {
        break;
      }

      // whether we visit all children
      if (now.dirIdx >= directions.size()) {
        st.pop();
        stackList.erase(now.pos);
        continue;
      } else {
        st.top().dirIdx++;
      }

      // record node count
      nodeCount++;
      if (nodeLimit > 0 && nodeCount >= nodeLimit) {
        nodeSearchExceed = true;
        break;
      }

      // check jump
      bool canJump =
          BASE::getBlockType(now.pos + TPos{0, 3, 0}).is(BlockType::AIR);

      // get next neighbour
      const Direction &dir = directions[now.dirIdx];

      TPos newOffsets = BASE::isAbleToWalkTo(now.pos, dir.offset,
                                             config.fallingDamageTolerance);
      for (TPos &newOffset : newOffsets) {
        if (newOffset.abs().sum() > 0) {
          const TPos newPos = now.pos + newOffset;
          CostT addGCost = dir.cost;
          if (newOffset.y > 0)
            addGCost += climbCost * (newOffset.y);
          else if (newOffset.y < 0)
            addGCost += fallCost * (-newOffset.y);

          CostT newGCost = now.gcost + addGCost;
          CostT newFCost =
              TWeighted::combine(newGCost, TEstimateEval::eval(newPos, to));

          // whether the child is in the pv
          if (stackList.count(newPos) == 0) {
            if (newFCost <= costLimit) {
              st.emplace(newPos, newGCost, 0);
              stackList.insert(newPos);
            } else {
              minExceedCost = std::min(minExceedCost, newFCost);
            }
          }
        }
      }
    }

    // back tracking to get the whole path
    std::shared_ptr<Path<TPos>> path = std::make_shared<Path<TPos>>();
    if (found || foundSuitable) {
      while (!st.empty()) {
        const TPos &nowPos = st.top().pos;
        path->add(nowPos);
        st.pop();
      }
      path->reverse();
      return {PathResult::FOUND, path, 0, nodeCount};
    } else if (timeUp) {
      return {PathResult::TIME_LIMIT_EXCEED, path, 0, nodeCount};
    } else if (nodeSearchExceed) {
      return {PathResult::NODE_SEARCH_LIMIT_EXCEED, path, 0, nodeCount};
    } else {
      return {
          PathResult::NOT_FOUND, path,
          (minExceedCost == std::numeric_limits<CostT>::max() ? costLimit
                                                              : minExceedCost),
          nodeCount};
    }
  }
};

}  // namespace pathfinding

#endif  // PATHFINDING_FINDER_IDASTARFINDER_H_