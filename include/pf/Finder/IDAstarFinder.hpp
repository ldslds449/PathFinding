// Copyright 2024 ldslds449

#ifndef INCLUDE_PF_FINDER_IDASTARFINDER_HPP_
#define INCLUDE_PF_FINDER_IDASTARFINDER_HPP_

#include <algorithm>
#include <chrono>
#include <limits>
#include <memory>
#include <stack>
#include <tuple>
#include <unordered_set>
#include <vector>

#include <pf/Evaluate/Evaluate.hpp>
#include <pf/Finder/FinderBase.hpp>
#include <pf/Type.hpp>
#include <pf/Vec3.hpp>
#include <pf/Weighted/Weighted.hpp>

namespace pathfinding {

template <class TDrived, class TPos = Position,
          class TEdgeEval = eval::Manhattan,
          class TEstimateEval = eval::Manhattan,
          class TWeighted = weight::ConstWeighted<>>
class IDAstarFinder
    : public FinderBase<
          IDAstarFinder<TDrived, TPos, TEdgeEval, TEstimateEval, TWeighted>,
          TPos> {
 private:
  using BASE = FinderBase<
      IDAstarFinder<TDrived, TPos, TEdgeEval, TEstimateEval, TWeighted>, TPos>;

 public:
  std::tuple<PathResult, std::shared_ptr<Path<TPos>>, U64> findPathImpl(
      const TPos &from, const goal::GoalBase<TPos> &goal, const U64 &timeLimit,
      const U64 &nodeLimit) override {
    CostT costLimit = TEstimateEval::eval(from, goal.getGoalPosition());
    U64 nowTimeLimit = timeLimit, nowNodeLimit = nodeLimit;
    while (true) {
      auto start = std::chrono::steady_clock::now();
      auto r = LimitedAstar(from, goal, nowTimeLimit, nowNodeLimit, costLimit);
      auto end = std::chrono::steady_clock::now();
      if (std::get<0>(r) == PathResult::FOUND) {
        return {PathResult::FOUND, std::get<1>(r), std::get<3>(r)};  // found
      } else if (std::get<0>(r) == PathResult::TIME_LIMIT_EXCEED) {
        return {PathResult::TIME_LIMIT_EXCEED, std::get<1>(r), std::get<3>(r)};
      } else if (std::get<0>(r) == PathResult::NODE_SEARCH_LIMIT_EXCEED) {
        return {PathResult::NODE_SEARCH_LIMIT_EXCEED, std::get<1>(r),
                std::get<3>(r)};
      } else if (std::get<0>(r) == PathResult::NOT_FOUND) {
        if (std::get<2>(r) == costLimit) {
          return {PathResult::NOT_FOUND, std::get<1>(r),
                  std::get<3>(r)};  // not found
        } else {                    // research
          costLimit = std::get<2>(r);
          std::cout << "Path not found with limit " << costLimit << "\n";
          nowTimeLimit -=
              std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
                  .count();
          nowNodeLimit -= std::get<3>(r);
        }
      } else {
        std::cerr << "Error Search Result\n";
        exit(EXIT_FAILURE);
      }
    }
  }

  IDAstarFinder() = default;
  explicit IDAstarFinder(const FinderConfig &_config) : BASE(_config) {}

 protected:
  std::tuple<PathResult, std::shared_ptr<Path<TPos>>, CostT, U64> LimitedAstar(
      const TPos &from, const goal::GoalBase<TPos> &goal, const U64 &timeLimit,
      const U64 &nodeLimit, const CostT &costLimit) {
    struct Node {
      TPos pos;
      CostT gcost;
      int16_t dirIdx = -1;
      Node(const TPos &p, const CostT &g, const int16_t &d)
          : pos(p), gcost(g), dirIdx(d) {}
      Node() = default;
    };

    const TPos &to = goal.getGoalPosition();

    // stack
    std::stack<Node> st;

    // time limit
    auto startTime = std::chrono::steady_clock::now();

    // stack table
    std::unordered_set<TPos> stackList;

    // directions for selecting neighbours
    std::vector<typename BASE::Direction> directions =
        BASE::template getDirections<TEdgeEval>();
    const CostT fallCost = TEdgeEval::eval(TPos{0, 1, 0});
    const CostT climbCost = TEdgeEval::eval(TPos{0, 1, 0});

    // add initial state
    st.emplace(from, 0, 0);
    stackList.insert(from);

    // for loop to find a path to goal
    bool found = false;
    bool timeUp = false, nodeSearchExceed = false;
    CostT minExceedCost = std::numeric_limits<CostT>::max();  // maximum
    U64 nodeCount = 0;
    while (!st.empty()) {
      auto now = st.top();

      // check whether we reach the goal
      if (goal.isSuitableGoal(now.pos)) {
        found = true;
        break;
      }

      if (BASE::isTimeUp(startTime, timeLimit)) {
        timeUp = true;
        break;
      }

      // whether we visit all children
      if (static_cast<std::size_t>(now.dirIdx) >= directions.size()) {
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

      // get next neighbour
      const typename BASE::Direction &dir = directions[now.dirIdx];

      std::vector<TPos> newOffsets = BASE::isAbleToWalkTo(now.pos, dir.offset);
      for (TPos &newOffset : newOffsets) {
        // action cost
        CostT addGCost = dir.cost;
        if (newOffset.y > 0)
          addGCost += climbCost * std::abs(newOffset.y);
        else if (newOffset.y < 0)
          addGCost += fallCost * std::abs(newOffset.y);

        // node extra cost
        const TPos newPos = now.pos + newOffset;
        addGCost += BASE::getBlockExtraCost(newPos);

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

    // back tracking to get the whole path
    std::shared_ptr<Path<TPos>> path = std::make_shared<Path<TPos>>();
    if (found) {
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

#endif  // INCLUDE_PF_FINDER_IDASTARFINDER_HPP_
