// Copyright 2024 ldslds449

#ifndef INCLUDE_PF_FINDER_ASTARFINDER_HPP_
#define INCLUDE_PF_FINDER_ASTARFINDER_HPP_

#include <chrono>
#include <memory>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include <pf/Evaluate/Evaluate.hpp>
#include <pf/Finder/FinderBase.hpp>
#include <pf/Heap/TableHeap.hpp>
#include <pf/Type.hpp>
#include <pf/Vec3.hpp>
#include <pf/Weighted/Weighted.hpp>

namespace pathfinding {

template <class TDrived, class TPos = Position,
          class TEdgeEval = eval::Manhattan,
          class TEstimateEval = eval::Manhattan,
          class TWeighted = weight::ConstWeighted<>>
class AstarFinder
    : public FinderBase<
          AstarFinder<TDrived, TPos, TEdgeEval, TEstimateEval, TWeighted>,
          TPos> {
 private:
  using BASE = FinderBase<
      AstarFinder<TDrived, TPos, TEdgeEval, TEstimateEval, TWeighted>, TPos>;

 public:
  std::tuple<PathResult, std::shared_ptr<Path<TPos>>, U64> findPathImpl(
      const TPos &from, const goal::GoalBase<TPos> &goal, const U64 &timeLimit,
      const U64 &nodeLimit) override {
    // record the information of a node
    struct PosInfo {
      TPos parent;
      bool closed;  // whether is in close set
    };

    const TPos &to = goal.getGoalPosition();

    // time limit
    auto startTime = std::chrono::steady_clock::now();

    // compare function for priority queue, sort Node
    auto heap_cmp = [](const std::pair<CostT, CostT> &a,
                       const std::pair<CostT, CostT> &b) {
      // from lowest cost to largest cost
      // if there is a tie in f(x), comparing h(x) instead.
      const CostT costA = TWeighted::combine(a.first, a.second);
      const CostT costB = TWeighted::combine(b.first, b.second);
      return (costA != costB ? (costA < costB) : (a.second < b.second));
    };
    TableHeap<TPos, std::pair<CostT, CostT>, decltype(heap_cmp)> heap(heap_cmp);

    // key: Position
    // value: {Parent, gCost of key, hCost of key}
    std::unordered_map<TPos, PosInfo> infoTable;

    // directions for selecting neighbours
    std::vector<typename BASE::Direction> directions =
        BASE::template getDirections<TEdgeEval>();
    const CostT fallCost = TEdgeEval::eval(TPos{0, 1, 0});
    const CostT climbCost = TEdgeEval::eval(TPos{0, 1, 0});

    // add initial state
    heap.update(from, {0, TEstimateEval::eval(from, to)});
    // gCost and hCost are useless
    infoTable[from] = {from, false};

    // for loop to find a path to goal
    TPos last;
    bool found = false;
    bool timeUp = false, nodeSearchExceed = false;
    U64 nodeCount = 0;
    while (heap.size() > 0) {
      CostT nowGCost = heap.lookup(heap.top()).first;
      TPos now = heap.extract();

      // add to close set
      infoTable[now].closed = true;

      // check if this node is the goal
      if (goal.isSuitableGoal(now)) {
        found = true;
        last = now;
        break;
      }

      // check if time is up
      if (BASE::isTimeUp(startTime, timeLimit)) {
        timeUp = true;
        break;
      }

      // record node count
      nodeCount++;
      if (nodeLimit > 0 && nodeCount >= nodeLimit) {
        nodeSearchExceed = true;
        break;
      }

      // find neighbour
      for (const typename BASE::Direction &dir : directions) {
        std::vector<TPos> newOffsets = BASE::isAbleToWalkTo(now, dir.offset);
        for (TPos &newOffset : newOffsets) {
          // action cost
          CostT addGCost = dir.cost;
          if (newOffset.y > 0)
            addGCost += climbCost * std::abs(newOffset.y);
          else if (newOffset.y < 0)
            addGCost += fallCost * std::abs(newOffset.y);

          // node extra cost
          const TPos newPos = now + newOffset;
          addGCost += BASE::getBlockExtraCost(newPos);

          // add new position
          const TPos &parent = now;
          const CostT newGCost = nowGCost + addGCost;

          auto found_it = infoTable.find(newPos);
          if (found_it != infoTable.end()) {
            if (!found_it->second.closed) {
              auto PreCost = heap.lookup(newPos);
              // compare the gCost and reserve the one with lower cost
              if (newGCost < PreCost.first) {
                found_it->second.parent = parent;
                heap.update(newPos, {newGCost, PreCost.second});
              }
            }
          } else {
            CostT newHCost = TEstimateEval::eval(newPos, to);
            heap.update(newPos, {newGCost, newHCost});
            infoTable[newPos] = {parent, false};
          }
        }
      }
    }

    // back tracking to get the whole path
    std::shared_ptr<Path<TPos>> path = std::make_shared<Path<TPos>>();
    if (found) {
      TPos nowPos = last;
      while (true) {
        path->add(nowPos);
        TPos &newPos = infoTable[nowPos].parent;
        if (newPos == nowPos) break;
        nowPos = newPos;
      }
      path->reverse();
      return {PathResult::FOUND, path, nodeCount};
    } else if (timeUp) {
      return {PathResult::TIME_LIMIT_EXCEED, path, nodeCount};
    } else if (nodeSearchExceed) {
      return {PathResult::NODE_SEARCH_LIMIT_EXCEED, path, nodeCount};
    } else {
      return {PathResult::NOT_FOUND, path, nodeCount};
    }
  }

  AstarFinder() = default;
  explicit AstarFinder(const FinderConfig &_config) : BASE(_config) {}
};

}  // namespace pathfinding

#endif  // INCLUDE_PF_FINDER_ASTARFINDER_HPP_
