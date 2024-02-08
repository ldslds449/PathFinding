#ifndef PATHFINDING_FINDER_ASTARFINDER_H_
#define PATHFINDING_FINDER_ASTARFINDER_H_

#include <chrono>

#include "Evaluate/Evaluate.hpp"
#include "Finder/FinderBase.hpp"
#include "Heap/TableHeap.hpp"
#include "Type.hpp"
#include "Vec3.hpp"
#include "Weighted/Weighted.hpp"

namespace pathfinding {

template <class TDrived, class TWeighted = weight::ConstWeighted<>,
          class TEstimateEval = eval::Manhattan,
          class TEdgeEval = eval::Euclidean, class TPos = Position>
class AstarFinder
    : public FinderBase<
          AstarFinder<TDrived, TWeighted, TEstimateEval, TEdgeEval, TPos>,
          TPos> {
 private:
  using BASE = FinderBase<
      AstarFinder<TDrived, TWeighted, TEstimateEval, TEdgeEval, TPos>, TPos>;

 public:
  virtual std::tuple<PathResult, std::shared_ptr<Path<TPos>>, U64> findPathImpl(
      const TPos &from, const goal::GoalBase<TPos> &goal, const U64 &timeLimit,
      const U64 &nodeLimit, const U64 &extraTimeLimit) const override {

    // record the information of a node
    struct PosInfo {
      TPos parent;
      bool closed;  // whether is in close set
    };

    // direction to generate next node
    struct Direction {
      TPos offset;
      CostT cost;
    };

    const TPos &to = goal.getGoalPosition();
    const bool goalExist = BASE::isGoalExist(goal);

    // time limit
    auto startTime = std::chrono::steady_clock::now();
    decltype(startTime) extraStartTime;

    // compare function for priority queue, sort Node
    auto heap_cmp = [](const std::pair<CostT, CostT> &a, const std::pair<CostT, CostT> &b) {
      // from lowest cost to largest cost
      return TWeighted::combine(a.first, a.second) <
             TWeighted::combine(b.first, b.second);
    };
    TableHeap<TPos, std::pair<CostT, CostT>, decltype(heap_cmp)> heap(heap_cmp);

    // key: Position
    // value: {Parent, gCost of key, hCost of key}
    std::unordered_map<TPos, PosInfo> infoTable;

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
    heap.update(from, {0, TEstimateEval::eval(from, to)});
    // gCost and hCost are useless
    infoTable[from] = {from, false};

    // for loop to find a path to goal
    TPos last;
    bool found = false, foundSuitable = false;
    bool timeUp = false, nodeSearchExceed = false;
    U64 nodeCount = 0;
    while (heap.size() > 0) {
      CostT nowGCost = heap.lookup(heap.top()).first;
      TPos now = heap.extract();

      // add to close set
      infoTable[now].closed = true;

      // check if this node is the goal
      if (goal.isSuitableGoal(now)) {
        if (now == to) {
          found = true;
          last = now;
          break;
        } else if (foundSuitable) {
          if (TEstimateEval::eval(now) < TEstimateEval::eval(last)) {
            last = now;
          }
        } else if (!goalExist) {
          foundSuitable = true;
          last = now;
          break;
        } else {
          extraStartTime = std::chrono::steady_clock::now();
          foundSuitable = true;
          last = now;
        }
      }

      // check if time is up
      if (BASE::isTimeUp(startTime, timeLimit)) {
        timeUp = true;
        break;
      }

      if (foundSuitable && BASE::isTimeUp(extraStartTime, extraTimeLimit)) {
        break;
      }

      // record node count
      nodeCount++;
      if (nodeLimit > 0 && nodeCount >= nodeLimit) {
        nodeSearchExceed = true;
        break;
      }

      // find neighbour
      for (const Direction &dir : directions) {
        std::vector<TPos> newOffsets = BASE::isAbleToWalkTo(
            now, dir.offset, config.fallingDamageTolerance);
        for (TPos &newOffset : newOffsets) {
          CostT addGCost = dir.cost;
          if (newOffset.y > 0)
            addGCost += climbCost * (newOffset.y);
          else if (newOffset.y < 0)
            addGCost += fallCost * (-newOffset.y);

          // add new position
          const TPos newPos = now + newOffset;
          const TPos &parent = now;
          const CostT newGCost = nowGCost + addGCost;

          auto found_it = infoTable.find(newPos);
          if (found_it != infoTable.end()) {
            if(!found_it->second.closed){
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
    if (found || foundSuitable) {
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

  struct finderConfig {
    bool moveDiagonally = true;
    float fallingDamageTolerance = 0.0;
  };

  AstarFinder() = default;
  AstarFinder(const finderConfig &_config) : config(_config) {}

 protected:
  finderConfig config;
};

}  // namespace pathfinding

#endif  // PATHFINDING_FINDER_ASTARFINDER_H_