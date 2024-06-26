// Copyright 2024 ldslds449

#ifndef INCLUDE_PF_FINDER_BIASTARFINDER_HPP_
#define INCLUDE_PF_FINDER_BIASTARFINDER_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <stack>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include <pf/Evaluate/Evaluate.hpp>
#include <pf/Finder/CompleteFinder.hpp>
#include <pf/Heap/TableHeap.hpp>
#include <pf/Type.hpp>
#include <pf/Vec3.hpp>
#include <pf/Weighted/Weighted.hpp>

namespace pathfinding {

template <class TDrived, class TPos = Position,
          class TEdgeEval = eval::Manhattan,
          class TEstimateEval = eval::Manhattan,
          class TWeighted = weight::ConstWeighted<>>
class BiAstarFinder
    : public CompleteFinder<
          BiAstarFinder<TDrived, TPos, TEdgeEval, TEstimateEval, TWeighted>,
          TPos> {
 private:
  using BASE = CompleteFinder<
      BiAstarFinder<TDrived, TPos, TEdgeEval, TEstimateEval, TWeighted>, TPos>;

 public:
  std::tuple<PathResult, std::shared_ptr<Path<TPos>>, U64> findPathImpl(
      const TPos &from, const goal::GoalBase<TPos> &goal, const U64 &timeLimit,
      const U64 &nodeLimit) override {
    // record the information of a node
    struct PosInfo {
      TPos parent;
      CostT gCost, hCost;  // the lowest gCost, hCost of key
      bool closed;         // whether in closed set
    };

    // store A star data
    struct DataSet {
     private:
      const TPos target;

     public:
      TableHeap<TPos, CostT, std::less<CostT>> heap;

      // key: Position
      // value: {Parent, gCost of key, hCost of key, closed}
      std::unordered_map<TPos, PosInfo> infoTable;

      std::function<std::vector<TPos>(const TPos &pos, const TPos &XZoffset)>
          walkable;

      CostT eval(const TPos &pos) const {
        return TEstimateEval::eval(pos, target);
      }

      DataSet(const TPos &t, decltype(walkable) f)
          : target(t), heap(std::less<CostT>{}), walkable(f) {}
    };

    // create two datasets
    const TPos &to = goal.getGoalPosition();
    DataSet forward(to, [&](const TPos &pos, const TPos &XZoffset) {
      return BASE::isAbleToWalkTo(pos, XZoffset);
    });
    DataSet backward(from, [&](const TPos &pos, const TPos &XZoffset) {
      return BASE::isAbleToWalkFrom(pos, XZoffset);
    });

    // time limit
    auto startTime = std::chrono::steady_clock::now();

    // directions for selecting neighbours
    std::vector<typename BASE::Direction> directions =
        BASE::template getDirections<TEdgeEval>();
    const CostT fallCost = TEdgeEval::eval(TPos{0, 1, 0});
    const CostT climbCost = TEdgeEval::eval(TPos{0, 1, 0});

    // add initial state
    forward.heap.update(from, forward.eval(from));
    backward.heap.update(to, backward.eval(to));
    // gCost and hCost are useless
    forward.infoTable[from] = {from, 0, 0, false};
    backward.infoTable[to] = {to, 0, 0, false};

    const bool goalExist = BASE::isGoalExist(goal);

    // for loop to find a path to goal
    TPos last;
    bool found = false;
    bool timeUp = false, nodeSearchExceed = false;
    U64 nodeCount = 0;
    while (forward.heap.size() > 0 && backward.heap.size() > 0) {
      const CostT f_fcost = forward.heap.lookup(forward.heap.top()),
                  b_fcost = backward.heap.lookup(backward.heap.top());
      DataSet &select = (f_fcost <= b_fcost ? forward : backward);
      auto &otherTable =
          (f_fcost <= b_fcost ? backward.infoTable : forward.infoTable);

      TPos now = select.heap.extract();
      // add to close set
      auto &now_info = select.infoTable[now];
      now_info.closed = true;
      CostT nowGcost = now_info.gCost;

      // check goal
      if (otherTable.find(now) != otherTable.end()) {
        // found goal
        found = true;
        last = now;
        break;
      }

      // check if time is up
      if (BASE::isTimeUp(startTime, timeLimit)) {
        timeUp = true;
        break;
      }

      // check search budget
      nodeCount++;
      if (nodeLimit > 0 && nodeCount >= nodeLimit) {
        nodeSearchExceed = true;
        break;
      }

      // find neighbour
      for (const typename BASE::Direction &dir : directions) {
        std::vector<TPos> newOffsets = select.walkable(now, dir.offset);
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
          const CostT newGCost = nowGcost + addGCost;

          auto found_it = select.infoTable.find(newPos);
          if (found_it != select.infoTable.end()) {
            if (!found_it->second.closed) {
              CostT &PreGCost = found_it->second.gCost;
              // compare the gCost and reserve the one with lower cost
              if (newGCost < PreGCost) {
                found_it->second.parent = parent;
                found_it->second.gCost = newGCost;
                select.heap.update(newPos, newGCost + found_it->second.hCost);
              }
            }
          } else {
            CostT newHCost = select.eval(newPos);
            select.heap.update(newPos, newGCost + newHCost);
            select.infoTable[newPos] = {parent, newGCost, newHCost, false};
          }
        }
      }
    }

    // back tracking to get the whole path
    std::shared_ptr<Path<TPos>> path = std::make_shared<Path<TPos>>();
    if (found) {
      std::vector<TPos> origPath;
      // forward
      {
        std::stack<TPos> st;
        TPos nowPos = last;
        while (true) {
          TPos &newPos = forward.infoTable[nowPos].parent;
          if (newPos == nowPos) break;
          nowPos = newPos;
          st.push(nowPos);
        }
        while (!st.empty()) {
          origPath.emplace_back(st.top());
          st.pop();
        }
      }
      // backward
      {
        TPos nowPos = last;
        while (true) {
          origPath.push_back(nowPos);
          TPos &newPos = backward.infoTable[nowPos].parent;
          if (newPos == nowPos) break;
          nowPos = newPos;
        }
      }
      for (auto &pos : origPath) {
        path->add(pos);
        if (!goalExist && goal.isSuitableGoal(pos)) break;
      }
      return {PathResult::FOUND, path, nodeCount};
    } else if (timeUp) {
      return {PathResult::TIME_LIMIT_EXCEED, path, nodeCount};
    } else if (nodeSearchExceed) {
      return {PathResult::NODE_SEARCH_LIMIT_EXCEED, path, nodeCount};
    } else {
      return {PathResult::NOT_FOUND, path, nodeCount};
    }
  }

  BiAstarFinder() = default;
  explicit BiAstarFinder(const FinderConfig &_config) : BASE(_config) {}
};

}  // namespace pathfinding

#endif  // INCLUDE_PF_FINDER_BIASTARFINDER_HPP_
