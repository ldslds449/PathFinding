#ifndef PATHFINDING_FINDER_BIASTARFINDER_H_
#define PATHFINDING_FINDER_BIASTARFINDER_H_

#include <chrono>

#include "Evaluate/Evaluate.hpp"
#include "Finder/FinderBase.hpp"
#include "Heap/TableHeap.hpp"
#include "Type.hpp"
#include "Vec3.hpp"
#include "Weighted/Weighted.hpp"

namespace pathfinding {

template <class TDrived, class TPos = Position,
          class TEdgeEval = eval::Manhattan,
          class TEstimateEval = eval::Manhattan,
          class TWeighted = weight::ConstWeighted<>>
class BiAstarFinder
    : public FinderBase<
          BiAstarFinder<TDrived, TPos, TEdgeEval, TEstimateEval, TWeighted>,
          TPos> {
 private:
  using BASE = FinderBase<
      BiAstarFinder<TDrived, TPos, TEdgeEval, TEstimateEval, TWeighted>, TPos>;

 public:
  virtual std::tuple<PathResult, std::shared_ptr<Path<TPos>>, U64> findPathImpl(
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

      std::function<std::vector<TPos>(const TPos &, const TPos &,
                                      const float &)>
          walkable;

      CostT eval(const TPos &pos) const {
        return TEstimateEval::eval(pos, target);
      }

      DataSet(const TPos &t, decltype(walkable) f)
          : target(t), heap(std::less<CostT>{}), walkable(f) {}
    };

    // create two datasets
    const TPos &to = goal.getGoalPosition();
    DataSet forward(to, [&](const TPos &a, const TPos &b, const float &c) {
      return BASE::isAbleToWalkTo(a, b, c);
    });
    DataSet backward(from, [&](const TPos &a, const TPos &b, const float &c) {
      return BASE::isAbleToWalkFrom(a, b, c);
    });

    // time limit
    auto startTime = std::chrono::steady_clock::now();

    // directions for selecting neighbours
    std::vector<Direction<CostT>> directions = getDirections<CostT, TEdgeEval>(config.moveDiagonally);
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
      for (const Direction<CostT> &dir : directions) {
        std::vector<TPos> newOffsets =
            select.walkable(now, dir.offset, config.fallingDamageTolerance);

        for (TPos &newOffset : newOffsets) {
          CostT addGCost = dir.cost;
          if (newOffset.y > 0)
            addGCost += climbCost * (newOffset.y);
          else if (newOffset.y < 0)
            addGCost += fallCost * (-newOffset.y);

          // add new position
          const TPos newPos = now + newOffset;
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
  BiAstarFinder(const FinderConfig &_config) : BASE(_config) {}
};

}  // namespace pathfinding

#endif  // PATHFINDING_FINDER_BIASTARFINDER_H_