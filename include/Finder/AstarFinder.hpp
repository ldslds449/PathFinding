#ifndef PATHFINDING_FINDER_ASTARFINDER_H_
#define PATHFINDING_FINDER_ASTARFINDER_H_

#include <chrono>
#include <queue>

#include "Evaluate/Evaluate.hpp"
#include "Finder/FinderBase.hpp"
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
    // a node in A*
    struct Node {
      TPos pos;
      CostT gCost, hCost;
      Node(const TPos &p, const CostT &g, const CostT &h)
          : pos(p), gCost(g), hCost(h) {}
      Node() = default;
    };

    // record the information of a node
    struct PosInfo {
      TPos parent;
      CostT gCost, hCost;  // the lowest gCost, hCost of key
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
    auto pq_cmp = [](const Node &a, const Node &b) {
      // from lowest cost to largest cost
      return TWeighted::combine(a.gCost, a.hCost) >
             TWeighted::combine(b.gCost, b.hCost);
    };
    std::priority_queue<Node, std::vector<Node>, decltype(pq_cmp)> pq(pq_cmp);

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
    pq.emplace(from, 0, TEstimateEval::eval(from, to));
    // gCost and hCost are useless
    infoTable[from] = {from, 0, 0};

    // for loop to find a path to goal
    Node last;
    bool found = false, foundSuitable = false;
    bool timeUp = false, nodeSearchExceed = false;
    U64 nodeCount = 0;
    while (!pq.empty()) {
      Node now = pq.top();
      pq.pop();

      // check if this node has visited before
      auto &posInfo = infoTable[now.pos];
      if (now.gCost > posInfo.gCost) continue;

      // check if this node is the goal
      if (goal.isSuitableGoal(now.pos)) {
        if (now.pos == to) {
          found = true;
          last = now;
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
            now.pos, dir.offset, config.fallingDamageTolerance);
        for (TPos &newOffset : newOffsets) {
          CostT addGCost = dir.cost;
          if (newOffset.y > 0)
            addGCost += climbCost * (newOffset.y);
          else if (newOffset.y < 0)
            addGCost += fallCost * (-newOffset.y);

          // add new position
          const TPos newPos = now.pos + newOffset;
          const TPos &parent = now.pos;
          const CostT newGCost = now.gCost + addGCost;

          auto found_it = infoTable.find(newPos);
          if (found_it != infoTable.end()) {
            CostT &PreGCost = found_it->second.gCost;
            // compare the gCost and reserve the one with lower cost
            if (newGCost < PreGCost) {
              found_it->second.parent = parent;
              found_it->second.gCost = newGCost;
              pq.emplace(newPos, newGCost,
                          found_it->second.hCost);  // lazy deletion
            }
          } else {
            CostT newHCost = TEstimateEval::eval(newPos, to);
            pq.emplace(newPos, newGCost, newHCost);
            infoTable[newPos] = {parent, newGCost, newHCost};
          }
        }
      }
    }

    // back tracking to get the whole path
    std::shared_ptr<Path<TPos>> path = std::make_shared<Path<TPos>>();
    if (found || foundSuitable) {
      TPos nowPos = last.pos;
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