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
  virtual std::pair<PathResult, std::shared_ptr<Path<TPos>>> findPathImpl(
      const TPos &from, const goal::GoalBase<TPos> &goal, const U64 &timeLimit,
      const U64 &nodeLimit) const override {
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
      CostT cost, upCost;
    };

    const TPos &to = goal.getGoalPosition();

    // time limit
    auto startTime = std::chrono::steady_clock::now();

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
    pq.emplace(from, 0, TEstimateEval::eval(from, to));
    // gCost and hCost are useless
    infoTable[from] = {from, 0, 0};

    // for loop to find a path to goal
    Node now, last;
    bool found = false, foundSuitable = false;
    bool timeUp = false, nodeSearchExceed = false;
    U64 nodeCount = 0;
    while (!pq.empty()) {
      Node pre = now;
      now = pq.top();
      pq.pop();
      if (goal.isGoal(now.pos)) {
        if (now.pos == to) {
          found = true;
          last = now;
          break;
        } else if (foundSuitable) {
          auto origYdiff = std::abs(last.pos.y - to.y);
          auto newYdiff = std::abs(now.pos.y - to.y);
          if ((origYdiff > newYdiff) ||
              (origYdiff == newYdiff &&
               TEstimateEval::eval(now.pos) < TEstimateEval::eval(last.pos))) {
            last = now;
          }
        } else {
          foundSuitable = true;
          last = now;
        }
      }
      // check if time is up
      if (BASE::isTimeUp(startTime, timeLimit)) {
        timeUp = true;
        break;
      }
      // check if this node has visited before
      auto &posInfo = infoTable[now.pos];
      if (now.gCost > posInfo.gCost) continue;
      // record node count
      nodeCount++;
      if (nodeLimit > 0 && nodeCount >= nodeLimit) {
        nodeSearchExceed = true;
        break;
      }

      // check jump
      bool canJump =
          BASE::getBlockType(now.pos + TPos{0, 3, 0}).is(BlockType::AIR);

      // find neighbour
      for (const Direction &dir : directions) {
        TPos newOffset = BASE::isAbleToWalkTo(now.pos, dir.offset,
                                              config.fallingDamageTolerance);
        if (newOffset.abs().sum() > 0) {
          CostT addGCost = dir.cost;
          if (newOffset.y > 0)
            addGCost = dir.upCost;
          else if (newOffset.y < 0)
            addGCost = fallCost * (-newOffset.y);

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
      return {PathResult::FOUND, path};
    } else if (timeUp) {
      return {PathResult::TIME_LIMIT_EXCEED, path};
    } else if (nodeSearchExceed) {
      return {PathResult::NODE_SEARCH_LIMIT_EXCEED, path};
    } else {
      return {PathResult::NOT_FOUND, path};
    }
  }

  struct finderConfig {
    bool moveDiagonally = true;
    float fallingDamageTolerance = 0.0;
  };

  AstarFinder() = default;
  AstarFinder(const finderConfig &_config) : config(_config) {}

 private:
  finderConfig config;
};

}  // namespace pathfinding

#endif  // PATHFINDING_FINDER_ASTARFINDER_H_