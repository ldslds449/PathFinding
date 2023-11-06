#ifndef PATHFINDING_FINDER_BIASTARFINDER_H_
#define PATHFINDING_FINDER_BIASTARFINDER_H_

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
class BiAstarFinder
    : public FinderBase<
          BiAstarFinder<TDrived, TWeighted, TEstimateEval, TEdgeEval, TPos>,
          TPos> {
 private:
  using BASE = FinderBase<
      BiAstarFinder<TDrived, TWeighted, TEstimateEval, TEdgeEval, TPos>, TPos>;

 public:
  virtual std::pair<PathResult, std::shared_ptr<Path<TPos>>> findPathImpl(
      const TPos &from, const goal::GoalBase<TPos> &goal, const U64 &timeLimit,
      const U64 &nodeLimit, const U64 &extraTimeLimit) const override {
    // a node in A*
    struct Node {
      TPos pos;
      CostT gCost, hCost, fCost;
      Node(const TPos &p, const CostT &g, const CostT &h)
          : pos(p), gCost(g), hCost(h), fCost(TWeighted::combine(g, h)) {}
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

    // store A star data
    struct DataSet {
     private:
      // compare function for priority queue, sort Node
      const std::function<bool(const Node &a, const Node &b)> pq_cmp =
          [](const Node &a, const Node &b) {
            // from lowest cost to largest cost
            return a.fCost > b.fCost;
          };

      const TPos target;

     public:
      std::priority_queue<Node, std::vector<Node>, decltype(pq_cmp)> pq;

      // key: Position
      // value: {Parent, gCost of key, hCost of key}
      std::unordered_map<TPos, PosInfo> infoTable;

      std::function<std::vector<TPos>(const TPos &, const TPos &,
                                      const float &)>
          walkable;

      CostT eval(const TPos &pos) const {
        return TEstimateEval::eval(pos, target);
      }

      DataSet(const TPos &t, decltype(walkable) f)
          : target(t), pq(pq_cmp), walkable(f) {}
    };

    const TPos &to = goal.getGoalPosition();
    DataSet forward(to, [&](const TPos &a, const TPos &b, const float &c) {
      return BASE::isAbleToWalkTo(a, b, c);
    });
    DataSet backward(from, [&](const TPos &a, const TPos &b, const float &c) {
      return BASE::isAbleToWalkFrom(a, b, c);
    });

    // time limit
    auto startTime = std::chrono::steady_clock::now();
    decltype(startTime) extraStartTime;

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
    forward.pq.emplace(from, 0, forward.eval(from));
    backward.pq.emplace(to, 0, backward.eval(to));
    // gCost and hCost are useless
    forward.infoTable[from] = {from, 0, 0};
    backward.infoTable[to] = {to, 0, 0};

    // for loop to find a path to goal
    Node last;
    bool found = false;
    bool timeUp = false, nodeSearchExceed = false;
    U64 nodeCount = 0;
    while (!forward.pq.empty() && !backward.pq.empty()) {
      while (forward.pq.top().gCost >
             forward.infoTable[forward.pq.top().pos].gCost)
        forward.pq.pop();
      while (backward.pq.top().gCost >
             backward.infoTable[backward.pq.top().pos].gCost)
        backward.pq.pop();

      const Node &fnode = forward.pq.top(), &bnode = backward.pq.top();
      DataSet &select = (fnode.fCost <= bnode.fCost ? forward : backward);
      auto &otherTable = (fnode.fCost <= bnode.fCost ? backward.infoTable : forward.infoTable);

      Node now = select.pq.top();
      select.pq.pop();

      // check goal
      if (otherTable.find(now.pos) != otherTable.end()) {
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
      for (const Direction &dir : directions) {
        std::vector<TPos> newOffsets =
            select.walkable(now.pos, dir.offset, config.fallingDamageTolerance);
        
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

          auto found_it = select.infoTable.find(newPos);
          if (found_it != select.infoTable.end()) {
            CostT &PreGCost = found_it->second.gCost;
            // compare the gCost and reserve the one with lower cost
            if (newGCost < PreGCost) {
              found_it->second.parent = parent;
              found_it->second.gCost = newGCost;
              select.pq.emplace(newPos, newGCost,
                                found_it->second.hCost);  // lazy deletion
            }
          } else {
            CostT newHCost = select.eval(newPos);
            select.pq.emplace(newPos, newGCost, newHCost);
            select.infoTable[newPos] = {parent, newGCost, newHCost};
          }
        }
      }
    }

    // back tracking to get the whole path
    std::shared_ptr<Path<TPos>> path = std::make_shared<Path<TPos>>();
    if (found) {
      // forward
      {
        TPos nowPos = last.pos;
        while (true) {
          TPos &newPos = forward.infoTable[nowPos].parent;
          if (newPos == nowPos) break;
          nowPos = newPos;
          path->add(nowPos);
        }
        path->reverse();
      }
      // backward
      {
        TPos nowPos = last.pos;
        while (true) {
          path->add(nowPos);
          TPos &newPos = backward.infoTable[nowPos].parent;
          if (newPos == nowPos) break;
          nowPos = newPos;
        }
      }
      // pop non-goal nodes
      while (!goal.isSuitableGoal(path->back())) {
        path->pop_back();
      }
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

  BiAstarFinder() = default;
  BiAstarFinder(const finderConfig &_config) : config(_config) {}

 protected:
  finderConfig config;
};

}  // namespace pathfinding

#endif  // PATHFINDING_FINDER_BIASTARFINDER_H_