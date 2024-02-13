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
  virtual std::tuple<PathResult, std::shared_ptr<Path<TPos>>, U64> findPathImpl(
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

  struct finderConfig {
    bool moveDiagonally = true;
    float fallingDamageTolerance = 0.0;
  };

  IDAstarFinder() = default;
  IDAstarFinder(const finderConfig &_config) : config(_config) {}

 protected:
  finderConfig config;

  std::tuple<PathResult, std::shared_ptr<Path<TPos>>, CostT, U64> LimitedAstar(
      const TPos &from, const goal::GoalBase<TPos> &goal, const U64 &timeLimit,
      const U64 &nodeLimit, const CostT &costLimit) {
    struct Node {
      TPos pos;
      CostT gcost;
      short dirIdx;
      Node(const TPos &p, const CostT &g, const short &d)
          : pos(p), gcost(g), dirIdx(d) {}
      Node() = default;
    };

    const TPos &to = goal.getGoalPosition();
    const bool goalExist = BASE::isGoalExist(goal);

    // stack
    std::stack<Node> st;

    // time limit
    auto startTime = std::chrono::steady_clock::now();

    // stack table
    std::unordered_set<TPos> stackList;

    // directions for selecting neighbours
    std::vector<Direction<CostT>> directions = getDirections<CostT, TEdgeEval>(config.moveDiagonally);
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
      const Direction<CostT> &dir = directions[now.dirIdx];

      std::vector<TPos> newOffsets = BASE::isAbleToWalkTo(
          now.pos, dir.offset, config.fallingDamageTolerance);
      for (TPos &newOffset : newOffsets) {
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

#endif  // PATHFINDING_FINDER_IDASTARFINDER_H_