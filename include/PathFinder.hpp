#ifndef PATHFINDING_PATHFINDING_H_
#define PATHFINDING_PATHFINDING_H_

#include <deque>
#include <functional>
#include <memory>
#include <queue>
#include <unordered_map>
#include <utility>
#include <vector>

#include "Path.hpp"
#include "Type.hpp"
#include "Vec3.hpp"
#include "World/WorldBase.hpp"

namespace pathfinding {

template <class TWorld, class TPos>
class PathFinder {
 public:
  template <class TEval>
  std::shared_ptr<Path<TPos>> findPath(const TPos &from, const TPos &to) {
    struct Node {
      TPos pos;
      U64 gCost, hCost;
    };

    struct PosInfo {
      TPos parent;
      U64 gCost, hCost;  // the lowest gCost, hCost of key
      BlockType type;
    };

    struct Direction {
      TPos offset;
      U64 cost;
    };

    // compare function for priority queue, sort Node from lowest cost to
    // largest cost
    auto pq_cmp = [](const Node &a, const Node &b) {
      return (a.gCost + a.hCost) > (b.gCost + b.hCost);
    };
    std::priority_queue<Node, std::deque<Node>, decltype(pq_cmp)> pq(pq_cmp);

    // hash function for unordered map
    auto map_hash = [](const TPos &a) { return a.hash(); };
    // key: Position
    // value: {Parent, gCost of key, hCost of key}
    std::unordered_map<TPos, PosInfo, decltype(map_hash)> infoTable(
        23, map_hash);  // 23: initial bucket count

    // directions for selecting neighbours
    std::vector<Direction> directions;
    for (int x = -1; x <= 1; ++x) {
      for (int y = -1; y <= 1; ++y) {
        for (int z = -1; z <= 1; ++z) {
          if (x == 0 && y == 0 && z == 0) continue;
          if (x == 0 && y == -1 && z == 0) continue;  // falling
          TPos offset{x, y, z};
          if (!config.moveDiagonally && offset.abs().sum() > 1) continue;
          directions.push_back({offset, TEval::eval(TPos{0, 0, 0}, offset)});
        }
      }
    }
    const Direction fallDirection = {
        TPos{0, -1, 0}, TEval::eval(TPos{0, 0, 0}, TPos{0, -1, 0})};

    // add initial state
    pq.push({from, 0, TEval::eval(from, to)});
    infoTable[from] = {from, 0, 0,
                       BlockType::SAFE};  // gCost and hCost are useless

    // for loop to find a path to goal
    Node now, last;
    bool reachChunkEdge = false;
    while (!pq.empty() && !reachChunkEdge) {
      Node pre = now;
      now = pq.top();
      pq.pop();
      if (now.pos == to) {
        last = now;
        break;
      }
      auto &posInfo = infoTable[now.pos];
      // check if the block reachs the edge of the chunk
      if (posInfo.type == BlockType::UNKNOWN) {
        last = pre;
        break;
      }
      // check if this node has visited before
      if (now.gCost > posInfo.gCost) continue;

      // add neighbour
      for (const Direction &dir : directions) {
        TPos newPos = now.pos + dir.offset;
        U64 newGCost = now.gCost + dir.cost;
        auto found_it = infoTable.find(newPos);

        // the neighbour was added before
        if (found_it != infoTable.end()) {
          U64 &PreGCost = found_it->second.gCost;
          // compare the gCost and reserve the one with lower cost
          if (newGCost < PreGCost) {
            found_it->second.gCost = newGCost;
            found_it->second.parent = now.pos;
            pq.push({newPos, newGCost, now.hCost});  // lazy deletion
          }
        } else {  // a new neighbour
          BlockType btype = world->getBlockType(newPos);

          // check the block type
          if (btype == BlockType::DANGER) {
            continue;
          } else if (btype == BlockType::SAFE ||
                     btype == BlockType::UNKNOWN) {  // add to queue
            U64 newHCost = TEval::eval(newPos, to);
            pq.push({newPos, newGCost, newHCost});
            infoTable[newPos] = {now.pos, newGCost, newHCost, btype};
          } else if (btype == BlockType::AIR) {  // find the landing position
            TPos top = newPos;
            do {
              // falling
              newPos += fallDirection.offset;
              newGCost += fallDirection.cost;
              btype = world->getBlockType(newPos);
            } while (btype == BlockType::AIR);
            if (btype != BlockType::DANGER &&
                world->calFallDamage(newPos, (top - newPos).y) <=
                    config.fallingDamageTolerance) {
              U64 newHCost = TEval::eval(newPos, to);
              pq.push({newPos, newGCost, newHCost});
              infoTable[newPos] = {now.pos, newGCost, newHCost, btype};
            }
          }
        }
      }
    }

    // back tracking to get the whole path
    std::shared_ptr<Path<TPos>> path = std::make_shared<Path<TPos>>();
    TPos nowPos = now.pos;
    while (true) {
      path->add(nowPos);
      TPos &newPos = infoTable[nowPos].parent;
      if (newPos == nowPos) break;
      nowPos = newPos;
    }
    path->reverse();

    return path;
  }

  struct pathFinderConfig {
    bool moveDiagonally = true;
    float fallingDamageTolerance = 0.0;
  };

  PathFinder(std::shared_ptr<TWorld> _world, pathFinderConfig &_config)
      : world(_world), config(_config) {}
  PathFinder(std::shared_ptr<TWorld> _world) : world(_world) {}

 private:
  std::shared_ptr<TWorld> world;
  pathFinderConfig config;
};

}  // namespace pathfinding

#endif  // PATHFINDING_PATHFINDING_H_