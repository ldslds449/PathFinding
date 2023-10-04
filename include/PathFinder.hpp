#ifndef PATHFINDING_PATHFINDING_H_
#define PATHFINDING_PATHFINDING_H_

#include <chrono>
#include <deque>
#include <functional>
#include <memory>
#include <queue>
#include <unordered_map>
#include <utility>
#include <vector>

#include "Client/ClientBase.hpp"
#include "Path.hpp"
#include "Type.hpp"
#include "Vec3.hpp"

namespace pathfinding {

template <class TClient>
class PathFinder {
 public:
  template <class TEval, class TPos = typename TClient::pos_type>
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
      U64 cost, upCost, downCost;
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
      for (int z = -1; z <= 1; ++z) {
        if (x == 0 && z == 0) continue;
        TPos offset{x, 0, z};
        if (!config.moveDiagonally && offset.abs().getXZ().sum() > 1) continue;
        directions.push_back(
            {offset,
             static_cast<U64>(offset.squaredNorm()),  // TEval::eval(offset)
             TEval::eval(offset + TPos{0, 1, 0}),
             TEval::eval(offset - TPos{0, 1, 0})});
      }
    }

    // add initial state
    pq.push({from, 0, TEval::eval(from, to)});
    // gCost and hCost are useless
    infoTable[from] = {from, 0, 0, {BlockType::SAFE, BlockType::NONE}};

    // for loop to find a path to goal
    Node now, last;
    bool found = false;
    while (!pq.empty()) {
      Node pre = now;
      now = pq.top();
      pq.pop();
      if (now.pos == to) {
        last = now;
        found = true;
        break;
      }
      auto &posInfo = infoTable[now.pos];
      // check if the block reachs the edge of the chunk
      if (posInfo.type.is(BlockType::UNKNOWN)) {
        found = true;
        last = pre;
        break;
      }
      // check if this node has visited before
      if (now.gCost > posInfo.gCost) continue;

      // add neighbour
      auto addNewPos = [&](const TPos &newPos, const TPos &parent,
                           const U64 &gCost, const BlockType &btype) {
        auto found_it = infoTable.find(newPos);
        if (found_it != infoTable.end()) {
          U64 &PreGCost = found_it->second.gCost;
          // compare the gCost and reserve the one with lower cost
          if (gCost < PreGCost) {
            found_it->second.parent = parent;
            found_it->second.gCost = gCost;
            pq.push({newPos, gCost, found_it->second.hCost});  // lazy deletion
          }
        } else {
          U64 hCost = TEval::eval(newPos, to);
          pq.push({newPos, gCost, hCost});
          infoTable[newPos] = {parent, gCost, hCost, btype};
        }
      };

      // check jump
      bool canJump =
          client->getBlockType(now.pos + TPos{0, 3, 0}).is(BlockType::AIR);

      // find neighbour
      for (const Direction &dir : directions) {
        const bool isDiagonal = dir.offset.getXZ().abs().sum() > 1;
        TPos floorPos = now.pos + dir.offset;
        BlockType floorType = client->getBlockType(floorPos);

        // up
        TPos up1Pos = floorPos + TPos{0, 1, 0}, up2Pos = up1Pos + TPos{0, 1, 0},
             up3Pos = up2Pos + TPos{0, 1, 0};
        TPos upZ1Pos = now.pos + TPos{0, 1, dir.offset.z},
             upX1Pos = now.pos + TPos{dir.offset.x, 1, 0},
             upZ2Pos = upZ1Pos + TPos{0, 1, 0},
             upX2Pos = upX1Pos + TPos{0, 1, 0},
             upZ3Pos = upZ2Pos + TPos{0, 1, 0},
             upX3Pos = upX2Pos + TPos{0, 1, 0};
        BlockType up1Type = client->getBlockType(up1Pos),
                  up2Type = client->getBlockType(up2Pos),
                  up3Type = client->getBlockType(up3Pos);
        BlockType upZ1Type = client->getBlockType(upZ1Pos),
                  upX1Type = client->getBlockType(upX1Pos),
                  upZ2Type = client->getBlockType(upZ2Pos),
                  upX2Type = client->getBlockType(upX2Pos),
                  upZ3Type = client->getBlockType(upZ3Pos),
                  upX3Type = client->getBlockType(upX3Pos);
        if (up1Type.is(BlockType::AIR) && up2Type.is(BlockType::AIR) &&
            floorType.is(BlockType::SAFE)) {
          if (!isDiagonal ||
              (upZ1Type.is(BlockType::AIR) && upX1Type.is(BlockType::AIR) &&
               upZ2Type.is(BlockType::AIR) && upX2Type.is(BlockType::AIR))) {
            addNewPos(floorPos, now.pos, now.gCost + dir.cost, floorType);
          }
        }
        if (up2Type.is(BlockType::AIR) && up3Type.is(BlockType::AIR) &&
            up1Type.is(BlockType::SAFE) && canJump) {
          if (!isDiagonal ||
              (upZ2Type.is(BlockType::AIR) && upX2Type.is(BlockType::AIR) &&
               upZ3Type.is(BlockType::AIR) && upX3Type.is(BlockType::AIR))) {
            addNewPos(up1Pos, now.pos, now.gCost + dir.upCost, up1Type);
          }
        }

        // down
        if (floorType.is(BlockType::AIR) &&
            ((!isDiagonal && up1Type.is(BlockType::AIR) &&
              up2Type.is(BlockType::AIR)) ||
             (isDiagonal && upZ1Type.is(BlockType::AIR) &&
              upX1Type.is(BlockType::AIR) && upZ2Type.is(BlockType::AIR) &&
              upX2Type.is(BlockType::AIR)))) {
          TPos landingPos = floorPos;
          BlockType landingType = client->getBlockType(landingPos);
          U64 cost = 0;
          while (landingType.is(BlockType::AIR) ||
                 landingType.is(BlockType::FORCE_DOWN)) {
            // falling
            landingPos -= TPos{0, 1, 0};
            landingType = client->getBlockType(landingPos);
            cost += TEval::eval(TPos{0, 1, 0});
          }
          if (!landingType.is(BlockType::DANGER) &&
              client->calFallDamage(landingPos, (floorPos - landingPos).y) <=
                  config.fallingDamageTolerance) {
            addNewPos(landingPos, now.pos, now.gCost + cost, landingType);
          }
        }
      }
    }

    // back tracking to get the whole path
    std::shared_ptr<Path<TPos>> path = std::make_shared<Path<TPos>>();
    if (found) {
      TPos nowPos = last.pos;
      while (true) {
        path->add(nowPos);
        TPos &newPos = infoTable[nowPos].parent;
        if (newPos == nowPos) break;
        nowPos = newPos;
      }
      path->reverse();
    }

    return path;
  }

  struct pathFinderConfig {
    bool moveDiagonally = true;
    float fallingDamageTolerance = 0.0;
  };

  PathFinder(std::shared_ptr<TClient> _client, const pathFinderConfig &_config)
      : client(_client), config(_config) {}
  PathFinder(std::shared_ptr<TClient> _client) : client(_client) {}

 private:
  std::shared_ptr<TClient> client;
  pathFinderConfig config;
};

}  // namespace pathfinding

#endif  // PATHFINDING_PATHFINDING_H_