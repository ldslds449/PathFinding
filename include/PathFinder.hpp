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
#include "Vec3.hpp"
#include "World/WorldBase.hpp"

namespace pathfinding {

class PathFinder {
 public:
  template <class TEval, class TWorld, class TPos>
  std::shared_ptr<Path<TPos>> findPath(std::shared_ptr<TWorld> world,
                                       const TPos &from, const TPos &to) {
    using NODE = Node<TPos>;

    // compare function for priority queue, sort NODE from lowest cost to
    // largest cost
    auto pq_cmp = [](const NODE &a, const NODE &b) {
      return (a.gCost + a.hCost) > (b.gCost + b.hCost);
    };
    // key: Position
    // value: {Parent, gCost of key, hCost of key}
    std::priority_queue<NODE, std::deque<NODE>, decltype(pq_cmp)> pq(pq_cmp);

    // hash function for unordered map
    auto map_hash = [](const TPos &a) { return a.hash(); };
    std::unordered_map<TPos, NODE, decltype(map_hash)> infoTable(23, map_hash);

    // directions for selecting neighbours
    std::vector<TPos> directions;
    for (int x = -1; x <= 1; ++x) {
      for (int y = -1; y <= 1; ++y) {
        for (int z = -1; z <= 1; ++z) {
          directions.push_back(TPos{x, y, z});
        }
      }
    }

    // add initial state
    pq.push({from, 0, TEval::eval(from, to)});
    infoTable[from] = {from, 0, 0};  // gCost and hCost are unless

    // for loop to find a path to goal
    while (!pq.empty()) {
      auto now = pq.top();
      pq.pop();
      if (now.pos == to) break;

      // add neighbour
      for (TPos &v : directions) {
        TPos newPos = now.pos + v;
        auto found_it = infoTable.find(newPos);
        if (found_it != infoTable.end()) {  // the neighbour was added before
          int &PreGCost = found_it->second.gCost;
          int newGCost = now.gCost + 1;
          // compare the gCost and reserve the one with lower cost
          if (PreGCost > newGCost) {
            found_it->second.gCost = newGCost;
            found_it->second.pos = now.pos;
          }
        } else {
          BlockType btype = world->getBlockType(newPos);

          // check the block type
          if (btype == BlockType::DANGER)
            continue;
          else if (btype == BlockType::SAFE) {
            int gCost = now.gCost + 1;
            int hCost = TEval::eval(newPos, to);
            NODE newNode{newPos, gCost, hCost};
            pq.push(newNode);
            infoTable[newPos] = {now.pos, gCost, hCost};
          }
        }
      }
    }

    // back tracking to get the whole path
    std::shared_ptr<Path<TPos>> path = std::make_shared<Path<TPos>>();
    TPos nowPos = to;
    while (true) {
      path->add(nowPos);
      TPos &newPos = infoTable[nowPos].pos;
      if (newPos == nowPos) break;
      nowPos = newPos;
    }
    path->reverse();

    return path;
  }

 private:
  template <class TPos>
  struct Node {
    TPos pos;
    int gCost;
    int hCost;
  };
};

}  // namespace pathfinding

#endif  // PATHFINDING_PATHFINDING_H_