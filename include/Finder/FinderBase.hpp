#ifndef PATHFINDING_FINDER_FINDERBASE_H_
#define PATHFINDING_FINDER_FINDERBASE_H_

#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <regex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "BlockType.hpp"
#include "Goal/Goal.hpp"
#include "Path.hpp"
#include "Vec3.hpp"

namespace pathfinding {

enum PathResult {
  FOUND,
  NOT_FOUND,
  TIME_LIMIT_EXCEED,
  NODE_SEARCH_LIMIT_EXCEED
};

template <class TDrived, class TPos>
class FinderBase {
 public:
  /*
   * Find a path to the goal
   */
  inline std::pair<PathResult, std::shared_ptr<Path<TPos>>> findPath(
      const TPos &from, const goal::GoalBase<TPos> &goal,
      const U64 &timeLimit = 0, const U64 &nodeLimit = 0,
      const U64 &extraTimeLimit = 100) const {
    return static_cast<const TDrived *>(this)->findPathImpl(
        from, goal, timeLimit, nodeLimit, extraTimeLimit);
  }

  /*
   * Move to the goal along the path
   * Input: a path
   * Return: whether movements are successful
   */
  inline bool go(const std::shared_ptr<Path<TPos>> &path) {
    return goImpl(path);
  }

  inline void findPathAndGo(const TPos &from, const goal::GoalBase<TPos> &goal,
                            const U64 &timeLimit = 0, const U64 &nodeLimit = 0,
                            const U64 &extraTimeLimit = 100,
                            const int &retry = 5) {
    auto run =
        [&](const TPos &nowFrom,
            const goal::GoalBase<TPos> &nowGoal) -> std::pair<bool, TPos> {
      auto t1 = std::chrono::steady_clock::now();
      auto r = findPath(nowFrom, nowGoal, timeLimit, nodeLimit);
      auto t2 = std::chrono::steady_clock::now();
      auto path = r.second;

      std::cout << "Length: " << path->size() << std::endl;
      std::cout << "Took: "
                << std::chrono::duration_cast<std::chrono::milliseconds>(t2 -
                                                                         t1)
                       .count()
                << "ms" << std::endl;

      if (path->size() == 0) {
        if (r.first == PathResult::NOT_FOUND) {
          std::cout << "Path Not Found" << std::endl << std::flush;
        } else if (r.first == PathResult::TIME_LIMIT_EXCEED) {
          std::cout << "Time Limit Exceed" << std::endl << std::flush;
        } else if (r.first == PathResult::NODE_SEARCH_LIMIT_EXCEED) {
          std::cout << "Node Search Limit Exceed" << std::endl << std::flush;
        }
        return {false, TPos()};
      }
      std::cout << "Executing...\n";
      go(path);
      std::cout << "Done\n";
      return {true, (*path)[path->size() - 1]};
    };

    TPos lastPos = from, nowGoalPos;
    const TPos &goalPos = goal.getGoalPosition();
    const int step = 2 * 16;  // 2 chunks

    for (int i = 0; i < retry; ++i) {
      // the goal is in a unload chunk
      if (getBlockType(goalPos).is(BlockType::UNKNOWN)) {
        const TPos vec = goalPos - lastPos;
        const auto vecUnit =
            static_cast<const Vec3<double>>(vec) / std::sqrt(vec.squaredNorm());

        nowGoalPos.x = lastPos.x + std::floor(vecUnit.x * step);
        nowGoalPos.y = lastPos.y;
        nowGoalPos.z = lastPos.z + std::floor(vecUnit.z * step);

        std::cout << "Position " << goalPos
                  << " is in a unload chunk, try to get closer " << nowGoalPos
                  << " to load the chunk." << std::endl
                  << std::flush;

        auto result = run(lastPos, goal::RangeGoal<TPos>(nowGoalPos, 5, -1, 5));
        if (!result.first) break;
        lastPos = result.second;
      } else {
        run(lastPos, goal);
        break;
      }
    }
  }

  /*
   * Ask the player to move
   */
  inline bool playerMove(const TPos &vec) {
    return static_cast<TDrived *>(this)->playerMoveImpl(vec);
  }

  /*
   * Get the block name of the block at a specific position
   */
  inline std::string getBlockName(const TPos &pos) const {
    return static_cast<const TDrived *>(this)->getBlockNameImpl(pos);
  }

  inline std::vector<std::string> getBlockName(
      const std::vector<TPos> &pos) const {
    return static_cast<const TDrived *>(this)->getBlockNameImpl(pos);
  }

  /*
   * Get the block type of the block at a specific position
   */

  inline BlockType getBlockType(const TPos &pos) const {
    return static_cast<const TDrived *>(this)->getBlockTypeImpl(pos);
  }

  inline std::vector<BlockType> getBlockType(
      const std::vector<TPos> &pos) const {
    return static_cast<const TDrived *>(this)->getBlockTypeImpl(pos);
  }

  inline BlockType getBlockType(const std::string &blockName) const {
    return static_cast<const TDrived *>(this)->getBlockTypeImpl(blockName);
  }

  /*
   * calculate the fall damage
   */
  inline float getFallDamage(const TPos &landingPos,
                             const typename TPos::value_type &height) const {
    return static_cast<const TDrived *>(this)->getFallDamageImpl(landingPos,
                                                                 height);
  };

  /*
   * This should be implemented in subclass
   */
  virtual std::pair<PathResult, std::shared_ptr<Path<TPos>>> findPathImpl(
      const TPos &from, const goal::GoalBase<TPos> &goal, const U64 &timeLimit,
      const U64 &nodeLimit, const U64 &extraTimeLimit) const = 0;

  bool goImpl(const std::shared_ptr<Path<TPos>> &path, const int &retry = 3) {
    auto &pathVec = path->get();
    // skip first position
    for (int i = 1; i < pathVec.size(); ++i) {
      const TPos &prevPos = pathVec[i - 1], &newPos = pathVec[i],
                 diffPos = newPos - prevPos;
      std::cout << "From: " << prevPos << " To: " << newPos
                << " Diff: " << diffPos << " (" << i << "/" << path->size()
                << ")" << std::endl
                << std::flush;
      bool r;
      int retryTime = 0;
      do {
        r = playerMove(diffPos);
        retryTime++;
      } while (!r && retryTime <= retry);
      if (!r) {
        std::cout << "Move Failed !" << std::endl << std::flush;
        return false;
      }
    }
    return true;
  }

  /*
   * This should be implemented in subclass
   */
  virtual bool playerMoveImpl(const TPos &vec) = 0;

  /*
   * This should be implemented in subclass
   */
  virtual std::string getBlockNameImpl(const TPos &pos) const = 0;

  virtual std::vector<std::string> getBlockNameImpl(
      const std::vector<TPos> &pos) const {
    std::vector<std::string> names;
    for (const TPos &p : pos) {
      names.push_back(getBlockNameImpl(p));
    }
    return names;
  }

  /*
   * Override this if needed
   */
  virtual BlockType getBlockTypeImpl(const TPos &pos) const {
    const std::string blockName = getBlockName(pos);
    return getBlockTypeImpl(blockName);
  }

  virtual std::vector<BlockType> getBlockTypeImpl(
      const std::vector<TPos> &pos) const {
    const std::vector<std::string> &blockNames = getBlockName(pos);
    std::vector<BlockType> blockTypes;
    for (const std::string &name : blockNames) {
      blockTypes.push_back(getBlockTypeImpl(name));
    }
    return blockTypes;
  }

  virtual BlockType getBlockTypeImpl(const std::string &blockName) const {
    const BlockType defaultBlockType = {BlockType::SAFE, BlockType::NONE};
    const std::unordered_map<std::string, BlockType> blockTable = {
        {"", {BlockType::UNKNOWN, BlockType::NONE}},
        {"minecraft:air", {BlockType::AIR, BlockType::FORCE_DOWN}},
        {"minecraft:lava", {BlockType::DANGER, BlockType::FORCE_DOWN}},
        {"minecraft:water", {BlockType::DANGER, BlockType::FORCE_DOWN}},
        {"minecraft:cactus", {BlockType::DANGER, BlockType::NONE}},
        {"minecraft:campfire", {BlockType::DANGER, BlockType::NONE}},
        {"minecraft:magma_block", {BlockType::DANGER, BlockType::NONE}},
        {"minecraft:wither_rose", {BlockType::DANGER, BlockType::NONE}}};
    auto it = blockTable.find(blockName);
    return it != blockTable.end() ? it->second : defaultBlockType;
  }

  /*
   * Override this if needed
   */
  virtual float getFallDamageImpl(
      const TPos &landingPos, const typename TPos::value_type &height) const {
    if (height < 3.375) return 0.0;
    float damage = std::floor(height - 3.375) + 1;

    std::string landingBlock = getBlockName(landingPos);
    if (landingBlock == "minecraft:hay_block" ||
        landingBlock == "minecraft:honey_block") {
      damage *= 0.2;
    } else if (std::regex_match(landingBlock,
                                std::regex("minecraft:\\w+_bed"))) {
      damage *= 0.5;
    } else if (landingBlock == "minecraft:slime_block" ||
               landingBlock == "minecraft:powder_snow" ||
               landingBlock == "minecraft:water" ||
               landingBlock == "minecraft:cobweb" ||
               landingBlock == "minecraft:web") {
      damage = 0;
    } else if (landingBlock == "minecraft:pointed_dripstone") {
      damage = height * 2 - 2;
    }
    return damage;
  };

 protected:
  inline bool isTimeUp(const std::chrono::steady_clock::time_point &start,
                       const U64 &timeLimit) const {
    if (timeLimit == 0) return false;
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now - start)
               .count() >= timeLimit;
  }

  TPos isAbleToWalkTo(const TPos &from, const TPos &XZoffset,
                      const float &fallDamageTol) const {
    bool canJump = getBlockType(from + TPos{0, 3, 0}).is(BlockType::AIR);
    const bool isDiagonal = XZoffset.getXZ().abs().sum() > 1;

    // check positions
    const std::vector blocksPos = {
        from + XZoffset,
        from + XZoffset + TPos{0, 1, 0},
        from + XZoffset + TPos{0, 2, 0},
        from + XZoffset + TPos{0, 3, 0},
        from + TPos{0, 1, XZoffset.z},
        from + TPos{0, 2, XZoffset.z},
        from + TPos{0, 3, XZoffset.z},
        from + TPos{XZoffset.x, 1, 0},
        from + TPos{XZoffset.x, 2, 0},
        from + TPos{XZoffset.x, 3, 0},
    };
    // alias
    enum COORD : short {
      FLOOR = 0,
      FLOOR_UP1,
      FLOOR_UP2,
      FLOOR_UP3,
      Z_UP1,
      Z_UP2,
      Z_UP3,
      X_UP1,
      X_UP2,
      X_UP3,
    };
    const std::vector<BlockType> &blockTypes = getBlockType(blocksPos);

    // unknown, always can not walk to
    if (blockTypes[COORD::FLOOR].is(BlockType::UNKNOWN)) {
      return TPos{0, 0, 0};
    }

    // walk
    if (blockTypes[COORD::FLOOR_UP1].is(BlockType::AIR) &&
        blockTypes[COORD::FLOOR_UP2].is(BlockType::AIR) &&
        blockTypes[COORD::FLOOR].is(BlockType::SAFE)) {
      if (!isDiagonal || (blockTypes[COORD::X_UP1].is(BlockType::AIR) &&
                          blockTypes[COORD::X_UP2].is(BlockType::AIR) &&
                          blockTypes[COORD::Z_UP1].is(BlockType::AIR) &&
                          blockTypes[COORD::Z_UP2].is(BlockType::AIR))) {
        return XZoffset;
      }
    }

    // walk + jump
    if (blockTypes[COORD::FLOOR_UP2].is(BlockType::AIR) &&
        blockTypes[COORD::FLOOR_UP3].is(BlockType::AIR) &&
        blockTypes[COORD::FLOOR_UP1].is(BlockType::SAFE) && canJump) {
      if (!isDiagonal || (blockTypes[COORD::X_UP2].is(BlockType::AIR) &&
                          blockTypes[COORD::X_UP3].is(BlockType::AIR) &&
                          blockTypes[COORD::Z_UP2].is(BlockType::AIR) &&
                          blockTypes[COORD::Z_UP3].is(BlockType::AIR))) {
        return XZoffset + TPos{0, 1, 0};
      }
    }

    // fall
    if ((blockTypes[COORD::FLOOR].is(BlockType::AIR) ||
         blockTypes[COORD::FLOOR].is(BlockType::FORCE_DOWN)) &&
        blockTypes[COORD::FLOOR_UP1].is(BlockType::AIR) &&
        blockTypes[COORD::FLOOR_UP2].is(BlockType::AIR) &&
        (!isDiagonal || blockTypes[COORD::Z_UP1].is(BlockType::AIR) &&
                            blockTypes[COORD::Z_UP2].is(BlockType::AIR) &&
                            blockTypes[COORD::X_UP1].is(BlockType::AIR) &&
                            blockTypes[COORD::X_UP2].is(BlockType::AIR))) {
      TPos landingPos = blocksPos[COORD::FLOOR];
      BlockType landingType;
      // falling
      do {
        landingPos -= TPos{0, 1, 0};
        landingType = getBlockType(landingPos);
      } while (landingType.is(BlockType::AIR) ||
               landingType.is(BlockType::FORCE_DOWN));
      if (!landingType.is(BlockType::DANGER) &&
          getFallDamage(landingPos, (blocksPos[COORD::FLOOR] - landingPos).y) <=
              fallDamageTol) {
        return landingPos - from;
      }
    }

    return TPos(0, 0, 0);
  }

  inline bool isGoalExist(const goal::GoalBase<TPos> &goal) const {
    const TPos &p = goal.getGoalPosition();
    return getBlockType(p).is(BlockType::SAFE) &&
           getBlockType(p.offset(0, 1, 0)).is(BlockType::AIR) &&
           getBlockType(p.offset(0, 2, 0)).is(BlockType::AIR);
  }

 private:
  FinderBase() = default;
  friend TDrived;
};

}  // namespace pathfinding

#endif  // PATHFINDING_FINDER_FINDERBASE_H_