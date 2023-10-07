#ifndef PATHFINDING_FINDER_FINDERBASE_H_
#define PATHFINDING_FINDER_FINDERBASE_H_

#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <regex>
#include <string>

#include "BlockType.hpp"
#include "Goal/Goal.hpp"
#include "Path.hpp"
#include "Vec3.hpp"

namespace pathfinding {

template <class TDrived, class TPos>
class FinderBase {
 public:
  /*
   * Find a path to the goal
   */
  inline std::shared_ptr<Path<TPos>> findPath(const TPos &from,
                                              const goal::GoalBase<TPos> &goal,
                                              const U64 &timeLimit = 0) const {
    return static_cast<const TDrived *>(this)->findPathImpl(from, goal,
                                                            timeLimit);
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
                            const U64 &timeLimit = 0, const int &retry = 10) {
    TPos lastPos = from, nowGoalPos;
    const int step = 2 * 16;  // 2 chunks

    for (int i = 0; i < retry; ++i) {
      std::shared_ptr<const goal::GoalBase<TPos>> nowGoal;

      // the goal is in a unload chunk
      if (getBlockType(goal.getGoalPosition()).is(BlockType::UNKNOWN)) {
        const TPos vec = goal.getGoalPosition() - lastPos;
        const auto vecUnit =
            static_cast<const Vec3<double>>(vec) / std::sqrt(vec.squaredNorm());

        nowGoalPos.x = lastPos.x + std::floor(vecUnit.x * step);
        nowGoalPos.y = lastPos.y;
        nowGoalPos.z = lastPos.z + std::floor(vecUnit.z * step);

        std::cout << "Position " << goal.getGoalPosition()
                  << " is in a unload chunk, try to get closer " << nowGoalPos
                  << " to load the chunk." << std::endl;

        nowGoal =
            std::make_shared<const goal::RangeGoal<TPos>>(nowGoalPos, 5, -1, 5);
      } else {
        nowGoal = std::shared_ptr<const goal::GoalBase<TPos>>(&goal);
      }

      auto t1 = std::chrono::steady_clock::now();
      auto path = findPath(lastPos, *nowGoal, timeLimit);
      auto t2 = std::chrono::steady_clock::now();

      std::cout << (*path) << "Length: " << path->size() << std::endl;
      std::cout << "Took: "
                << std::chrono::duration_cast<std::chrono::milliseconds>(t2 -
                                                                         t1)
                       .count()
                << "ms" << std::endl;

      if (path->size() == 0) break;
      std::cout << "Executing...\n";
      go(path);
      std::cout << "Done\n";

      lastPos = (*path)[path->size() - 1];
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

  /*
   * Get the block type of the block at a specific position
   */
  inline BlockType getBlockType(const TPos &pos) const {
    return static_cast<const TDrived *>(this)->getBlockTypeImpl(pos);
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
  virtual std::shared_ptr<Path<TPos>> findPathImpl(
      const TPos &from, const goal::GoalBase<TPos> &goal,
      const U64 &timeLimit) const = 0;

  bool goImpl(const std::shared_ptr<Path<TPos>> &path) {
    auto &pathVec = path->get();
    // skip first position
    for (int i = 1; i < pathVec.size(); ++i) {
      const TPos &prevPos = pathVec[i - 1], &newPos = pathVec[i],
                 diffPos = newPos - prevPos;
      std::cout << "From: " << prevPos << " To: " << newPos
                << " Diff: " << diffPos << std::endl;
      bool r = playerMove(diffPos);
      if (!r) return false;
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

  /*
   * Override this if needed
   */
  virtual BlockType getBlockTypeImpl(const TPos &pos) const {
    const std::string blockName = getBlockName(pos);
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

 private:
  FinderBase() = default;
  friend TDrived;
};

}  // namespace pathfinding

#endif  // PATHFINDING_FINDER_FINDERBASE_H_