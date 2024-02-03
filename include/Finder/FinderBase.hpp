#ifndef PATHFINDING_FINDER_FINDERBASE_H_
#define PATHFINDING_FINDER_FINDERBASE_H_

#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <regex>
#include <string>
#include <thread>
#include <tuple>
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
  inline std::tuple<PathResult, std::shared_ptr<Path<TPos>>, U64> findPath(
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
    return static_cast<TDrived *>(this)->goImpl(path);
  }

  /*
   * get player current location
   * Return: player current location
   */
  inline TPos getPlayerLocation() {
    return static_cast<const TDrived *>(this)->getPlayerLocationImpl();
  }

  /*
   * Find a path to the goal and move along the path
   * Input: start locaiont and the goal
   * Return: whether movements are successful
   */
  inline bool findPathAndGo(const TPos &from, const goal::GoalBase<TPos> &goal,
                            const U64 &timeLimit = 0, const U64 &nodeLimit = 0,
                            const U64 &extraTimeLimit = 100,
                            const int &retry = 20) {
    auto run = [&](const TPos &nowFrom, const goal::GoalBase<TPos> &nowGoal)
        -> std::tuple<bool, bool,
                      TPos> {  // find path error, move error, current location
      std::cout << "Find Path... (From: " << nowFrom
                << ", To: " << goal.getGoalPosition() << ")\n"
                << std::flush;
      auto t1 = std::chrono::steady_clock::now();
      auto r = findPath(nowFrom, nowGoal, timeLimit, nodeLimit, extraTimeLimit);
      auto t2 = std::chrono::steady_clock::now();
      auto &path = std::get<1>(r);
      auto &nodeSearch = std::get<2>(r);

      std::cout << "Length: " << path->size() << std::endl;
      std::cout << "Node Search: " << nodeSearch << std::endl;
      std::cout << "Took: "
                << std::chrono::duration_cast<std::chrono::milliseconds>(t2 -
                                                                         t1)
                       .count()
                << "ms" << std::endl
                << std::flush;

      if (path->size() == 0) {
        if (std::get<0>(r) == PathResult::NOT_FOUND) {
          std::cout << "Path Not Found" << std::endl << std::flush;
        } else if (std::get<0>(r) == PathResult::TIME_LIMIT_EXCEED) {
          std::cout << "Time Limit Exceed" << std::endl << std::flush;
        } else if (std::get<0>(r) == PathResult::NODE_SEARCH_LIMIT_EXCEED) {
          std::cout << "Node Search Limit Exceed" << std::endl << std::flush;
        } else {
          std::cout << "Unknown Error" << std::endl << std::flush;
        }
        return {true, false, TPos()};
      }
      std::cout << "Executing...\n" << std::flush;
      if (!go(path)) {
        // error occur during moving
        std::cout << "Move Failed!\n" << std::flush;
        return {false, true, getPlayerLocation()};
      } else if (getPlayerLocation() !=
                 (*path)[path->size() - 1]) {  // destination isn't match
        std::cout << "Move Failed! (Destination isn't match)\n" << std::flush;
        return {false, true, getPlayerLocation()};
      } else {  // success
        std::cout << "Done\n" << std::flush;
        return {false, false, (*path)[path->size() - 1]};
      }
    };

    TPos lastPos = from, nowGoalPos;
    const TPos &goalPos = goal.getGoalPosition();
    const int step = 2 * 16;           // 2 chunks
    const int far_threshold = 3 * 16;  // 3 chunks

    for (int i = 0;; ++i) {
      // check retry time
      if (i > retry) {
        std::cout << "Exceed retry time (" << retry << ")\n" << std::flush;
        return false;
      }

      // the goal is in a unload chunk, or the goal is too far away from the
      // player
      const TPos vec = goalPos - lastPos;
      const TPos vecXZ = vec.getXZ();
      bool goal_is_in_unload_chunk =
          getBlockType(goalPos).is(BlockType::UNKNOWN);
      bool goal_is_too_far = vecXZ.sum() > far_threshold;
      if (goal_is_in_unload_chunk || goal_is_too_far) {
        const auto vecUnit = static_cast<const Vec3<double>>(vecXZ) /
                             std::sqrt(vecXZ.squaredNorm());

        nowGoalPos.x = lastPos.x + std::floor(vecUnit.x * step);
        nowGoalPos.y = lastPos.y;
        nowGoalPos.z = lastPos.z + std::floor(vecUnit.z * step);

        std::cout << "Position " << goalPos
                  << (goal_is_in_unload_chunk ? " is in an unload chunk" : "")
                  << (goal_is_too_far ? " is too far away from the player" : "")
                  << ", try to get closer " << nowGoalPos
                  << " to load the chunk." << std::endl
                  << std::flush;

        auto result = run(lastPos, goal::RangeGoal<TPos>(nowGoalPos, 5, -1, 5));
        if (std::get<0>(result)) {
          std::cout << "Find Path Error\n" << std::flush;
          return false;
        } else if (std::get<1>(result)) {
          std::cout << "Moving Error, replanning the path\n" << std::flush;
        }
        lastPos = std::get<2>(result);
        continue;
      } else {
        auto result = run(lastPos, goal);
        if (std::get<0>(result)) {
          std::cout << "Find Path Error\n" << std::flush;
          return false;
        } else if (std::get<1>(result)) {
          std::cout << "Moving Error, replanning the path\n" << std::flush;
          lastPos = std::get<2>(result);
          continue;
        } else {
          return true;
        }
      }
    }
    return false;
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
   * get minimum y of current dimension
   */
  inline int getMinY() const {
    return static_cast<const TDrived *>(this)->getMinYImpl();
  }

  /*
   * get maximum y of current dimension
   */
  inline int getMaxY() const {
    return static_cast<const TDrived *>(this)->getMaxYImpl();
  }

  /*
   * This should be implemented in subclass
   */
  virtual std::tuple<PathResult, std::shared_ptr<Path<TPos>>, U64> findPathImpl(
      const TPos &from, const goal::GoalBase<TPos> &goal, const U64 &timeLimit,
      const U64 &nodeLimit, const U64 &extraTimeLimit) const = 0;

  /*
   * This should be implemented in subclass
   */
  virtual bool goImpl(const std::shared_ptr<Path<TPos>> &path) = 0;

  /*
   * This should be implemented in subclass
   */
  virtual TPos getPlayerLocationImpl() const = 0;

  /*
   * This should be implemented in subclass
   */
  virtual BlockType getBlockTypeImpl(const TPos &pos) const = 0;

  /*
   * This should be implemented in subclass
   */
  virtual float getFallDamageImpl(
      const TPos &landingPos,
      const typename TPos::value_type &height) const = 0;

  /*
   * This should be implemented in subclass
   */
  virtual int getMinYImpl() const = 0;

  /*
   * This should be implemented in subclass
   */
  virtual int getMaxYImpl() const = 0;

 protected:
  inline bool isTimeUp(const std::chrono::steady_clock::time_point &start,
                       const U64 &timeLimit) const {
    if (timeLimit == 0) return false;
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now - start)
               .count() >= timeLimit;
  }

  std::vector<TPos> isAbleToWalkTo(const TPos &from, const TPos &XZoffset,
                                   const float &fallDamageTol) const {
    bool canJump = getBlockType(from + TPos{0, 3, 0}).is(BlockType::AIR);
    const bool isDiagonal = XZoffset.getXZ().abs().sum() > 1;
    const bool isHorizontal = XZoffset.getXZ().abs().sum() > 0;

    // check positions
    const std::vector<TPos> blocksPos = {
        from + XZoffset,
        from + XZoffset + TPos{0, 1, 0},
        from + XZoffset + TPos{0, 2, 0},
        from + XZoffset + TPos{0, 3, 0},
        from + XZoffset - TPos{0, 1, 0},
        from + TPos{0, 1, XZoffset.z},
        from + TPos{0, 2, XZoffset.z},
        from + TPos{0, 3, XZoffset.z},
        from + TPos{XZoffset.x, 1, 0},
        from + TPos{XZoffset.x, 2, 0},
        from + TPos{XZoffset.x, 3, 0},
        from + TPos{0, 1, 0},
        from + TPos{0, 2, 0},
    };
    // alias
    enum COORD : short {
      FLOOR = 0,
      FLOOR_UP1,
      FLOOR_UP2,
      FLOOR_UP3,
      FLOOR_DOWN1,
      Z_UP1,
      Z_UP2,
      Z_UP3,
      X_UP1,
      X_UP2,
      X_UP3,
      ORIG_UP1,
      ORIG_UP2,
    };
    std::vector<BlockType> blockTypes(blocksPos.size());
    for (unsigned i = 0; i < blocksPos.size(); ++i) {
      blockTypes[i] = getBlockType(blocksPos[i]);
    }
    std::vector<TPos> possiblePos;

    // walk
    if (isHorizontal && blockTypes[COORD::FLOOR_UP1].is(BlockType::AIR) &&
        blockTypes[COORD::FLOOR_UP2].is(BlockType::AIR) &&
        blockTypes[COORD::FLOOR].is(BlockType::SAFE)) {
      if (!isDiagonal || (blockTypes[COORD::X_UP1].canPass() &&
                          blockTypes[COORD::X_UP2].canPass() &&
                          blockTypes[COORD::Z_UP1].canPass() &&
                          blockTypes[COORD::Z_UP2].canPass())) {
        possiblePos.emplace_back(XZoffset);
      }
    }

    // walk + jump
    if (isHorizontal && canJump && blockTypes[COORD::FLOOR_UP2].canPass() &&
        blockTypes[COORD::FLOOR_UP3].canPass() &&
        blockTypes[COORD::FLOOR_UP1].is(BlockType::SAFE) &&
        blockTypes[COORD::ORIG_UP1].is(BlockType::AIR)) {
      if (!isDiagonal || (blockTypes[COORD::X_UP2].canPass() &&
                          blockTypes[COORD::X_UP3].canPass() &&
                          blockTypes[COORD::Z_UP2].canPass() &&
                          blockTypes[COORD::Z_UP3].canPass())) {
        possiblePos.emplace_back(XZoffset + TPos{0, 1, 0});
      }
    }

    // fall
    if (isHorizontal &&
        (blockTypes[COORD::FLOOR].is(BlockType::AIR) ||
         blockTypes[COORD::FLOOR].is(BlockType::FORCE_DOWN)) &&
        blockTypes[COORD::FLOOR_UP1].canPass() &&
        blockTypes[COORD::FLOOR_UP2].canPass() &&
        (!isDiagonal || (blockTypes[COORD::X_UP1].canPass() &&
                         blockTypes[COORD::X_UP2].canPass() &&
                         blockTypes[COORD::Z_UP1].canPass() &&
                         blockTypes[COORD::Z_UP2].canPass()))) {
      TPos landingPos = blocksPos[COORD::FLOOR];
      BlockType landingType;
      // falling
      do {
        landingPos -= TPos{0, 1, 0};
        landingType = getBlockType(landingPos);
      } while ((landingType.is(BlockType::AIR) ||
                landingType.is(BlockType::FORCE_DOWN)) &&
               landingPos.y > getMinY());
      if (landingType.is(BlockType::SAFE) &&
          getFallDamage(landingPos, (blocksPos[COORD::FLOOR] - landingPos).y) <=
              fallDamageTol) {
        possiblePos.emplace_back(landingPos - from);
      }
    }

    // climb up
    if (!isHorizontal && blockTypes[COORD::FLOOR].is(BlockType::SAFE) &&
        (blockTypes[COORD::FLOOR_UP1].is(BlockType::CAN_UP) ||
         blockTypes[COORD::FLOOR_UP1].is(BlockType::CAN_UP_DOWN)) &&
        blockTypes[COORD::FLOOR_UP1].canPass() &&
        blockTypes[COORD::FLOOR_UP2].canPass() &&
        blockTypes[COORD::FLOOR_UP3].canPass()) {
      possiblePos.emplace_back(XZoffset + TPos{0, 1, 0});
    }

    // climb down
    if (!isHorizontal && blockTypes[COORD::FLOOR_DOWN1].is(BlockType::SAFE) &&
        (blockTypes[COORD::FLOOR].is(BlockType::CAN_DOWN) ||
         blockTypes[COORD::FLOOR].is(BlockType::CAN_UP_DOWN)) &&
        blockTypes[COORD::FLOOR].canPass() &&
        blockTypes[COORD::FLOOR_UP1].canPass() &&
        blockTypes[COORD::FLOOR_UP2].canPass()) {
      possiblePos.emplace_back(XZoffset - TPos{0, 1, 0});
    }

    return possiblePos;
  }

  std::vector<TPos> isAbleToWalkFrom(const TPos &to, const TPos &XZoffset,
                                     const float &fallDamageTol) const {
    const bool isDiagonal = XZoffset.getXZ().abs().sum() > 1;
    const bool isHorizontal = XZoffset.getXZ().abs().sum() > 0;

    // check positions
    const std::vector<TPos> blocksPos = {
        to + XZoffset,
        to + XZoffset + TPos{0, 1, 0},
        to + XZoffset + TPos{0, 2, 0},
        to + XZoffset + TPos{0, 3, 0},
        to + XZoffset - TPos{0, 1, 0},
        to + TPos{0, 1, XZoffset.z},
        to + TPos{0, 2, XZoffset.z},
        to + TPos{0, 3, XZoffset.z},
        to + TPos{XZoffset.x, 1, 0},
        to + TPos{XZoffset.x, 2, 0},
        to + TPos{XZoffset.x, 3, 0},
        to,
        to + TPos{0, 3, 0},
    };
    // alias
    enum COORD : short {
      FLOOR = 0,
      FLOOR_UP1,
      FLOOR_UP2,
      FLOOR_UP3,
      FLOOR_DOWN1,
      Z_UP1,
      Z_UP2,
      Z_UP3,
      X_UP1,
      X_UP2,
      X_UP3,
      ORIG,
      ORIG_UP3,
    };
    std::vector<BlockType> blockTypes(blocksPos.size());
    for (unsigned i = 0; i < blocksPos.size(); ++i) {
      blockTypes[i] = getBlockType(blocksPos[i]);
    }
    bool canJump = blockTypes[COORD::FLOOR_UP2].is(BlockType::AIR);
    std::vector<TPos> possiblePos;

    // walk
    if (isHorizontal && blockTypes[COORD::FLOOR_UP1].is(BlockType::AIR) &&
        blockTypes[COORD::FLOOR_UP2].is(BlockType::AIR) &&
        blockTypes[COORD::FLOOR].is(BlockType::SAFE)) {
      if (!isDiagonal || (blockTypes[COORD::X_UP1].canPass() &&
                          blockTypes[COORD::X_UP2].canPass() &&
                          blockTypes[COORD::Z_UP1].canPass() &&
                          blockTypes[COORD::Z_UP2].canPass())) {
        possiblePos.emplace_back(XZoffset);
      }
    }

    // walk + jump
    if (isHorizontal && canJump && blockTypes[COORD::FLOOR_UP2].canPass() &&
        blockTypes[COORD::FLOOR_UP1].canPass() &&
        blockTypes[COORD::FLOOR].is(BlockType::AIR) &&
        blockTypes[COORD::FLOOR_DOWN1].is(BlockType::SAFE)) {
      if (!isDiagonal || (blockTypes[COORD::X_UP2].canPass() &&
                          blockTypes[COORD::X_UP3].canPass() &&
                          blockTypes[COORD::Z_UP2].canPass() &&
                          blockTypes[COORD::Z_UP3].canPass())) {
        possiblePos.emplace_back(XZoffset - TPos{0, 1, 0});
      }
    }

    // fall
    if (isHorizontal &&
        (blockTypes[COORD::ORIG_UP3].is(BlockType::AIR) ||
         blockTypes[COORD::ORIG_UP3].is(BlockType::FORCE_DOWN))) {
      TPos upPos = blocksPos[COORD::ORIG];
      // falling
      while (upPos.y < getMaxY()) {
        upPos += TPos{0, 1, 0};
        BlockType upType = getBlockType(upPos);
        if (upType.isNot(BlockType::AIR)) break;
        if (getFallDamage(to, upPos.y - blocksPos[COORD::ORIG].y) <=
            fallDamageTol) {
          TPos platformPos = upPos + XZoffset,
               platformPos_up1 = platformPos + TPos{0, 1, 0},
               platformPos_up2 = platformPos + TPos{0, 2, 0},
               platformPos_z1 = platformPos + TPos{0, 1, XZoffset.z},
               platformPos_z2 = platformPos + TPos{0, 2, XZoffset.z},
               platformPos_x1 = platformPos + TPos{XZoffset.x, 1, 0},
               platformPos_x2 = platformPos + TPos{XZoffset.x, 2, 0};
          if (getBlockType(platformPos).is(BlockType::SAFE) &&
              getBlockType(platformPos_up1).is(BlockType::AIR) &&
              getBlockType(platformPos_up2).is(BlockType::AIR) &&
              (!isDiagonal || (getBlockType(platformPos_z1).canPass() &&
                               getBlockType(platformPos_z2).canPass() &&
                               getBlockType(platformPos_x1).canPass() &&
                               getBlockType(platformPos_x2).canPass()))) {
            possiblePos.emplace_back(platformPos - to);
          }
        }
      }
    }

    // climb up
    if (!isHorizontal && blockTypes[COORD::FLOOR_DOWN1].is(BlockType::SAFE) &&
        (blockTypes[COORD::FLOOR].is(BlockType::CAN_UP) ||
         blockTypes[COORD::FLOOR].is(BlockType::CAN_UP_DOWN)) &&
        blockTypes[COORD::FLOOR].canPass()) {
      possiblePos.emplace_back(XZoffset - TPos{0, 1, 0});
    }

    // climb down
    if (!isHorizontal && blockTypes[COORD::FLOOR_UP3].canPass() &&
        (blockTypes[COORD::FLOOR_UP1].is(BlockType::CAN_DOWN) ||
         blockTypes[COORD::FLOOR_UP1].is(BlockType::CAN_UP_DOWN)) &&
        blockTypes[COORD::FLOOR_UP1].canPass()) {
      possiblePos.emplace_back(XZoffset + TPos{0, 1, 0});
    }

    return possiblePos;
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