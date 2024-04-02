// Copyright 2024 ldslds449

#ifndef INCLUDE_PF_FINDER_FINDERBASE_HPP_
#define INCLUDE_PF_FINDER_FINDERBASE_HPP_

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <regex>
#include <string>
#include <thread>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <pf/BlockType.hpp>
#include <pf/Goal/Goal.hpp>
#include <pf/Path.hpp>
#include <pf/Vec3.hpp>

namespace pathfinding {

enum PathResult {
  FOUND,
  NOT_FOUND,
  TIME_LIMIT_EXCEED,
  NODE_SEARCH_LIMIT_EXCEED
};

class FinderConfig {
 public:
  bool moveDiagonally = true;
  bool jumpOverBlock = false;
  bool enableFalling = true;
  bool predictByCache = false;
};

template <class TDrived, class TMid, class TPos>
class FinderBase {
 protected:
  std::unordered_map<TPos, BlockType>
      blockTypeCache;  // cache the block types for each searching
  std::unordered_set<TPos>
      isPosUpdated;  // whether the position is updated in this searching

  FinderConfig config;

  // direction to generate next node
  class Direction {
   public:
    TPos offset;
    CostT cost;
    Direction(const TPos &_offset, const CostT &_c)
        : offset(_offset), cost(_c) {}
  };

 public:
  /*
   * Find a path to the goal
   */
  inline std::tuple<PathResult, std::shared_ptr<Path<TPos>>, U64> findPath(
      const TPos &from, const goal::GoalBase<TPos> &goal,
      const U64 &timeLimit = 0, const U64 &nodeLimit = 0) {
    if (!config.predictByCache) {
      // clear the hash table of block type before each searching
      resetCache();
    } else {
      // clear the set of updated block before each searching
      isPosUpdated.clear();
    }
    return static_cast<TDrived *>(this)->findPathImpl(from, goal, timeLimit,
                                                      nodeLimit);
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
  inline TPos getPlayerLocation() const {
    return static_cast<const TDrived *>(this)->getPlayerLocationImpl();
  }

  /*
   * Find a path to the goal and move along the path
   * Input: start locaiont and the goal
   * Return: whether movements are successful
   */
  inline bool findPathAndGo(const TPos &from, const goal::GoalBase<TPos> &goal,
                            const U64 &timeLimit = 0, const U64 &nodeLimit = 0,
                            const int &retry = 20) {
    return static_cast<TDrived *>(this)->findPathAndGoImpl(
        from, goal, timeLimit, nodeLimit, retry);
  }

  /*
   * Get the block type of the block at a specific position
   */
  inline BlockType getBlockType(const TPos &pos) {
    if (config.predictByCache) {  // if we use previous cache as a prediction
      if (isPosUpdated.count(pos) > 0) {
        // we have already seen this position before in this searching
        return blockTypeCache[pos];
      } else {
        BlockType type =
            static_cast<const TDrived *>(this)->getBlockTypeImpl(pos);
        // check whether the position is in a loaded chunk
        if (!type.is(BlockType::UNKNOWN)) {
          // the position is in a loaded chunk, so we can cache it as an update
          // and return it
          blockTypeCache.emplace(pos, type);  // cache
          isPosUpdated.insert(pos);
          return type;
        } else {
          // try to find the position in the cache
          auto it = blockTypeCache.find(pos);
          if (it != blockTypeCache.end()) {
            // we found the info in the cache, and we use it as a prediction
            // we got this position info from the previous searching
            isPosUpdated.insert(pos);
            return it->second;
          } else {
            // the position is in neither a loaded chunk nor the cache, just
            // return the result that we get
            blockTypeCache.emplace(pos, type);  // cache
            isPosUpdated.insert(pos);
            return type;
          }
        }
      }
    } else {  // just cache the block info only for this searching
      auto it = blockTypeCache.find(pos);
      if (it != blockTypeCache.end()) {
        return it->second;
      }
      BlockType type =
          static_cast<const TDrived *>(this)->getBlockTypeImpl(pos);
      blockTypeCache.emplace(pos, type);  // cache
      return type;
    }
  }

  /*
   * Clear block type cache
   */
  inline void resetCache() { blockTypeCache.clear(); }

  /*
   * calculate the fall damage
   */
  inline float getFallDamage(const TPos &landingPos,
                             const typename TPos::value_type &height) const {
    return static_cast<const TDrived *>(this)->getFallDamageImpl(landingPos,
                                                                 height);
  }

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
   * get falling damage tolerance
   */
  inline float getfallingDamageTolerance() const {
    return static_cast<const TDrived *>(this)->getfallingDamageToleranceImpl();
  }

  /*
   * get falling damage tolerance
   */
  inline CostT getBlockExtraCost(const TPos &pos) const {
    return static_cast<const TDrived *>(this)->getBlockExtraCostImpl(pos);
  }

  /*
   * This should be implemented in subclass
   */
  virtual std::tuple<PathResult, std::shared_ptr<Path<TPos>>, U64> findPathImpl(
      const TPos &from, const goal::GoalBase<TPos> &goal, const U64 &timeLimit,
      const U64 &nodeLimit) = 0;

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
  virtual bool findPathAndGoImpl(const TPos &from,
                                 const goal::GoalBase<TPos> &goal,
                                 const U64 &timeLimit, const U64 &nodeLimit,
                                 const int &retry) = 0;

  /*
   * This should be implemented in subclass
   */
  virtual BlockType getBlockTypeImpl(const TPos &pos) const = 0;

  /*
   * This may be override in subclass
   */
  virtual float getFallDamageImpl(
      [[maybe_unused]] const TPos &landingPos,
      const typename TPos::value_type &height) const {
    // do not apply any damage reduction
    // you can override this function to calculate a more precise damage value.
    if (height < 3.375) return 0.0;
    float damage = static_cast<float>(std::floor(height - 3.375)) + 1;
    return damage;
  }

  /*
   * This may be override in subclass
   */
  virtual int getMinYImpl() const { return -64; }

  /*
   * This may be override in subclass
   */
  virtual int getMaxYImpl() const { return 320; }

  /*
   * This may be override in subclass
   */
  virtual float getfallingDamageToleranceImpl() const { return 0.0; }

  /*
   * This may be override in subclass
   */
  virtual CostT getBlockExtraCostImpl([[maybe_unused]] const TPos &pos) const {
    return 0.0;
  }

 protected:
  inline bool isTimeUp(const std::chrono::steady_clock::time_point &start,
                       const U64 &timeLimit) const {
    if (timeLimit == 0) return false;
    auto now = std::chrono::steady_clock::now();
    return static_cast<U64>(
               std::chrono::duration_cast<std::chrono::milliseconds>(now -
                                                                     start)
                   .count()) >= timeLimit;
  }

  std::vector<TPos> isAbleToWalkTo(const TPos &from, const TPos &XZoffset) {
    bool canJump = getBlockType(from + TPos{0, 3, 0}).is(BlockType::AIR);
    const bool isDiagonal = XZoffset.getXZ().abs().sum() > 1;
    const bool isHorizontal_1 = XZoffset.getXZ().abs().sum() == 1;
    const bool isHorizontal_2 = XZoffset.getXZ().abs().sum() == 2;
    const bool noHorizontal = !(isHorizontal_1 || isHorizontal_2);

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
    enum COORD : int16_t {
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
    if (isHorizontal_1 && blockTypes[COORD::FLOOR_UP1].is(BlockType::AIR) &&
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
    if (isHorizontal_1 && canJump && blockTypes[COORD::FLOOR_UP2].canPass() &&
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

    // jump over a block
    if (isHorizontal_2 && canJump &&
        blockTypes[COORD::FLOOR].is(BlockType::SAFE) &&
        blockTypes[COORD::FLOOR_UP1].is(BlockType::AIR) &&
        blockTypes[COORD::FLOOR_UP2].is(BlockType::AIR) &&
        blockTypes[COORD::FLOOR_UP3].is(BlockType::AIR) &&
        getBlockType(from + (XZoffset / 2)).is(BlockType::AIR) &&
        getBlockType(from + (XZoffset / 2) + TPos{0, 1, 0})
            .is(BlockType::AIR) &&
        getBlockType(from + (XZoffset / 2) + TPos{0, 2, 0})
            .is(BlockType::AIR) &&
        getBlockType(from + (XZoffset / 2) + TPos{0, 3, 0})
            .is(BlockType::AIR)) {
      possiblePos.emplace_back(XZoffset);
    }

    // fall
    if (config.enableFalling && isHorizontal_1 &&
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
              getfallingDamageTolerance()) {
        possiblePos.emplace_back(landingPos - from);
      }
    }

    // climb up
    if (noHorizontal && blockTypes[COORD::FLOOR].is(BlockType::SAFE) &&
        (blockTypes[COORD::FLOOR_UP1].is(BlockType::CAN_UP) ||
         blockTypes[COORD::FLOOR_UP1].is(BlockType::CAN_UP_DOWN)) &&
        blockTypes[COORD::FLOOR_UP1].canPass() &&
        blockTypes[COORD::FLOOR_UP2].canPass() &&
        blockTypes[COORD::FLOOR_UP3].canPass()) {
      possiblePos.emplace_back(XZoffset + TPos{0, 1, 0});
    }

    // climb down
    if (noHorizontal && blockTypes[COORD::FLOOR_DOWN1].is(BlockType::SAFE) &&
        (blockTypes[COORD::FLOOR].is(BlockType::CAN_DOWN) ||
         blockTypes[COORD::FLOOR].is(BlockType::CAN_UP_DOWN)) &&
        blockTypes[COORD::FLOOR].canPass() &&
        blockTypes[COORD::FLOOR_UP1].canPass() &&
        blockTypes[COORD::FLOOR_UP2].canPass()) {
      possiblePos.emplace_back(XZoffset - TPos{0, 1, 0});
    }

    return possiblePos;
  }

  std::vector<TPos> isAbleToWalkFrom(const TPos &to, const TPos &XZoffset) {
    const bool isDiagonal = XZoffset.getXZ().abs().sum() > 1;
    const bool isHorizontal_1 = XZoffset.getXZ().abs().sum() == 1;
    const bool isHorizontal_2 = XZoffset.getXZ().abs().sum() == 2;
    const bool noHorizontal = !(isHorizontal_1 || isHorizontal_2);

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
    enum COORD : int16_t {
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
    bool canJump = blockTypes[COORD::FLOOR_UP3].is(BlockType::AIR);
    std::vector<TPos> possiblePos;

    // walk
    if (isHorizontal_1 && blockTypes[COORD::FLOOR_UP1].is(BlockType::AIR) &&
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
    if (isHorizontal_1 && canJump && blockTypes[COORD::FLOOR_UP2].canPass() &&
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

    // jump over a block
    if (isHorizontal_2 && canJump &&
        blockTypes[COORD::FLOOR].is(BlockType::SAFE) &&
        blockTypes[COORD::FLOOR_UP1].is(BlockType::AIR) &&
        blockTypes[COORD::FLOOR_UP2].is(BlockType::AIR) &&
        blockTypes[COORD::ORIG_UP3].is(BlockType::AIR) &&
        getBlockType(to + (XZoffset / 2)).is(BlockType::AIR) &&
        getBlockType(to + (XZoffset / 2) + TPos{0, 1, 0}).is(BlockType::AIR) &&
        getBlockType(to + (XZoffset / 2) + TPos{0, 2, 0}).is(BlockType::AIR) &&
        getBlockType(to + (XZoffset / 2) + TPos{0, 3, 0}).is(BlockType::AIR)) {
      possiblePos.emplace_back(XZoffset);
    }

    // fall
    if (config.enableFalling && isHorizontal_1 &&
        (blockTypes[COORD::ORIG_UP3].is(BlockType::AIR) ||
         blockTypes[COORD::ORIG_UP3].is(BlockType::FORCE_DOWN))) {
      TPos upPos = blocksPos[COORD::ORIG];
      // falling
      while (upPos.y < getMaxY()) {
        upPos += TPos{0, 1, 0};
        BlockType upType = getBlockType(upPos);
        if (upType.isNot(BlockType::AIR)) break;
        if (getFallDamage(to, upPos.y - blocksPos[COORD::ORIG].y) <=
            getfallingDamageTolerance()) {
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
    if (noHorizontal && blockTypes[COORD::FLOOR_DOWN1].is(BlockType::SAFE) &&
        (blockTypes[COORD::FLOOR].is(BlockType::CAN_UP) ||
         blockTypes[COORD::FLOOR].is(BlockType::CAN_UP_DOWN)) &&
        blockTypes[COORD::FLOOR].canPass()) {
      possiblePos.emplace_back(XZoffset - TPos{0, 1, 0});
    }

    // climb down
    if (noHorizontal && blockTypes[COORD::FLOOR_UP3].canPass() &&
        (blockTypes[COORD::FLOOR_UP1].is(BlockType::CAN_DOWN) ||
         blockTypes[COORD::FLOOR_UP1].is(BlockType::CAN_UP_DOWN)) &&
        blockTypes[COORD::FLOOR_UP1].canPass()) {
      possiblePos.emplace_back(XZoffset + TPos{0, 1, 0});
    }

    return possiblePos;
  }

  inline bool isGoalExist(const goal::GoalBase<TPos> &goal) {
    const TPos &p = goal.getGoalPosition();
    return getBlockType(p).is(BlockType::SAFE) &&
           getBlockType(p.offset(0, 1, 0)).is(BlockType::AIR) &&
           getBlockType(p.offset(0, 2, 0)).is(BlockType::AIR);
  }

  template <class TEdgeEval>
  inline std::vector<Direction> getDirections() {
    std::vector<Direction> directions;

    // 4/8 directions
    for (int x = -1; x <= 1; ++x) {
      for (int z = -1; z <= 1; ++z) {
        TPos offset{x, 0, z};
        if (!config.moveDiagonally && offset.abs().getXZ().sum() > 1) continue;
        directions.emplace_back(offset, TEdgeEval::eval(offset));
      }
    }

    //    O
    //
    // O  X  O
    //
    //    O
    if (config.jumpOverBlock) {
      directions.emplace_back(TPos{2, 0, 0}, TEdgeEval::eval(TPos{2, 1, 0}));
      directions.emplace_back(TPos{0, 0, 2}, TEdgeEval::eval(TPos{0, 1, 2}));
      directions.emplace_back(TPos{-2, 0, 0}, TEdgeEval::eval(TPos{-2, 1, 0}));
      directions.emplace_back(TPos{0, 0, -2}, TEdgeEval::eval(TPos{0, 1, -2}));
    }

    return directions;
  }

 private:
  FinderBase() = default;
  explicit FinderBase(const FinderConfig &_config) : config(_config) {}

 private:
  friend TDrived;
  friend TMid;
};

}  // namespace pathfinding

#endif  // INCLUDE_PF_FINDER_FINDERBASE_HPP_
