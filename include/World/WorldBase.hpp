#ifndef PATHFINDING_WORLD_WORLDBASE_H_
#define PATHFINDING_WORLD_WORLDBASE_H_

#include <string>

#include "BlockType.hpp"
#include "Vec3.hpp"

namespace pathfinding {

template <class TDrived, class TPos>
class WorldBase {
 public:
  inline std::string getBlock(const TPos &pos) const {
    return static_cast<const TDrived *>(this)->getBlockImpl(pos);
  };

  virtual inline std::string getBlockImpl(const TPos &pos) const = 0;

  inline BlockType getBlockType(const TPos &pos) const {
    return getBlockType(getBlock(pos));
  };

  inline BlockType getBlockType(const std::string &blockName) const {
    return static_cast<const TDrived *>(this)->getBlockTypeImpl(blockName);
  };

  virtual inline BlockType getBlockTypeImpl(const std::string &blockName) const = 0;

  inline float calFallDamage(const TPos &landingPos,
                             const typename TPos::value_type &height) const {
    return static_cast<const TDrived *>(this)->calFallDamageImpl(landingPos, height);
  };

  virtual inline float calFallDamageImpl(
      const TPos &landingPos,
      const typename TPos::value_type &height) const = 0;

 private:
  WorldBase() {}
  friend TDrived;
};

}  // namespace pathfinding

#endif  // PATHFINDING_WORLD_WORLDBASE_H_