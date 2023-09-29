#ifndef PATHFINDING_WORLD_WORLDBASE_H_
#define PATHFINDING_WORLD_WORLDBASE_H_

#include <string>

#include "BlockType.hpp"
#include "Vec3.hpp"

namespace pathfinding {

template <class TDrived>
class WorldBase {
 public:
  template <class TVec3>
  inline std::string getBlock(const Vec3<TVec3> &pos) {
    return static_cast<TDrived *>(this)->getBlockImpl(pos);
  };

  template <class TVec3>
  inline BlockType getBlockType(const Vec3<TVec3> &pos) {
    return getBlockType(getBlock(pos));
  };

  inline BlockType getBlockType(const std::string &blockName) {
    return static_cast<TDrived *>(this)->getBlockTypeImpl(blockName);
  };

 private:
  WorldBase() {}
  friend TDrived;

  //  protected:
  //   template <class TVec3>
  //   virtual std::string getBlockImpl(const Vec3<TVec3> &pos) = 0;
};

}  // namespace pathfinding

#endif  // PATHFINDING_WORLD_WORLDBASE_H_