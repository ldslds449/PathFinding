#ifndef PATHFINDING_CLIENT_CLIENTBASE_H_
#define PATHFINDING_CLIENT_CLIENTBASE_H_

#include <string>
#include <utility>
#include <vector>

#include "BlockType.hpp"
#include "Evaluate/EvaluateBase.hpp"

namespace pathfinding {

template <class TDrived, class TPos>
class ClientBase {
 public:
  using pos_type = TPos;

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

  virtual inline BlockType getBlockTypeImpl(
      const std::string &blockName) const = 0;

  inline float calFallDamage(const TPos &landingPos,
                             const typename TPos::value_type &height) const {
    return static_cast<const TDrived *>(this)->calFallDamageImpl(landingPos,
                                                                 height);
  };

  virtual inline float calFallDamageImpl(
      const TPos &landingPos,
      const typename TPos::value_type &height) const = 0;

  inline bool move(const TPos &offset) {
    return static_cast<TDrived *>(this)->moveImpl(offset);
  };

  virtual inline bool moveImpl(const TPos &offset) = 0;
};

}  // namespace pathfinding

#endif  // PATHFINDING_CLIENT_CLIENTBASE_H_