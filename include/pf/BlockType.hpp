#ifndef PATHFINDING_BLOCKTYPE_H_
#define PATHFINDING_BLOCKTYPE_H_

#include <pf/Type.hpp>

namespace pathfinding {

class BlockType {
 public:
  enum Property : U8 { SAFE, DANGER, AIR, UNKNOWN };
  enum Ability : U8 {
    FORCE_DOWN,
    FORCE_UP,
    CAN_UP,
    CAN_DOWN,
    CAN_UP_DOWN,
    NONE
  };

  inline bool is(const Property &flag) const { return property == flag; }
  inline bool isNot(const Property &flag) const { return property != flag; }

  inline bool is(const Ability &flag) const { return ability == flag; }
  inline bool isNot(const Ability &flag) const { return ability != flag; }

  inline bool canPass() const {
    return isNot(BlockType::UNKNOWN) &&
           (is(BlockType::AIR) || is(BlockType::CAN_UP_DOWN) ||
            is(BlockType::CAN_UP) || is(BlockType::CAN_DOWN));
  }

  BlockType(const Property &_p, const Ability &_a)
      : property(_p), ability(_a) {}
  BlockType(const BlockType &_bt) {
    property = _bt.property;
    ability = _bt.ability;
  }
  BlockType() = default;

  Property property;
  Ability ability;
};

}  // namespace pathfinding

#endif  // PATHFINDING_BLOCKTYPE_H_