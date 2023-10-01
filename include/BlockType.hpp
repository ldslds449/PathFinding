#ifndef PATHFINDING_BLOCKTYPE_H_
#define PATHFINDING_BLOCKTYPE_H_

#include "Type.hpp"

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

  inline bool is(const Property &flag) { return property == flag; }

  inline bool is(const Ability &flag) { return ability == flag; }

  BlockType(const Property &_p, const Ability &_a)
      : property(_p), ability(_a) {}
  BlockType() = default;

  Property property;
  Ability ability;
};

}  // namespace pathfinding

#endif  // PATHFINDING_BLOCKTYPE_H_