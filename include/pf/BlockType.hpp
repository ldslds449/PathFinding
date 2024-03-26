// Copyright 2024 ldslds449

#ifndef INCLUDE_PF_BLOCKTYPE_HPP_
#define INCLUDE_PF_BLOCKTYPE_HPP_

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

  BlockType() = default;

  bool operator==(const BlockType const &other) const {
    return property == other.property && ability == other.ability;
  }

  bool operator!=(const BlockType const &other) const {
    return property != other.property || ability != other.ability;
  }

  Property property = Property::SAFE;
  Ability ability = Ability::NONE;
};

}  // namespace pathfinding

#endif  // INCLUDE_PF_BLOCKTYPE_HPP_
