#ifndef PATHFINDING_BLOCKTYPE_H_
#define PATHFINDING_BLOCKTYPE_H_

#include <stdint.h>

#include "Type.hpp"

namespace pathfinding {

enum BlockType : U8 {
  SAFE,
  DANGER,
  AIR,
  UNKNOWN
};

}  // namespace pathfinding

#endif  // PATHFINDING_BLOCKTYPE_H_