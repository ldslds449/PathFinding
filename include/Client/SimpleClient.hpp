#ifndef PATHFINDING_CLIENT_SIMPLECLIENT_H_
#define PATHFINDING_CLIENT_SIMPLECLIENT_H_

#include <cmath>
#include <regex>
#include <string>
#include <unordered_map>
#include <vector>

#include "ClientBase.hpp"

namespace pathfinding {

template <class TDrived, class TPos = Position>
class SimpleClient : public ClientBase<TDrived, TPos> {
 public:
  virtual inline BlockType getBlockTypeImpl(
      const std::string &blockName) const override {
    const BlockType defaultBlockType = {BlockType::SAFE, BlockType::NONE};
    const std::unordered_map<std::string, BlockType> blockTable = {
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

  virtual inline float calFallDamageImpl(
      const Position &landingPos,
      const typename Position::value_type &height) const override {
    if (height < 3.375) return 0.0;
    float damage = std::floor(height - 3.375) + 1;

    std::string landingBlock = this->getBlock(landingPos);
    if (landingBlock == "minecraft:hay_block" ||
        landingBlock == "minecraft:honey_block") {
      damage *= 0.2;
    } else if (std::regex_match(landingBlock, std::regex("minecraft:\\w+_bed"))) {
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
  }
};

}  // namespace pathfinding

#endif  // PATHFINDING_CLIENT_SIMPLECLIENT_H_