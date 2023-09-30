#include <Evaluate/Evaluate.hpp>
#include <PathFinder.hpp>
#include <World/WorldBase.hpp>
#include <iostream>

namespace pf = pathfinding;
using Position = pf::Position;

class TestWorld final : public pf::WorldBase<TestWorld, Position> {
 public:
  virtual std::string getBlockImpl(const Position &pos) const override {
    int map[9][9] = {{1, 1, 1, 1, 1, 1, 1, 1, 1}, 
                     {1, 1, 0, 1, 0, 1, 1, 1, 1},
                     {1, 1, 0, 0, 0, 0, 0, 0, 1}, 
                     {1, 0, 1, 0, 1, 0, 1, 0, 1},
                     {1, 0, 0, 1, 0, 1, 1, 0, 1}, 
                     {1, 0, 1, 0, 0, 0, 0, 0, 1},
                     {1, 0, 1, 1, 0, 0, 1, 0, 1}, 
                     {1, 0, 0, 0, 0, 0, 0, 0, 1},
                     {1, 1, 1, 1, 1, 1, 1, 1, 1}};
    std::string name[] = {"minecraft:grass", "minecraft:lava"};
    return name[map[pos.x][pos.z]];
  }

  virtual pf::BlockType getBlockTypeImpl(const std::string &blockName) const override {
    return (blockName == "minecraft:lava" ? pf::BlockType::DANGER
                                          : pf::BlockType::SAFE);
  }

  virtual inline float calFallDamageImpl(
      [[maybe_unused]] const Position &landingPos,
      [[maybe_unused]] const typename Position::value_type &height) const override {
    return 0.0;
  }
};

int main() {
  auto world = std::make_shared<TestWorld>();
  pf::PathFinder<TestWorld, Position> finder(world);

  auto path = finder.findPath<pf::eval::MaxAxisOffset>(Position{3, 0, 5},
                                                       Position{3, 0, 1});
  std::cout << (*path) << "Length: " << path->size() << std::endl;
}