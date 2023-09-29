#include <Evaluate/Evaluate.hpp>
#include <PathFinder.hpp>
#include <World/WorldBase.hpp>
#include <iostream>

namespace pf = pathfinding;
using Position = pf::Position;

class TestWorld final : public pf::WorldBase<TestWorld> {
 public:
  template <class TVec3>
  std::string getBlockImpl(const pf::Vec3<TVec3> &pos) {
    int map[9][9] = {{1, 1, 1, 0, 1, 1, 1, 1, 1}, {1, 1, 0, 1, 0, 1, 1, 1, 1},
                     {1, 1, 0, 0, 0, 0, 0, 0, 1}, {1, 0, 1, 0, 1, 0, 1, 0, 1},
                     {1, 0, 0, 1, 0, 1, 1, 0, 1}, {1, 0, 1, 0, 0, 0, 0, 0, 1},
                     {1, 0, 1, 1, 0, 0, 1, 0, 1}, {1, 0, 0, 0, 0, 0, 0, 0, 1},
                     {1, 1, 1, 1, 1, 1, 1, 1, 1}};
    std::string name[] = {"minecraft:grass", "minecraft:lava"};
    return name[map[pos.x][pos.z]];
  }

  pf::BlockType getBlockTypeImpl(const std::string &blockName) {
    return (blockName == "minecraft:lava" ? pf::BlockType::DANGER
                                          : pf::BlockType::SAFE);
  }
};

int main() {
  auto world = std::make_shared<TestWorld>();
  pf::PathFinder finder;

  auto path = finder.findPath<pf::eval::MaxAxisOffset>(world, Position{3, 0, 5},
                                                       Position{3, 0, 1});
  std::cout << (*path) << "Length: " << path->size() << std::endl;
}