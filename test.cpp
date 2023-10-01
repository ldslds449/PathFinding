#include <Client/SimpleClient.hpp>
#include <Evaluate/Evaluate.hpp>
#include <PathFinder.hpp>
#include <iostream>
#include <vector>

namespace pf = pathfinding;
using Position = pf::Position;

class TestClient final : public pf::SimpleClient<TestClient> {
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
    std::string name[] = {"minecraft:grass", "minecraft:lava", "minecraft:air"};
    return pos.y == 0 ? name[map[pos.x][pos.z]] : name[2];
  }

  virtual inline float calFallDamageImpl(
      [[maybe_unused]] const Position &landingPos,
      [[maybe_unused]] const typename Position::value_type &height)
      const override {
    return 0.0;
  }
};

int main() {
  auto client = std::make_shared<TestClient>();
  pf::PathFinder<TestClient> finder(client);

  auto path = finder.findPath<pf::eval::MaxAxisOffset>(Position{3, 0, 5},
                                                       Position{3, 0, 1});
  std::cout << (*path) << "Length: " << path->size() << std::endl;
}