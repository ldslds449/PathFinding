# Pathfinding

A library written in C++17 for finding a moving path in Minecraft.

## Install

This is a header-only library, so just include the headers you need and compile them in C++17.

## Usage

Here is a simple example:

```C++
#include <Pathfinding/Pathfinding.hpp>

namespace pf = pathfinding;

template <template <class, class, class, class, class>
          class TFinder = pf::AstarFinder,
          class TEdgeEval = pf::eval::Manhattan,
          class TEstimateEval = pf::eval::Manhattan,
          class TWeight = pf::weight::ConstWeighted<1, 1>>
class BotFinder final
    : public TFinder<BotFinder<TFinder, TEdgeEval, TEstimateEval, TWeight>,
                      pf::Position, TEdgeEval, TEstimateEval,
                      TWeight> {
 public:
  virtual pf::BlockType getBlockTypeImpl(
      const pf::Position& pos) const override {
    // get the block type of the given location
    //
    // Return: the block type of the given location
    //
    // Example:
    return {pf::BlockType::SAFE,
            pf::BlockType::NONE};  // dummy
  }

  virtual inline bool goImpl(
      const std::shared_ptr<pf::Path<pf::Position>>& path)
      override {
    // move the bot
    //
    // Return: whether this movement is successful
    //
    // Warning: the first entry of the path is the starting
    // point, you can ignore it
    return true;  // dummy
  }

  virtual inline pf::Position getPlayerLocationImpl()
      const override {
    // get the current player's location
    //
    // return: the coordinate of the bot
    //
    // Warning: the coordinate of the bot is the block that
    // the bot is standing on.
    //
    // Example:
    // static_cast<int>(std::floor(player_pos.x)),
    // static_cast<int>(std::floor(player_pos.y)) - 1,
    // static_cast<int>(std::floor(player_pos.z))
    return pf::Position(0, 0, 0);  // dummy
  }

  BotFinder()
      : TFinder<BotFinder<TFinder, TEdgeEval, TEstimateEval, TWeight>,
                  pf::Position, TEdgeEval, TEstimateEval,
                  TWeight>() {}
};

int main() {
  BotFinder<> finder;
  pf::Position start(0, 0, 0), end(10, 0, 10);
  if (finder.findPathAndGo(
          start, 
          pf::goal::RangeGoal<pf::Position>(end, 0, 0, 0))) {
    std::cout << "Done" << std::endl;
  } else {
    std::cerr << "Some error occurs during the moving"
              << std::endl;
  }
}
```

---

### Finder

We offer several algorithms to handle different kinds of situations. We recommend you use the A* algorithm.

- A* (`AstarFinder`)
- Bi-directional A* (`BiAstarFinder`)
- Iterative Deeping A* (`IDAstarFinder`)
- Uniform Cost Search (`UCSFinder`)
- Iterative Lengthening Search (`ILSFinder`)
- Iterative Deeping Best First Search (`IDBestFirstFinder`)

---

### Goal

We provide several kinds of goals to meet different needs.

There is one thing to note, currently, we do not support multiple goals. We stop searching once we reach a goal.

Here is an example of building your own goal:

```C++
#include <Pathfinding/Goal/GoalBase.hpp>
#include <Pathfinding/Vec3.hpp>

namespace pf = pathfinding;

class MyGoal : public pf::goal::GoalBase<pf::Position> {
 public:
  virtual bool isSuitableGoal(
      const pf::Position &pos) const override {
    return pos == goalPos;
  }

  MyGoal(const pf::Position &_goalPos)
      : GoalBase<pf::Position>(_goalPos) {}
};
```

---

### Weighted

If you care more about the searching speed than getting an optimal solution, we also provide a custom weighting to balance the values of `g(x)` and `f(x)`.

```C++
#include <Pathfinding/Type.hpp>
#include <Pathfinding/Weighted/WeightedBase.hpp>

namespace pf = pathfinding;

class MyWeight final
    : public pf::weight::WeightedBase<MyWeight> {
 public:
  inline static pf::CostT combineImpl(const pf::CostT &g,
                                      const pf::CostT &h) {
    return g + h;
  };

  MyWeight() = delete;
};
```

---

### Evaluate

To measure the cost of a move or evaluate the cost between two points, we offer some basic measurement methods, such as Manhattan or Euclidean. You can write your evaluation function as well.

```C++
#include <Pathfinding/Evaluate/EvaluateBase.hpp>

namespace pf = pathfinding;

class MyEval final : public pf::eval::EvaluateBase<MyEval> {
 public:
  static pf::CostT evalImpl(const pf::Position &pos,
                            const pf::Position &target) {
    return (target - pos).abs().sum();
  };

  MyEval() = delete;
};
```
