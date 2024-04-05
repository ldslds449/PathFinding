// Copyright 2024 ldslds449

#ifndef INCLUDE_PF_FINDER_COMPLETEFINDER_HPP_
#define INCLUDE_PF_FINDER_COMPLETEFINDER_HPP_

#include <algorithm>
#include <tuple>

#include <pf/Finder/FinderBase.hpp>

namespace pathfinding {

template <class TDrived, class TPos>
class CompleteFinder
    : public FinderBase<TDrived, CompleteFinder<TDrived, TPos>, TPos> {
 private:
  using BASE = FinderBase<TDrived, CompleteFinder<TDrived, TPos>, TPos>;

 public:
  bool findPathAndGoImpl(const TPos &from, const goal::GoalBase<TPos> &goal,
                         const std::size_t &max_interval, const U64 &timeLimit,
                         const U64 &nodeLimit, const int &retry) final {
    auto run = [&](const TPos &nowFrom, const goal::GoalBase<TPos> &nowGoal)
        -> std::tuple<bool, bool,
                      TPos> {  // find path error, move error, current location
      std::cout << "Find Path... (From: " << nowFrom
                << ", To: " << goal.getGoalPosition() << ")\n"
                << std::flush;
      auto t1 = std::chrono::steady_clock::now();
      auto r = BASE::findPath(nowFrom, nowGoal, timeLimit, nodeLimit);
      auto t2 = std::chrono::steady_clock::now();
      auto &path = std::get<1>(r);
      auto &nodeSearch = std::get<2>(r);

      auto path_len = path->size();
      path->refine(max_interval);
      auto refined_path_len = path->size();

      std::cout << "Length: " << path_len
                << ", After refinement: " << refined_path_len << std::endl;
      std::cout << "Node Search: " << nodeSearch << std::endl;
      std::cout << "Took: "
                << std::chrono::duration_cast<std::chrono::milliseconds>(t2 -
                                                                         t1)
                       .count()
                << "ms" << std::endl
                << std::flush;

      if (path->size() == 0) {
        if (std::get<0>(r) == PathResult::NOT_FOUND) {
          std::cout << "Path Not Found" << std::endl << std::flush;
        } else if (std::get<0>(r) == PathResult::TIME_LIMIT_EXCEED) {
          std::cout << "Time Limit Exceed" << std::endl << std::flush;
        } else if (std::get<0>(r) == PathResult::NODE_SEARCH_LIMIT_EXCEED) {
          std::cout << "Node Search Limit Exceed" << std::endl << std::flush;
        } else {
          std::cout << "Unknown Error" << std::endl << std::flush;
        }
        return {true, false, TPos()};
      }
      std::cout << "Executing...\n" << std::flush;
      if (!BASE::go(path)) {
        // error occur during moving
        std::cout << "Move Failed!\n" << std::flush;
        return {false, true, BASE::getPlayerLocation()};
      } else if (BASE::getPlayerLocation() !=
                 (*path)[path->size() - 1]) {  // destination isn't match
        std::cout << "Move Failed! (Destination isn't match)\n" << std::flush;
        return {false, true, BASE::getPlayerLocation()};
      } else {  // success
        std::cout << "Done\n" << std::flush;
        return {false, false, (*path)[path->size() - 1]};
      }
    };

    TPos lastPos = from, nowGoalPos;
    const TPos &goalPos = goal.getGoalPosition();
    const int far_threshold = 3 * 16;  // 3 chunks
    int threshold_discount = 100;
    const int threshold_discount_step = 20;  // 100, 80, 60, 40, 20, 0

    for (int i = 0;; ++i) {
      // check retry time
      if (i > retry) {
        std::cout << "Exceed retry time (" << retry << ")\n" << std::flush;
        return false;
      }

      // direction vector
      const TPos vec = goalPos - lastPos;
      const TPos vecXZ = vec.getXZ();
      const double vecDist = std::sqrt(vecXZ.squaredNorm());

      // current threshold and step
      const int cur_far_threshold =
          (threshold_discount <= 0 ? 0
                                   : far_threshold * threshold_discount / 100);
      const int cur_step = static_cast<int>(
          std::min(static_cast<double>(cur_far_threshold), vecDist));
      std::cout << "Far Threshold: " << cur_far_threshold
                << ", Threshold Discount: " << threshold_discount
                << ", Step: " << cur_step << std::endl;

      // the goal is in a unload chunk, or the goal is too far away from the
      // player
      bool goal_is_in_unload_chunk =
          BASE::getBlockType(goalPos).is(BlockType::UNKNOWN);
      bool goal_is_too_far = vecDist > cur_far_threshold;

      if (goal_is_in_unload_chunk || goal_is_too_far) {
        const auto vecUnit = static_cast<const Vec3<double>>(vecXZ) / vecDist;

        nowGoalPos.x = lastPos.x + static_cast<typename TPos::value_type>(
                                       std::floor(vecUnit.x * cur_step));
        nowGoalPos.y = lastPos.y;
        nowGoalPos.z = lastPos.z + static_cast<typename TPos::value_type>(
                                       std::floor(vecUnit.z * cur_step));

        std::cout << "Position " << goalPos
                  << (goal_is_in_unload_chunk ? " is in an unload chunk" : "")
                  << (!goal_is_in_unload_chunk && goal_is_too_far
                          ? " is too far away from the player"
                          : "")
                  << ", try to get closer " << nowGoalPos
                  << " to load the chunk." << std::endl
                  << std::flush;

        auto result = run(lastPos, goal::RangeGoal<TPos>(nowGoalPos, 5, -1, 5));
        if (std::get<0>(result)) {
          std::cout << "Find Path Error\n" << std::flush;
          return false;
        } else if (std::get<1>(result)) {
          std::cout << "Moving Error, replanning the path\n" << std::flush;
        }
        lastPos = std::get<2>(result);
        continue;
      } else {
        auto result = run(lastPos, goal);
        if (std::get<0>(result)) {
          std::cout << "Find Path Error\n" << std::flush;
          if (threshold_discount <= 0) {
            return false;
          }
          threshold_discount -= threshold_discount_step;
          continue;
        } else if (std::get<1>(result)) {
          std::cout << "Moving Error, replanning the path\n" << std::flush;
          lastPos = std::get<2>(result);
          threshold_discount = 100;  // reset
          continue;
        } else {
          return true;
        }
      }
    }
    return false;
  }

  CompleteFinder() = default;
  explicit CompleteFinder(const FinderConfig &_config) : BASE(_config) {}
};

}  // namespace pathfinding

#endif  // INCLUDE_PF_FINDER_COMPLETEFINDER_HPP_
