#ifndef PATHFINDING_HEAP_HEAP_H_
#define PATHFINDING_HEAP_HEAP_H_

#include <exception>
#include <functional>
#include <vector>

#define L_CHILD(idx) ((idx) * 2 + 1)
#define R_CHILD(idx) ((idx) * 2 + 2)
#define PARENT(idx) ((idx - 1) / 2)

namespace pathfinding {

// not thread safe
template <class T, class TCmp>
class Heap {
 private:
  std::vector<T> arr;
  TCmp cmp;

  inline std::size_t compare(const std::size_t &idx_a, const std::size_t &idx_b,
                             const std::size_t &idx_c) {
    const std::size_t &tmp = (cmp(arr[idx_a], arr[idx_b]) ? idx_a : idx_b);
    return (cmp(arr[tmp], arr[idx_c]) ? tmp : idx_c);
  }

  inline std::size_t compare(const std::size_t &idx_a,
                             const std::size_t &idx_b) {
    return (cmp(arr[idx_a], arr[idx_b]) ? idx_a : idx_b);
  }

  void sink(std::size_t idx) {
    while (0 <= idx && idx < arr.size()) {
      std::size_t top;
      if (R_CHILD(idx) >= arr.size() && L_CHILD(idx) >= arr.size()) {
        // no children
        break;
      } else if (R_CHILD(idx) >= arr.size()) {
        top = compare(idx, L_CHILD(idx));
      } else {
        top = compare(idx, L_CHILD(idx), R_CHILD(idx));
      }
      if (top == idx) break;  // stable
      std::swap(arr[idx], arr[top]);
      idx = top;
    }
  }

  void swim(std::size_t idx) {
    while (0 <= idx && idx < arr.size()) {
      if (idx == 0) break;  // reach root
      std::size_t top = compare(idx, PARENT(idx));
      if (top == PARENT(idx)) break;  // stable
      std::swap(arr[idx], arr[top]);
      idx = PARENT(idx);
    }
  }

 public:
  T extract() {
    if (arr.size() == 0) {
      throw std::runtime_error("heap::extract - size is 0");
    } else {
      T top = arr[0];
      std::swap(arr[0], arr[arr.size() - 1]);
      arr.pop_back();
      if (arr.size() == 0) return;
      sink(0);
      return top;
    }
  }

  void insert(const T &val) {
    arr.emplace_back(val);
    swim(arr.size() - 1);
  }

  T top() {
    if (arr.size() == 0) {
      throw std::runtime_error("heap::extract - size is 0");
    } else {
      return arr[0];
    }
  }

  std::size_t size() const { return arr.size(); }

  Heap(TCmp _cmp) : cmp(_cmp) {}
};

}  // namespace pathfinding

#undef L_CHILD
#undef R_CHILD
#undef PARENT

#endif  // PATHFINDING_HEAP_HEAP_H_