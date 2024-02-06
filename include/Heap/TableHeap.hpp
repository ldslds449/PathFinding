#ifndef PATHFINDING_HEAP_TABLEHEAP_H_
#define PATHFINDING_HEAP_TABLEHEAP_H_

#include <exception>
#include <functional>
#include <unordered_map>
#include <vector>

#define L_CHILD(idx) ((idx) * 2 + 1)
#define R_CHILD(idx) ((idx) * 2 + 2)
#define PARENT(idx) ((idx - 1) / 2)

namespace pathfinding {

// default: min-heap
// not thread safe
template <class TKey, class TVal, class TCmp>
class TableHeap {
 private:
  // entry for table
  struct Entry {
    std::size_t idx;
    TVal val;
    Entry() = default;
    Entry(const std::size_t &_idx, const TVal &_val) : idx(_idx), val(_val) {};
  };

  std::unordered_map<TKey, Entry> table;
  std::vector<TKey> arr;
  TCmp cmp;

  inline std::size_t compare(const std::size_t &idx_a, const std::size_t &idx_b,
                             const std::size_t &idx_c) {
    const std::size_t &tmp =
        (cmp(table[arr[idx_a]].val, table[arr[idx_b]].val) ? idx_a : idx_b);
    return (cmp(table[arr[tmp]].val, table[arr[idx_c]].val) ? tmp : idx_c);
  }

  inline std::size_t compare(const std::size_t &idx_a,
                             const std::size_t &idx_b) {
    return (cmp(table[arr[idx_a]].val, table[arr[idx_b]].val) ? idx_a : idx_b);
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
      std::swap(table[arr[idx]].idx, table[arr[top]].idx);
      std::swap(arr[idx], arr[top]);
      idx = top;
    }
  }

  void swim(std::size_t idx) {
    while (0 <= idx && idx < arr.size()) {
      if (idx == 0) break;  // reach root
      std::size_t top = compare(idx, PARENT(idx));
      if (top == PARENT(idx)) break;  // stable
      std::swap(table[arr[idx]].idx, table[arr[top]].idx);
      std::swap(arr[idx], arr[top]);
      idx = top;
    }
  }

 public:
  // reutrn: if find the element
  TVal lookup(const TKey &key) {
    auto it = table.find(key);
    if (it == table.end()){
      throw std::runtime_error("heap::lookup - element is not found");
    }
    return it->second.val;
  }

  void update(const TKey &key, const TVal &val) {
    auto it = table.find(key);
    if (it != table.end()) {  // exist
      if (cmp(val, it->second.val)) {
        it->second.val = val;
        swim(it->second.idx);
      } else {
        it->second.val = val;
        sink(it->second.idx);
      }
    } else {
      arr.emplace_back(key);
      table[key] = Entry(arr.size() - 1, val);
      swim(arr.size() - 1);
    }
  }

  void compareUpdate(const TKey &key, const TVal &val) {
    auto it = table.find(key);
    if (it != table.end()) {  // exist
      if (cmp(val, it->second.val)) {
        it->second.val = val;
        swim(it->second.idx);
      }
    } else {
      arr.emplace_back(key);
      table[key] = Entry(arr.size() - 1, val);
      swim(arr.size() - 1);
    }
  }

  TKey extract() {
    if (arr.size() == 0) {
      throw std::runtime_error("heap::extract - size is 0");
    } else {
      TKey top = arr[0];
      std::swap(table[arr[0]].idx, table[arr[arr.size() - 1]].idx);
      std::swap(arr[0], arr[arr.size() - 1]);
      table.erase(arr[arr.size() - 1]);
      arr.pop_back();
      if (arr.size() == 0) return top;
      sink(0);
      return top;
    }
  }

  TKey top() {
    if (arr.size() == 0) {
      throw std::runtime_error("heap::extract - size is 0");
    } else {
      return arr[0];
    }
  }

  std::size_t size() const { return arr.size(); }

  TableHeap(TCmp _cmp) : cmp(_cmp) {}
};

}  // namespace pathfinding

#undef L_CHILD
#undef R_CHILD
#undef PARENT

#endif  // PATHFINDING_HEAP_TABLEHEAP_H_