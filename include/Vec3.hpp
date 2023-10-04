#ifndef PATHFINDING_VEC3_H_
#define PATHFINDING_VEC3_H_

#include <algorithm>
#include <functional>
#include <ostream>

#include "Type.hpp"

namespace pathfinding {

template <class T>
class Vec3 {
 public:
  using value_type = T;
  T x, y, z;

  Vec3(T _x, T _y, T _z) : x(_x), y(_y), z(_z) {}
  Vec3() : x(0), y(0), z(0) {}

  template <class Tnew>
  Vec3(const Vec3<Tnew> &vec) : x(vec.x), y(vec.y), z(vec.z) {}

  inline const T &getX() const { return x; }
  inline const T &getY() const { return y; }
  inline const T &getZ() const { return z; }

  inline Vec3<T> getXY() const { return Vec3<T>{x, y, 0}; }
  inline Vec3<T> getXZ() const { return Vec3<T>{x, 0, z}; }
  inline Vec3<T> getYZ() const { return Vec3<T>{0, y, z}; }

  bool operator==(const Vec3<T> &target) const {
    return (x == target.x) && (y == target.y) && (z == target.z);
  }

  Vec3<T> operator+(const Vec3<T> &target) const {
    return offset(target.x, target.y, target.z);
  }

  Vec3<T> operator+(const T &val) const { return offset(val, val, val); }

  Vec3<T> &operator+=(const Vec3<T> &target) {
    adjust(target.x, target.y, target.z);
    return *this;
  }

  Vec3<T> &operator+=(const T &val) {
    adjust(val, val, val);
    return *this;
  }

  Vec3<T> operator-() const { return Vec3<T>{-x, -y, -z}; }

  Vec3<T> operator-(const Vec3<T> &target) const {
    return offset(-target.x, -target.y, -target.z);
  }

  Vec3<T> &operator-=(const Vec3<T> &target) {
    adjust(-target.x, -target.y, -target.z);
    return *this;
  }

  Vec3<T> &operator-=(const T &val) {
    adjust(-val, -val, -val);
    return *this;
  }

  Vec3<T> operator-(const T &val) const { return offset(-val, -val, -val); }

  Vec3<T> operator*(const Vec3<T> &target) const {
    return Vec3<T>{x * target.x, y * target.y, z * target.z};
  }

  Vec3<T> operator*(const T &val) const {
    return Vec3<T>{x * val, y * val, z * val};
  }

  Vec3<T> operator/(const Vec3<T> &target) const {
    return Vec3<T>{x / target.x, y / target.y, z / target.z};
  }

  Vec3<T> operator/(const T &val) const {
    return Vec3<T>{x / val, y / val, z / val};
  }

  inline Vec3<T> offset(T x_offset, T y_offset, T z_offset) const {
    Vec3<T> newVec3 = *this;
    newVec3.x += x_offset;
    newVec3.y += y_offset;
    newVec3.z += z_offset;
    return newVec3;
  }

  inline void adjust(T x_offset, T y_offset, T z_offset) {
    x += x_offset;
    y += y_offset;
    z += z_offset;
  }

  inline Vec3<T> abs() const {
    return Vec3<T>{std::abs(x), std::abs(y), std::abs(z)};
  }

  inline Vec3<T> floor() const {
    return Vec3<T>{std::floor(x), std::floor(y), std::floor(z)};
  }

  inline Vec3<T> ceil() const {
    return Vec3<T>{std::ceil(x), std::ceil(y), std::ceil(z)};
  }

  inline T sum() const { return x + y + z; }

  template <class TReturn = T>
  inline TReturn squaredNorm() const {
    return squaredEuclideanDist<TReturn>({0, 0, 0});
  }

  inline T manhattanDist(const Vec3<T> &target) const {
    return std::abs(target.x - x) + std::abs(target.y - y) +
           std::abs(target.z - z);
  }

  template <class TReturn = T>
  inline TReturn squaredEuclideanDist(const Vec3<T> &target) const {
    return static_cast<TReturn>(target.x - x) *
               static_cast<TReturn>(target.x - x) +
           static_cast<TReturn>(target.y - y) *
               static_cast<TReturn>(target.y - y) +
           static_cast<TReturn>(target.z - z) *
               static_cast<TReturn>(target.z - z);
  }

  inline T maxAxisOffset(const Vec3<T> &target) const {
    return std::max(std::max(std::abs(target.x - x), std::abs(target.y - y)),
                    std::abs(target.z - z));
  }

  inline std::size_t hash() const {
    return std::hash<T>{}(x) ^ std::hash<T>{}(y << 1) ^ std::hash<T>{}(z << 2);
  }

  friend std::ostream &operator<<(std::ostream &os, const Vec3<T> &vec3) {
    return os << ("(" + std::to_string(vec3.x) + ", " + std::to_string(vec3.y) +
                  ", " + std::to_string(vec3.z) + ")");
  }
};

using Position = Vec3<int>;

}  // namespace pathfinding

#endif  // PATHFINDING_VEC3_H_