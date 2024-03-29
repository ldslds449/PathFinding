// Copyright 2024 ldslds449

#ifndef INCLUDE_PF_VEC3_HPP_
#define INCLUDE_PF_VEC3_HPP_

#include <algorithm>
#include <cmath>
#include <functional>
#include <ostream>

#include <pf/Type.hpp>

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

  bool operator!=(const Vec3<T> &target) const {
    return (x != target.x) || (y != target.y) || (z != target.z);
  }

  bool operator<(const Vec3<T> &target) const {
    return (x < target.x) && (y < target.y) && (z < target.z);
  }

  bool operator<=(const Vec3<T> &target) const {
    return (x <= target.x) && (y <= target.y) && (z <= target.z);
  }

  bool operator>(const Vec3<T> &target) const {
    return (x > target.x) && (y > target.y) && (z > target.z);
  }

  bool operator>=(const Vec3<T> &target) const {
    return (x >= target.x) && (y >= target.y) && (z >= target.z);
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

  inline Vec3<T> offsetX(T x_offset) const {
    Vec3<T> newVec3 = *this;
    newVec3.x += x_offset;
    return newVec3;
  }

  inline Vec3<T> offsetY(T y_offset) const {
    Vec3<T> newVec3 = *this;
    newVec3.y += y_offset;
    return newVec3;
  }

  inline Vec3<T> offsetZ(T z_offset) const {
    Vec3<T> newVec3 = *this;
    newVec3.z += z_offset;
    return newVec3;
  }

  inline Vec3<T> offsetXY(T x_offset, T y_offset) const {
    Vec3<T> newVec3 = *this;
    newVec3.x += x_offset;
    newVec3.y += y_offset;
    return newVec3;
  }

  inline Vec3<T> offsetXZ(T x_offset, T z_offset) const {
    Vec3<T> newVec3 = *this;
    newVec3.x += x_offset;
    newVec3.z += z_offset;
    return newVec3;
  }

  inline Vec3<T> offsetYZ(T y_offset, T z_offset) const {
    Vec3<T> newVec3 = *this;
    newVec3.y += y_offset;
    newVec3.z += z_offset;
    return newVec3;
  }

  inline void adjust(T x_offset, T y_offset, T z_offset) {
    x += x_offset;
    y += y_offset;
    z += z_offset;
  }

  inline T maxAxisVal() { return std::max(std::max(x, y), z); }

  inline T minAxisVal() { return std::min(std::min(x, y), z); }

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

  inline T squaredNorm() const { return squaredEuclideanDist({0, 0, 0}); }

  inline T manhattanDist(const Vec3<T> &target) const {
    return std::abs(target.x - x) + std::abs(target.y - y) +
           std::abs(target.z - z);
  }

  inline T squaredEuclideanDist(const Vec3<T> &target) const {
    return (target.x - x) * (target.x - x) + (target.y - y) * (target.y - y) +
           (target.z - z) * (target.z - z);
  }

  inline std::size_t hash() const {
    std::hash<T> hasher;
    std::size_t value = hasher(x);
    value ^= hasher(y) + 0x9e3779b9 + (value << 6) + (value >> 2);
    value ^= hasher(z) + 0x9e3779b9 + (value << 6) + (value >> 2);
    return value;
  }

  friend std::ostream &operator<<(std::ostream &os, const Vec3<T> &vec3) {
    return os << ("(" + std::to_string(vec3.x) + ", " + std::to_string(vec3.y) +
                  ", " + std::to_string(vec3.z) + ")");
  }
};

using Position = Vec3<int>;

}  // namespace pathfinding

namespace std {

template <typename T>
struct hash<pathfinding::Vec3<T>> {
  inline std::size_t operator()(const pathfinding::Vec3<T> &v) const {
    return v.hash();
  }
};

}  // namespace std

#endif  // INCLUDE_PF_VEC3_HPP_
