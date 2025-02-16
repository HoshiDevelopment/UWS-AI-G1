#ifndef _VEC_HPP_
#define _VEC_HPP_

#include "raylib.h"
#include <cmath>   // std:sin, std::cos, std::sqrt
#include <ostream> // std::ostream::operator<<

// Make this more generic using a union or std::variant?

namespace ai
{

struct Vector2;
inline float dot(const Vector2& a, const Vector2& b);

struct Vector2 : public ::Vector2
{
  using self_t = Vector2;

  Vector2() = default;
  Vector2(const self_t&) = default;
  Vector2(self_t&&) = default;
  Vector2(float x, float y) : ::Vector2{x, y} {}
  ~Vector2() = default;

  self_t &operator=(const self_t&) = default;
  self_t &operator=(self_t&&) = default;
  self_t &operator=(const float f) { x=f; y=f; return *this; }

  bool operator==(const self_t& r) const { return x==r.x && y==r.y; }

  // copy constructor and assignment from an ::Vector2
  Vector2(const ::Vector2& r) : ::Vector2{r.x, r.y} {}
  self_t &operator=(const ::Vector2& r) { x=r.x; y=r.y; return *this; }

  self_t operator+(const float f) const { return {x+f,y+f}; }
  self_t operator-(const float f) const { return {x-f,y-f}; }
  self_t operator*(const float f) const { return {x*f,y*f}; }
  friend self_t operator*(const float f, const self_t& r) { return r*f; }
  self_t operator/(const float f) const { return {x/f,y/f}; }

//  self_t operator+(const self_t &r) const { return {x+r.x, y+r.y}; }
//  self_t operator-(const self_t &r) const { return {x-r.x, y-r.y}; }
  friend self_t operator+(const self_t& l, const self_t& r) {
    return {l.x+r.x, l.y+r.y};
  }
  friend self_t operator-(const self_t& l, const self_t& r) {
    return {l.x-r.x, l.y-r.y};
  }

  self_t &operator+=(const self_t &r) { x+=r.x; y+=r.y; return *this; }
  self_t &operator/=(const self_t &r) { x/=r.x; y/=r.y; return *this; }

  self_t &operator*=(const float f) { x*=f; y*=f; return *this; }
  self_t &operator/=(const float f) { x/=f; y/=f; return *this; }

  float length()         const { return std::sqrt(dot(*this,*this)); }
  float length_squared() const { return           dot(*this,*this);  }
  void normalise() { (*this) /= (length() == 0 ? 1 : length()); }
};

inline float dot(const Vector2& a, const Vector2& b) { return a.x*b.x+a.y*b.y; }

struct Vector3;
inline float dot(const Vector3& a, const Vector3& b);

inline std::ostream &operator<<(std::ostream &os, const Vector2& v)
{
  os << '(' << v.x << ',' << v.y << ')';
  return os;
}

struct Vector3 : public ::Vector3
{
  using self_t = Vector3;

  Vector3() = default;
  Vector3(const self_t&) = default;
  Vector3(self_t&&) = default;
  Vector3(float x, float y, float z) : ::Vector3{x, y, z} {}
  ~Vector3() = default;

  self_t &operator=(const self_t&) = default;
  self_t &operator=(self_t&&) = default;
  self_t &operator=(const float f) { x=f; y=f; z=f; return *this; }

  bool operator==(const self_t& r) const { return x==r.x && y==r.y && z==r.z; }

  // copy constructor and assignment from an ::Vector3
  Vector3(const ::Vector3& r) : ::Vector3{r.x, r.y, r.z} {}
  self_t &operator=(const ::Vector3& r) { x=r.x; y=r.y; z=r.z; return *this; }

  self_t operator+(const float f) const { return {x+f,y+f,z+f}; }
  self_t operator-(const float f) const { return {x-f,y-f,z-f}; }
  self_t operator*(const float f) const { return {x*f,y*f,z*f}; }
  friend self_t operator*(const float f, const self_t& r) { return r*f; }
  self_t operator/(const float f) const { return {x/f,y/f,z/f}; }

//  self_t operator+(const self_t &r) const { return {x+r.x, y+r.y, z+r.z}; }
//  self_t operator-(const self_t &r) const { return {x-r.x, y-r.y, z-r.z}; }
  friend self_t operator+(const self_t& l, const self_t& r) {
    return {l.x+r.x, l.y+r.y, l.z+r.z};
  }
  friend self_t operator-(const self_t& l, const self_t& r) {
    return {l.x-r.x, l.y-r.y, l.z-r.z};
  }

  self_t &operator+=(const self_t &r) { x+=r.x; y+=r.y; z+=r.z; return *this; }
  self_t &operator/=(const self_t &r) { x/=r.x; y/=r.y; z/=r.z; return *this; }

  self_t &operator*=(const float f) { x*=f; y*=f; z*=f; return *this; }
  self_t &operator/=(const float f) { x/=f; y/=f; z/=f; return *this; }

  float length()         const { return std::sqrt(dot(*this,*this)); }
  float length_squared() const { return           dot(*this,*this);  }
  void normalise() { (*this) /= (length() == 0 ? 1 : length()); }
};

inline float dot(const Vector3& a, const Vector3& b) {
  return a.x*b.x + a.y*b.y + a.z*b.z;
}

inline std::ostream &operator<<(std::ostream &os, const Vector3& v)
{
  os << '(' << v.x << ',' << v.y << ',' << v.z << ')';
  return os;
}

// Returns the closest point to p on the line segment between a and b
// For many points, the closest point will be a or b (see the ternary ?:)
inline Vector3
closest_point_on_segment(const Vector3& a, const Vector3& b, const Vector3& p)
{
  const Vector3 u = b - a;
  const Vector3 v = p - a;
  const float sqr_proj   = ai::dot(u,v); // the projection distance squared
  const float sqr_length = ai::dot(u,u); // the length of u squared
  const float ratio = sqr_proj / sqr_length;
  const Vector3 delta = u * ratio;
  return sqr_proj <= 0 ? a : (sqr_proj >= sqr_length ? b : a + delta);
};

inline Vector3 asVector(const float orientation) {
  return {-std::sin(orientation), 0, std::cos(orientation)};
}

} // namespace ai

#endif // _VEC_HPP_
