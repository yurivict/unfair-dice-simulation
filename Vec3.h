#pragma once

#include <boost/array.hpp>
#include <ostream>
#include <math.h>

typedef double Float;

/// Vector helper classes
enum VecCoords {X=1, Y=2, Z=3};

class Vec3 : public boost::array<Float, 3> {
public:
  typedef Float val_t;
  Vec3() { }
  Vec3(Float x1, Float x2, Float x3) : boost::array<Float, 3>({{x1, x2, x3}}) { }
  Float& operator()(unsigned idx) { // 1-based index access
    return (*this)[idx-1];
  }
  Float operator()(unsigned idx) const { // 1-based index access
    return (*this)[idx-1];
  }
  static Vec3 one(unsigned idx, Float val) {Vec3 vec(0,0,0); vec(idx) = val; return vec;}
  Float len2() const {return (*this)(X)*(*this)(X) + (*this)(Y)*(*this)(Y) + (*this)(Z)*(*this)(Z);}
  Float len() const {return sqrt(len2());}
  friend std::ostream& operator<<(std::ostream &os, const Vec3 &v) {
    os << "{" << v(X) << "," << v(Y) << "," << v(Z) << "}";
    return os;
  }
  Vec3 operator*(Float m) const {
    return Vec3((*this)(X)*m, (*this)(Y)*m, (*this)(Z)*m);
  }
  Float operator*(const Vec3 &v) const { // scalar multiplication
    return (*this)(1)*v(1) + (*this)(2)*v(2) + (*this)(3)*v(3);
  }
  Vec3 operator/(Float m) const {
    return Vec3((*this)(X)/m, (*this)(Y)/m, (*this)(Z)/m);
  }
  Vec3 operator+(const Vec3 &v) const {
    return Vec3((*this)(X)+v(X), (*this)(Y)+v(Y), (*this)(Z)+v(Z));
  }
  Vec3 operator-(const Vec3 &v) const {
    return Vec3((*this)(X)-v(X), (*this)(Y)-v(Y), (*this)(Z)-v(Z));
  }
  Vec3 operator-() const {
    return Vec3(-(*this)(X), -(*this)(Y), -(*this)(Z));
  }
};
