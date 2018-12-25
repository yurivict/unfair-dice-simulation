#pragma once

//
// RotationMatrixIntegrator integrates rotation evolution in the object space and produces rotation evolution in the original coordinate system
// https://en.wikipedia.org/wiki/Rotation_matrix#Rotation_matrix_from_axis_and_angle
//

template<typename F, class RSolver>
class RotationMatrixIntegrator {
  RSolver *rotationSolver;
  Mat3     m;
  F        t;
  F        dt;
public:
  RotationMatrixIntegrator(RSolver *newRotationSolver, const Vec3 &rot0, F newT, F newDt) : rotationSolver(newRotationSolver), m(Mat3::rotate(rot0)), t(newT), dt(newDt) { }
  ~RotationMatrixIntegrator() {
    //std::cerr << "~RotationMatrixIntegrator: @t=" << std::setw( 15 ) << std::setprecision( 9 ) << t << std::endl;
  }
public:
  Mat3 operator()(F te) {
    while (t < te) {
      Vec3 rotVec = (*rotationSolver)(t);
      F rotLen = rotVec.len();
      if (rotLen != 0)
        m = deltaF(rotVec/rotLen, rotLen*dt)*m;
      //std::cout << "RotationMatrixIntegrator: @t=" << t << " m=" << m << std::endl;
      t += dt;
    }
    return m;
  }
private:
  static Mat3 delta1(const Vec3 &u, F theta) { // for small rotation theta around the unit exis u: XXX fast but leads to a lot of errors
    return Mat3(
      Vec3(1,             -u(3)*theta,   u(2)*theta),
      Vec3(u(3)*theta,    1,             -u(1)*theta),
      Vec3(-u(2)*theta,   u(1)*theta,    1)
    );
  }
  static Mat3 deltaF(const Vec3 &u, F theta) { // for small rotation theta around the unit exis u: XXX expensive but is much more accurate than delta1
    return Mat3::rotate(u, theta);
  }
}; // RotationMatrixIntegrator

