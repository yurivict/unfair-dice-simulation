#pragma once

#include <boost/numeric/odeint.hpp>
#include <ostream>
#include "Vec3.h"

#define RS_DO_OUTPUT(stmt...)
#define RS_DBG_TREND(stmt...)

//
// RotationSolver: solved rotation ODE: https://en.wikipedia.org/wiki/Euler%27s_equations_(rigid_body_dynamics)
//                 caches up
//                 F        - floating point type
//                 M[3]     - fixed torque, should normally be 0,0,0, but is present here for tests and for generality

template<typename F>
class RotationSolver {
  typedef boost::numeric::odeint::runge_kutta_dopri5<Vec3> boost_rk_stepper;
  class evolver {
    const Vec3              &I;
    const Vec3              &M;
  public:
    evolver(const Vec3 &newI, const Vec3 &newM) : I(newI), M(newM) { }
    void operator()(const Vec3 &w, Vec3 &dwdt, F t) const {
      dwdt(1) = M(1)/I(1) - (I(3)-I(2))/I(1)*w(2)*w(3);
      dwdt(2) = M(2)/I(2) - (I(1)-I(3))/I(2)*w(3)*w(1);
      dwdt(3) = M(3)/I(3) - (I(2)-I(1))/I(3)*w(1)*w(2);
      RS_DO_OUTPUT(std::cout << "RotationSolver::evolver::operator(): t=" << t << " w=" << w << " -> dwdt=" << dwdt << std::endl;)
    }
  }; // evolver
  Vec3                     I; // inertia matrix
  Vec3                     M; // torque
  F                        t0;
  F                        dt;
  F                        lastEvolved; // time to which we evolved
  std::vector<Vec3>        cycle; // calculated ones, up to a period
  boost_rk_stepper         odeStepper; // boost ODE Runge Kutta stepper, we use stepper to keep the step constant, as opposed to adaptive
  bool                     foundCycle; // completed the cycle, the whole cycle is now in 'cycle'
  int                      evolTrend;  // +1: getting further, -1: getting closer, 0: same distance
public: // constr
  RotationSolver(const Vec3 &newI, const Vec3 &newM, const Vec3 &w0, F newT0, F newDt)
  : I(newI), M(newM), t0(newT0), dt(newDt), lastEvolved(t0), foundCycle(false), evolTrend(+1) {
    cycle.push_back(w0);
    RS_DBG_TREND(std::cout << "trend: begin with +1" << std::endl;)
  }
  ~RotationSolver() {
    std::cerr << "~RotationSolver: foundCycle=" << foundCycle << std::endl;
  }
public: // iface
  Vec3 operator()(F t) {
    // evolve when necessary
    if (!foundCycle && t > lastEvolved)
      evolveUpTo(t);

    // return value according to the state
    if (foundCycle)
      return cycle[size_t((t-t0)/dt)%cycle.size()];
    else
      return cycle[(t-t0)/dt];
  }
private:
  void evolveUpTo(F tTo) { // evolves to the time tTo, or hits the cycle
    evolver e(I,M);
    Vec3 x = *cycle.rbegin();
    F distPrev2 = dist2(cycle[0], x);
    while (tTo > lastEvolved) {
      // ODE solver step
      odeStepper.do_step(e, x, lastEvolved, dt);
      RS_DO_OUTPUT(std::cout << "after do_step: t=" << lastEvolved << "->" << lastEvolved+dt << " => x=" << x << std::endl;)
      // found the cycle?
      F distNext2 = dist2(cycle[0], x);
      auto newTrend = deltaSign(distPrev2, distNext2);
      if (newTrend != evolTrend) {
        RS_DBG_TREND(std::cout << "trend: change @ t=" << lastEvolved << ": old=" << evolTrend << " new=" << newTrend << std::endl;)
        if (newTrend < evolTrend)
          evolTrend = newTrend; // accept: begin approaching
        else { // begin going away: but are we close enough?
          Vec3 dxdt;
          e(x, dxdt, lastEvolved);
          //dxdt *= dt;
          if ((distPrev2+distNext2)/2. < len2(dxdt*dt)*2) {
            foundCycle = true;
            RS_DBG_TREND(std::cout << "trend: Found cycle @ t=" << lastEvolved << std::endl;)
            return;
          }
          RS_DBG_TREND(std::cout << "trend: Not a cycle @ t=" << lastEvolved << ", just a local minimum" << std::endl;)
          evolTrend = newTrend; // accept: passed the local minimum that wasn't near zero distance, begin separatring again
        }
      } else {
        RS_DBG_TREND(std::cout << "trend: ... " << evolTrend << " no change: @t=" << lastEvolved << " x=" << x << ": distPrev2=" << distPrev2 << " distNext2=" << distNext2 << std::endl;)
      }
      // move on
      cycle.push_back(x);
      distPrev2 = distNext2;
      lastEvolved += dt;
    }
  }
  static F dist2(const Vec3 &x1, const Vec3 &x2) {
    return sq(x1[0]-x2[0]) + sq(x1[1]-x2[1]) + sq(x1[2]-x2[2]);
  }
  static F len2(const Vec3 &x) {
    return sq(x[0]) + sq(x[1]) + sq(x[2]);
  }
  static int deltaSign(F f1, F f2) {
    if (f1 < f2)
      return +1;
    if (f1 > f2)
      return -1;
    return 0;
  }
  static F sq(F f) {return f*f;}
}; // RotationSolver
