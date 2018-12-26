#pragma once

#include <boost/numeric/odeint.hpp>
#include <ostream>
#include "Vec3.h"

#define RS_DO_OUTPUT(stmt...)
#define RS_DBG_TREND(stmt...)

//
// RotationSolver: solves the rotation ODE: https://en.wikipedia.org/wiki/Euler%27s_equations_(rigid_body_dynamics)
//                 caches the solution, finds the cycle
//                 F        - floating point type
//                 M[3]     - fixed torque, should normally be 0,0,0, but is present here for tests and for generality

// equations:
// I₁ẇ₁ + (I₃-I₂)w₂w₃ = M₁
// I₂ẇ₂ + (I₁-I₃)w₃w₁ = M₂
// I₃ẇ₃ + (I₂-I₁)w₁w₂ = M₃

template<typename F>
class RotationSolver {
  typedef boost::numeric::odeint::runge_kutta_dopri5<Vec3> boost_rk_stepper;
  class ODE {
    const Vec3              &I;
    const Vec3              &M;
  public:
    ODE(const Vec3 &newI, const Vec3 &newM) : I(newI), M(newM) { }
    void operator()(const Vec3 &w, Vec3 &dwdt, F t) const {
      dwdt(1) = M(1)/I(1) - (I(3)-I(2))/I(1)*w(2)*w(3);
      dwdt(2) = M(2)/I(2) - (I(1)-I(3))/I(2)*w(3)*w(1);
      dwdt(3) = M(3)/I(3) - (I(2)-I(1))/I(3)*w(1)*w(2);
      RS_DO_OUTPUT(std::cout << "RotationSolver::ODE::operator(): t=" << t << " w=" << w << " -> dwdt=" << dwdt << std::endl;)
    }
  }; // ODE
  Vec3                     I; // inertia matrix
  Vec3                     M; // torque
  F                        t0;
  F                        dt;
  F                        t;          // time to which we evolved
  boost_rk_stepper         odeStepper; // boost ODE Runge Kutta stepper, we use stepper to keep the step constant, as opposed to adaptive
  // cache cycle
  std::vector<Vec3>        cycle;      // calculated ones, up to a period
  bool                     foundCycle; // completed the cycle, the whole cycle is now in 'cycle'
  int                      evolTrend;  // +1: getting further, -1: getting closer, 0: same distance
public: // constr
  RotationSolver(const Vec3 &newI, const Vec3 &newM, const Vec3 &w0, F newT0, F newDt)
  : I(newI), M(newM), t0(newT0), dt(newDt), t(t0), foundCycle(false), evolTrend(+1) {
    cycle.push_back(w0);
    RS_DBG_TREND(std::cout << "trend: begin with +1" << std::endl;)
  }
  ~RotationSolver() {
    std::cerr << "~RotationSolver: foundCycle=" << foundCycle << std::endl;
  }
public: // iface
  Vec3 operator()(F tnew) {
    // evolve when necessary
    if (!foundCycle && tnew > t)
      evolveUpTo(tnew);

    // set time, even though N*dt has been added to it in evolveUpTo
    t = tnew;

    // return value according to the state
    return getFromCycle(tnew);
  }
private:
  void evolveUpTo(F tTo) { // evolves to the time tTo, or hits the cycle
    ODE ode(I,M);
    Vec3 x = *cycle.rbegin();
    F distPrev2 = (cycle[0]-x).len2();
    while (tTo > t) {
      // ODE solver step
      odeStepper.do_step(ode, x, t, dt);
      RS_DO_OUTPUT(std::cout << "after do_step: t=" << t << "->" << t+dt << " => x=" << x << std::endl;)
      // found the cycle?
      F distNext2 = (cycle[0]-x).len2();
      auto newTrend = deltaSign(distPrev2, distNext2);
      if (newTrend != evolTrend) {
        RS_DBG_TREND(std::cout << "trend: change @ t=" << t << ": old=" << evolTrend << " new=" << newTrend << std::endl;)
        if (newTrend < evolTrend)
          evolTrend = newTrend; // accept: begin approaching
        else { // begin going away: but are we close enough?
          Vec3 dxdt;
          ode(x, dxdt, t);
          //dxdt *= dt;
          if ((distPrev2+distNext2)/2. < (dxdt*dt).len2()*2) {
            foundCycle = true;
            RS_DBG_TREND(std::cout << "trend: Found cycle @ t=" << t << std::endl;)
            return;
          }
          RS_DBG_TREND(std::cout << "trend: Not a cycle @ t=" << t << ", just a local minimum" << std::endl;)
          evolTrend = newTrend; // accept: passed the local minimum that wasn't near zero distance, begin separatring again
        }
      } else {
        RS_DBG_TREND(std::cout << "trend: ... " << evolTrend << " no change: @t=" << t << " x=" << x << ": distPrev2=" << distPrev2 << " distNext2=" << distNext2 << std::endl;)
      }
      // move on
      cycle.push_back(x);
      distPrev2 = distNext2;
      t += dt;
    }
  }
  Vec3 getFromCycle(F tnew) const {
    if (foundCycle)
      return cycle[size_t((tnew-t0)/dt)%cycle.size()];
    else
      return *cycle.rbegin();
  }
  static int deltaSign(F f1, F f2) {
    if (f1 < f2)
      return +1;
    if (f1 > f2)
      return -1;
    return 0;
  }
}; // RotationSolver
