#include "lib/motion/motionProfileFollower.hpp"

namespace mlib {
MotionProfileFollower::MotionProfileFollower(double kP, double kD, double kA,
                                             double kV)
    : kP(kP),
      kD(kD),
      kA(kA),
      kV(kV),
      lastError(0),
      lastTime(pros::millis()),
      dT(0.0) {}

double MotionProfileFollower::follow(ProfileOutput output,
                                     double currDistance) {
  dT = pros::millis() - lastTime;
  double error = output.position - currDistance;
  double power = kP * error + kD * ((error - lastError) / dT) +
                 (kV * output.velocity) + (kA * output.accel);
  lastError = error;
  lastTime += dT;

  return power;
}

}  // namespace mlib