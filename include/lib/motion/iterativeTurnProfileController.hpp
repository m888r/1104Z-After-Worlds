#pragma once
#include "okapi/api.hpp"
#include "lib/motion/motionProfileFollower.hpp"
#include "lib/motion/trapezoidalProfile.hpp"

namespace mlib {
class IterativeTurnProfileController {
  KinematicConstraints constraints;
  TrapezoidalProfile profile;
  okapi::QAngle startAngle;
  MotionProfileFollower follower;
  bool settled;
  int direction;
  okapi::Timer timer;
  okapi::QAngle currTarget;

 public:
  IterativeTurnProfileController(KinematicConstraints turnConstraints,
                                 MotionProfileFollower follower);

  void setTargetRelative(okapi::QAngle target, okapi::QAngle currAngle);
  void setTargetAbsolute(okapi::QAngle target, okapi::QAngle currAngle);

  // return power
  double step(okapi::QAngle currAngle);

  bool isSettled();
};
}  // namespace mlib