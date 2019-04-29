#pragma once

#include "SCurveProfile.hpp"
#include "api.h"

namespace mlib {
class MotionProfileFollower {
  public:

  double lastError;
  double dT;
  double lastTime;

  const double kP;
  const double kD;
  const double kA;
  const double kV;

  MotionProfileFollower(double kP, double kD, double kA, double kV);

  double follow(ProfileOutput output, double currDistance);
};
}