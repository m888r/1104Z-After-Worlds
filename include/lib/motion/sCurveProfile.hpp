#pragma once

#include "API.h"
#include "lib/motion/profiling.hpp"
#include "lib/structs.hpp"

namespace mlib {
class SCurveProfile : public MotionProfile {
    // runs initial calculations
    void setup();

    // helpers for some of the more tedious math used in setup
    void calculateAccelPeriodDisplacement();
    void calculateVelocityConstraint();

    KinematicConstraints constraints;
    double totalDisplacement;

    double velocityConstraint();

    double accelPeriodDisplacement;
    double t1;

    double v1;
    double v2;

    double t2;
    double d1;
    double d2;

    double t3;

public:
    SCurveProfile(KinematicConstraints constraints, double displacement);

    ProfileOutput calculate(double t) override;
    double getTotalTime() override;
};
}