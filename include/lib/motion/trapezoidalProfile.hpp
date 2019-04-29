#pragma once

#include "lib/motion/profiling.hpp"
#include "lib/structs.hpp"

namespace mlib {
class TrapezoidalProfile : public MotionProfile {
    void setup();

    KinematicConstraints constraints;
    double totalDisplacement;

    double t1; // time accel/decel
    double t2; // time cruise

    double d1; // distance accel/decel
    double d2; // distance cruise

public:
    TrapezoidalProfile(KinematicConstraints constraints, double displacement);

    ProfileOutput calculate(double t) override;
    double getTotalTime() override;
};
}