#include "lib/motion/TrapezoidalProfile.hpp"

namespace mlib {
void TrapezoidalProfile::setup() {
    t1 = constraints.maxVel / constraints.maxAccel;
    d1 = constraints.maxAccel * pow(t1, 2) / 2.0;
    d2 = totalDisplacement - d1 * 2;
    t2 = d2 / constraints.maxVel;
}

ProfileOutput TrapezoidalProfile::calculate(double t) {
    if (t < t1) {
        return ProfileOutput(
            constraints.maxAccel * pow(t, 2) / 2.0,
            constraints.maxAccel * t,
            constraints.maxAccel);
    }

    t -= t1;

    if (t < t2) {
        return ProfileOutput(
            d1 + constraints.maxVel * t,
            constraints.maxVel,
            0);
    }

    t -= t2;

    if (t < t1) {
        return ProfileOutput(
            // d1 + d2 + (constraints.maxVel - constraints.maxAccel * t) * t,
            d1 + d2 + constraints.maxVel * t - constraints.maxAccel / 2.0 * pow(t, 2),
            constraints.maxVel - constraints.maxAccel * t,
            -constraints.maxAccel);
    }

    return ProfileOutput(totalDisplacement, 0, 0);
}

double TrapezoidalProfile::getTotalTime() {
    return t1 * 2 + t2;
}

TrapezoidalProfile::TrapezoidalProfile(KinematicConstraints constraints, double totalDisplacement)
    : constraints(constraints)
    , totalDisplacement(totalDisplacement) {
    setup();
}
}