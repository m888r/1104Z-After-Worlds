#include "lib/motion/profiling.hpp"

namespace mlib {
ProfileOutput::ProfileOutput(double position, double velocity, double accel)
    : position(position)
    , velocity(velocity)
    , accel(accel) {}
}