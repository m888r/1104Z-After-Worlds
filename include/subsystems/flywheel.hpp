#pragma once

#include "robotIO.hpp"
#include "okapi/api.hpp"

namespace subsystems
{

namespace flywheel
{

enum flywheelSpeed
{
    full = 600,
    normal = 390, //was 380
    stop = -20
};

extern flywheelSpeed currSpeed;

void init();

void run();

void changeSpeed(flywheelSpeed speed);

bool isAtRpm();

} // namespace flywheel
} // namespace subsystems
