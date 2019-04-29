#pragma once

//#include "api.h"
//#include "okapi/api.hpp"
//#include <tuple>
#include "robotIO.hpp"

namespace motioncontrol {
using namespace okapi;

extern QLength currX;
extern QLength currY;
extern QAngle currAngle;

extern pros::Mutex angleMutex;

void init();

void calculate();

QLength distanceToPoint(QLength x, QLength y);
QAngle angleToPoint(QLength x, QLength y);

std::tuple<QLength, QAngle> distanceAndAngleToPoint(QLength x, QLength y);

QLength distanceToPoint(QLength x, QLength y);

void run(void *p);

void turnAbsolute(QAngle angle, QTime timeout);
void turnRelative(QAngle angle, QTime timeout);

void turnAbsoluteIntegrated(QAngle angle);

void turnProfileTestAbsolute(QAngle angle);

bool stepTurnProfile(int dT, QAngle angle);
bool stepTurnProfileAbsolute(int dT, QAngle angle);
void turnProfile(QAngle angle);
void turnProfileAbsolute(QAngle angle);

void driveApp();

void waitUntilSettled();

void waitUntilSettled(int time);

void runApp(void *p);

}  // namespace motioncontrol
