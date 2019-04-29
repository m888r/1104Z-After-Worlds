#pragma once

#include "backshot/recorder.hpp"
#include "robotIO.hpp"
#include "shooting.hpp"
#include "subsystems/angler.hpp"
#include "subsystems/flywheel.hpp"
#include "motioncontrol.hpp"
#include "okapi/api.hpp"

namespace backshot {

extern bool isTurnShotting;

enum shotAngle {
  blueRightSideLeft,
  redRightSideLeft,
  blueLeftSideLeft,
  redLeftSideLeft,
  blueMiddleLeft,
  redMiddleLeft,

  blueRightSideRight,
  redRightSideRight,
  blueLeftSideRight,
  redLeftSideRight,
  blueMiddleRight,
  redMiddleRight,

  blueCenterLeft,
  redCenterLeft,
  blueCenterRight,
  redCenterRight,

  zero
};

enum turnshotState {
  turning,
  shooting,
  driving,
  nothing
};

extern turnshotState currTurnshotState;

void init();

void turnShoot(flag flagDesired, pole poleDesired);

void changeState(turnshotState state);

void execute();

void startTurnShots();

void runBackshot(void* p);
}  // namespace backshot