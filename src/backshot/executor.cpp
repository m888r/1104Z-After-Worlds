#include "executor.hpp"
#include "shooting.hpp"
#include "motioncontrol.hpp"

namespace backshot
{

bool isTurnShotting = false;

turnshotState currTurnshotState = nothing;

okapi::Timer timer;

std::tuple<flag, pole> executionCombos[2] = {
    std::make_tuple(flag::nothing, pole::nothing),
    std::make_tuple(flag::nothing, pole::nothing)};

platform executionPlatform = platform::nothing;

void init()
{
  profileController.generatePath(
      {{0_in, 0_in, 0_deg},
       {5_in, 0_in, 0_deg}},
      "turn shot drive");
}

okapi::IterativePosPIDController turnController =
    okapi::IterativeControllerFactory::posPID(1.1, 0.066, 0.03, 0, // kp kd ki bias
                                              std::make_unique<AverageFilter<3>>());
okapi::SettledUtil su =
    okapi::SettledUtilFactory::create(1, 1, 0_ms); // target Error, target derivative, settle time

void changeState(turnshotState state)
{
  currTurnshotState = state;
}

QAngle getShotAngle(shotAngle sAngle)
{
  QAngle angle = 0_deg;
  switch (sAngle)
  {
  case blueLeftSideLeft: // blue alliance left pole left platform
    angle = 337_deg;
    break;
  case blueMiddleLeft:
    angle = 355_deg;
    break;
  case redLeftSideLeft:
    angle = 343_deg;
    break;
  case redMiddleLeft:
    angle = 3_deg;
    break;
  case blueRightSideLeft: // blue right pole left platform
    angle = 27.5_deg;
    break;
  case redRightSideLeft:
    angle = 30_deg;
    break;

  case blueLeftSideRight: // blue left pole right platform
    angle = 340_deg;
    break;
  case blueMiddleRight:
    angle = 351_deg;
    break;
  case redLeftSideRight:
    angle = 355_deg;
    break;
  case redMiddleRight:
    angle = 2_deg;
    break;
  case blueRightSideRight: // blue right pole right platform
    angle = 13_deg;
    break;
  case redRightSideRight:
    angle = 18_deg;
    break;
  
  case redCenterRight:
    angle = 0_deg;
    break;
  case blueCenterRight:
    angle = 0_deg;
    break;
  case redCenterLeft:
    angle = 0_deg;
    break;
  case blueCenterLeft:
    angle = 0_deg;
    break;

  default:
    angle = 0_deg;
    break;
  }
  return angle;
}

bool firstTurnRun = false;
bool firstDriveRun = false;
bool firstShootRun = false;

void execute()
{
  if (currTurnshotState != turning)
    firstTurnRun = false;
  if (currTurnshotState != driving)
    firstDriveRun = false;
  if (currTurnshotState != shooting)
    firstShootRun = false;
  switch (currTurnshotState)
  {
  case nothing:
    IndexerMotor.moveVelocity(0);
    discoChassis.stop();
    break;
  case turning:
  {
    flag targetFlag = std::get<0>(executionCombos[0]);
    switch (targetFlag) {
      case flag::high:
        subsystems::angler::changeState(subsystems::angler::turnshotTop);
        break;
      case flag::middle:
        subsystems::angler::changeState(subsystems::angler::turnshotMid);
        break;
      case flag::nothing:
        currTurnshotState = nothing;
        return;
        break;
    }

    pole targetPole = std::get<1>(executionCombos[0]);

    shotAngle angleState = zero;
    switch (currentAlliance)
    {
    case blue:
      switch (executionPlatform) {
        case platform::left:
          switch (targetPole)
          {
          case pole::left:
            angleState = redLeftSideLeft;
            break;
          case pole::right:
            angleState = redRightSideLeft;
            break;
          case pole::middle:
            angleState = redMiddleLeft;
            break;
          default:
            return;
          }
          break;
        case platform::right:
          switch (targetPole)
          {
          case pole::left:
            angleState = redLeftSideRight;
            break;
          case pole::right:
            angleState = redRightSideRight;
            break;
          case pole::middle:
            angleState = redMiddleRight;
            break;
          default:
            return;
          }
          break;
        }
      break;
    case red:
      switch (executionPlatform) {
        case platform::left:
          switch (targetPole)
          {
          case pole::left:
            angleState = blueLeftSideLeft;
            break;
          case pole::right:
            angleState = blueRightSideLeft;
            break;
          case pole::middle:
            angleState = blueMiddleLeft;
            break;
          default:
            return;
          }
          break;
        case platform::right:
          switch (targetPole)
          {
          case pole::left:
            angleState = blueLeftSideRight;
            break;
          case pole::right:
            angleState = blueRightSideRight;
            break;
          case pole::middle:
            angleState = blueMiddleRight;
            break;
          default:
            return;
          }
          break;
        }
      break;
    }

    QAngle targetAngle = getShotAngle(angleState);

    turnController.setTarget(0);

    double angleError = targetAngle.convert(okapi::radian) -
                        motioncontrol::currAngle.convert(okapi::radian);

    angleError = atan2(sin(angleError),
                       cos(angleError));
    if (!firstTurnRun) {
      if (angleError * 180.0 / 1_pi <= 25)
      {
        turnController =
            okapi::IterativeControllerFactory::posPID(1.5, 0.8, 0.05, 0, // kp ki kd bias //was 1.3 kp
                                                      std::make_unique<AverageFilter<3>>());
      } else if (angleError * 180.0 / 1_pi <= 40) {
        turnController =
            okapi::IterativeControllerFactory::posPID(1.2, 0.066, 0.03, 0, // kp ki kd bias
                                                      std::make_unique<AverageFilter<3>>());
      }
      else
      {
        turnController =
            okapi::IterativeControllerFactory::posPID(1.2, 0.066, 0.03, 0, // kp ki kd bias
                                                      std::make_unique<AverageFilter<3>>());
      }
      firstTurnRun = true;
    }
    double power = turnController.step(-angleError);
    discoChassis.arcade(0, power);

    if (!BallDetectOkapi.isPressed())
    {
      IntakeMotor.moveVoltage(12000);
    }
    else
    {
      IntakeMotor.moveVoltage(0);
    }

    if (su.isSettled(angleError * 180.0 / PI))
    {
      discoChassis.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
      discoChassis.stop();
      firstTurnRun = false;
      currTurnshotState = shooting;
    }
  }
  break;
  case driving: // switch to this state in order to start the execution and it'll auto do the rest
  {
    if (!BallDetectOkapi.isPressed())
    {
      IntakeMotor.moveVoltage(12000);
    }
    else
    {
      IntakeMotor.moveVoltage(0);
    }

    profileController.flipDisable(false);
    if (!firstDriveRun) {
      motioncontrol::angleMutex.take(TIMEOUT_MAX);
      motioncontrol::currAngle = 0_deg;
      motioncontrol::angleMutex.give();
      profileController.setTarget("turn shot drive", true);
      firstDriveRun = true;
    }

    if (profileController.isSettled())
    {
      profileController.flipDisable(true);
      firstDriveRun = false;
      currTurnshotState = turning;
    }

    break;
  }
  case shooting:
  {
    //IndexerMotor.moveVelocity(200);
    if (!BallDetectOkapi.isPressed())
    {
      IntakeMotor.moveVoltage(12000);
    }
    else
    {
      IntakeMotor.moveVoltage(0);
    }

    if (abs(390 - FlywheelMotor.getActualVelocity()) > 10) {
      printf("Flywheel not yet in velocity (Error: %d)\n", 390 - FlywheelMotor.getActualVelocity());
      break;
    }
    if (!firstShootRun) {
      if (std::get<1>(executionCombos[1]) == pole::nothing) {
        shooting::currState = shooting::disabled;
        IndexerMotor.moveVelocity(200);
      } else {
        shooting::currState = shooting::singleShot;
      }
      firstShootRun = true;
    }

    
    if (shooting::currState == shooting::notShooting)
    {
      if (std::get<0>(executionCombos[1]) == flag::nothing)
      {
        currTurnshotState = nothing;
      }
      else
      {
        IndexerMotor.moveVelocity(0);
        // move the thing forward in the array and delete the last one
        executionCombos[0] = executionCombos[1];
        executionCombos[1] = std::make_tuple(flag::nothing, pole::nothing);
        currTurnshotState = turning;
      }
    }
  }
  break;
  }
}

void startTurnShots()
{
  executionCombos[0] = getFirstTurnShot();
  executionCombos[1] = getSecondTurnShot();
  executionPlatform = currPlatform;
  currTurnshotState = driving;
  isTurnShotting = true;
}

void runBackshot(void *p)
{
  while (true)
  {
    if (isTurnShotting)
    {
      //printf("Starting turn shot\n");
      execute();
    }
    else
    {
      discoChassis.setBrakeMode(AbstractMotor::brakeMode::coast);
      profileController.flipDisable(true);
      firstTurnRun = false;
      firstDriveRun = false;
      firstShootRun = false;
    }
    pros::delay(5);
  }
}

} // namespace backshot