#include "auton/autons.hpp"

void initRedBack() {

}

void redBack() {
  using namespace subsystems;
  using namespace pros;

  currentAlliance = red;

  ramseteProfile.setCorrectionMode(mlib::RamseteProfileController::CorrectionMode::Heading);

  discoChassis.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);

  flywheel::changeSpeed(subsystems::flywheel::normal);
  subsystems::intake::changeState(subsystems::intake::forceIntaking);
  ramseteProfile.setTimestep(10_ms);
  ramseteProfile.moveToAsync({34_in, 0_in, 0_deg});

  ramseteProfile.waitUntilSettled();

  ramseteProfile.moveToAsync({18_in, -24_in, 90_deg});
  delay(700);

  subsystems::intake::changeState(subsystems::intake::stopped);

  ramseteProfile.waitUntilSettled();

  // discoChassis.setMaxVelocity(130);
  // motioncontrol::turnAbsoluteIntegrated(-71.75_deg);
  // discoChassis.setMaxVelocity(200);
  
  
  motioncontrol::turnAbsolute(-77.4_deg, 2000_ms);

  while (!flywheel::isAtRpm())
  {
    pros::delay(20);
  }
  pros::delay(800); // wait for fw to settle75
  intake::changeState(subsystems::intake::forceIntaking);
  CapFlipMotor.setBrakeMode(AbstractMotor::brakeMode::hold);
  shooting::changeState(shooting::disabled);
  shooting::doubleshotCustom(0, 15, 700);
  pros::delay(500);
  intake::changeState(subsystems::intake::stopped); // stop intake before turn

  discoChassis.setMaxVelocity(130);
  motioncontrol::turnAbsoluteIntegrated(0_deg);
  discoChassis.setMaxVelocity(200);

  // CAP BALLS PHASE
  ramseteProfile.setCorrectionMode(mlib::RamseteProfileController::CorrectionMode::None);
  //odometry.setPose({0_in, 0_in, 0_deg});

  auto frontOfBallCap = new path::Line(
    {motioncontrol::currX, motioncontrol::currY},
    {17.5_in, 13.75_in},
    1000,
    1000
  );

  auto toBallCap = new path::Line(
      {motioncontrol::currX, motioncontrol::currY},
      {17.5_in, 27.5_in},
      1000,
      1000);

  auto ballCapCombo = new path::PathGroup({
    *frontOfBallCap,
    *toBallCap
  },
  2000,
  1000
  );

  appController.setStraightGains(0.05, 0.0, 0.0);
  appController.runPathAsync(ballCapCombo);

  while (motioncontrol::distanceToPoint(17.5_in, 27.5_in).convert(okapi::inch) > 6)
  {
    pros::delay(20);
  }

  shooting::scraperSpeed = 80;
  shooting::scraperTarget = -230;
  shooting::changeState(shooting::indexerReverse);

  //ramseteProfile.moveTo({18_in, 10.5_in, 0_deg});
  appController.waitUntilSettled();

  pros::delay(300);
  shooting::scraperSpeed = 200;

  discoChassis.setMaxVelocity(100);
  discoChassis.moveDistanceAsync(-19_in);
  pros::delay(300);
  intake::changeState(intake::forceIntaking);
  discoChassis.waitUntilSettled();
  discoChassis.setMaxVelocity(200);
  shooting::changeState(shooting::notShooting);

  angler::changePosition(0);

  // discoChassis.setMaxVelocity(130);
  // motioncontrol::turnAbsoluteIntegrated(-43_deg);
  // discoChassis.setMaxVelocity(200);
  //motioncontrol::turnAbsolute(-46_deg, 2000_ms);
  okapi::Motor leftOne(3);
  okapi::Motor leftTwo(10);

  leftOne.moveRelative(-460, 100);
  leftTwo.moveRelative(-460, 100);

  shooting::changeState(shooting::indexerReleased);

  while (abs(leftOne.getActualVelocity()) > 10 || fabs(leftOne.getTargetPosition() - leftOne.getPosition()) > 100) {
    pros::delay(20);
  }

  while (!flywheel::isAtRpm())
  {
    pros::delay(20);
  }

  // while (!leftOne.isStopped()) {
  //   pros::delay(20);
  // }
  // pros::delay(1000);
  odometry.setPose({0_in, 0_in, 0_deg});
  ramseteProfile.moveTo({24_in, 0_in, -1_deg});

  shooting::doubleshotCustom(0, 8, 700);

  auto frontOfPlatform = new path::Line(
      {motioncontrol::currX, motioncontrol::currY},
      {13_in, 20.5_in},
      1000,
      1000);

  auto ontoPlatform = new path::Line(
      {motioncontrol::currX, motioncontrol::currY},
      {-2_in, 20.5_in},
      1000,
      1000);

  auto comboGroup = new path::PathGroup(
    {*frontOfPlatform,
    *ontoPlatform},
    2000,
    1000);

  // appController.setStraightGains(0.05, 0.0, 0.0);
  // appController.runPathAsync(comboGroup);
  // appController.waitUntilSettled();

  // discoChassis.moveDistance(2.2_ft);
}