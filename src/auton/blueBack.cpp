#include "auton/autons.hpp"

void initBlueBack() {}

void blueBack()
{
  using namespace subsystems;
  using namespace pros;

  currentAlliance = blue;

  ramseteProfile.setCorrectionMode(mlib::RamseteProfileController::CorrectionMode::Heading);

  discoChassis.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);

  flywheel::changeSpeed(subsystems::flywheel::normal);
  subsystems::intake::changeState(subsystems::intake::forceIntaking);
  ramseteProfile.setTimestep(10_ms);
  ramseteProfile.moveToAsync({34_in, 0_in, 0_deg});

  ramseteProfile.waitUntilSettled();

  //pros::delay(2000000);

  ramseteProfile.setTimestep(10_ms);
  ramseteProfile.moveToAsync({18_in, 24_in, 90_deg}); // was 90_deg
  pros::delay(700);

  subsystems::intake::changeState(subsystems::intake::stopped);

  ramseteProfile.waitUntilSettled();

  discoChassis.setMaxVelocity(130);
  motioncontrol::turnAbsoluteIntegrated(68.75_deg);
  discoChassis.setMaxVelocity(200);

  while (!flywheel::isAtRpm())
  {
    pros::delay(20);
  }
  pros::delay(800); // wait for fw to settle
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
  ramseteProfile.setCorrectionMode(mlib::RamseteProfileController::CorrectionMode::Heading);
  //odometry.setPose({0_in, 0_in, 0_deg});

  auto toBallCap = new path::Line(
      {motioncontrol::currX, motioncontrol::currY},
      {-17_in, 24_in},
      1000,
      1000);

  appController.setStraightGains(0.05, 0.0, 0.0);
  appController.runPathAsync(toBallCap);

  while (motioncontrol::distanceToPoint(-17_in, 24_in).convert(okapi::inch) > 7)
  {
    pros::delay(20);
  }

  shooting::scraperSpeed = 80;
  shooting::scraperTarget = -220;
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

  discoChassis.setMaxVelocity(130);
  motioncontrol::turnAbsoluteIntegrated(37.2_deg);
  discoChassis.setMaxVelocity(200);

  shooting::doubleshotCustom(0, 5, 700);

  auto frontOfPlatform = new path::Line(
      {motioncontrol::currX, motioncontrol::currY},
      {-10_in, 19_in},
      1000,
      1000);

  auto ontoPlatform = new path::Line(
      {motioncontrol::currX, motioncontrol::currY},
      {2_in, 19_in},
      1000,
      1000);

  auto comboGroup = new path::PathGroup(
    {*frontOfPlatform,
    *ontoPlatform},
    2000,
    1000);

  appController.setStraightGains(0.05, 0.0, 0.0);
  appController.runPathAsync(comboGroup);
  appController.waitUntilSettled();

  discoChassis.moveDistance(2.8_ft);
}