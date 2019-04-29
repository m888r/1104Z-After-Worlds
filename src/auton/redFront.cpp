#include "auton/autons.hpp"

void initRedFront() {

}

void redFront() {
  using namespace subsystems;
  using namespace pros;

  currentAlliance = red;

  ramseteProfile.setCorrectionMode(mlib::RamseteProfileController::CorrectionMode::None);

  discoChassis.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
  //flywheel::run();
  flywheel::changeSpeed(subsystems::flywheel::normal);
  subsystems::intake::changeState(subsystems::intake::forceIntaking);
  ramseteProfile.setTimestep(10_ms);
  ramseteProfile.moveToAsync({34_in, 0_in, 0_deg});
  // profileController.generatePath({{0_in, 0_in, 0_deg},
  //                                 {42_in, 3_in, 0_deg}},
  //                                "To start");
  ramseteProfile.waitUntilSettled();
  //pros::delay(250);

  //profileController.setTarget("To start", true);
  ramseteProfile.moveToAsync({-5_in, 0_in, 0_deg});
  delay(700);

  subsystems::intake::changeState(subsystems::intake::stopped);
  // generate next path here LOOK HERE LOOK HERE LOOK HERE LOOK HERE LOOK HERE LOOK HERE
  ramseteProfile.waitUntilSettled();
  //profileController.waitUntilSettled();
  //subsystems::intake::changeState(subsystems::intake::intaking);

  //profileController.removePath("To start");
  //ramseteProfile.setTimestep(10_ms);
  //ramseteProfile.moveToAsync({0_in, 0_in, 0_deg});
  //ramseteProfile.waitUntilSettled();
  motioncontrol::turnAbsolute(-93_deg, 2000_ms);


  // discoChassis.setMaxVelocity(130);
  // motioncontrol::turnAbsoluteIntegrated(-80_deg);
  // discoChassis.setMaxVelocity(200);

  subsystems::intake::changeState(subsystems::intake::forceIntaking);
  // discoChassis.setMaxVelocity(100);
  // discoChassis.turnAngle(92_deg);
  // discoChassis.waitUntilSettled();
  // discoChassis.setMaxVelocity(200);
  while (!flywheel::isAtRpm())
  {
    pros::delay(20);
  }
  //shooting::doubleshot();
  shooting::doubleshotCustom(0, 58);

  //IndexerMotor.move_velocity(200);
  //while(BallDetectOkapi.isPressed()){
  //}
  //CapFlipMotor.moveAbsolute(-52,200);
  pros::delay(200);
  //CapFlipMotor.moveAbsolute(0,200);

  subsystems::intake::changeState(subsystems::intake::stopped);
  // auto toFlag = new path::Bezier({
  //   {0_in, 0_in},
  //   {15_in, 0_in},
  //   {15_in, 0_in},
  //   {32_in, 0_in}
  // },
  // 1000,
  // 250);

  auto toFlag = new path::Line(
      {0_in, -8_in},
      {-32_in, -8_in},
      1000,  // resolution
      1000); // lookhead
  discoChassis.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
  appController.setStraightGains(0.09, 0.0, 0.0);
  appController.runPathAsync(toFlag);

  shooting::scraperTarget = -120;
  while (motioncontrol::distanceToPoint(-32_in, -8_in).convert(inch) > 12)
  {
    pros::delay(20);
  }

  shooting::changeState(shooting::indexerReverse);

  appController.waitUntilSettled();
  shooting::changeState(shooting::notShooting);

  discoChassis.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
  appController.setStraightGains(0.05, 0.0, 0.0);

  auto awayFromFlag = new path::Line(
      {37_in, -6_in},
      {0.115_ft, -0.75_ft},
      200,
      1000);

  shooting::changeState(shooting::indexerReleased);

  //appController.runPathAsync(awayFromFlag);
  //profileController.setTarget("Away From Flag", true);
  //pros::delay(300);
  auto pose = odometry.getPose();
  ramseteProfile.moveToAsync({-0.6_ft, 0.25_ft, 90_deg});

  // profileController.generatePath({{0_in, 0_in, 0_deg},
  //                                 {19.5_in, 0_in, 0_deg}},
  //                                "To Cap 1");

  //appController.waitUntilSettled();
  //profileController.waitUntilSettled();
  ramseteProfile.waitUntilSettled();

  motioncontrol::turnAbsolute(-48_deg, 1500_ms);

  //intake::changeState(intake::forceIntaking);
  ramseteProfile.setCorrectionMode(mlib::RamseteProfileController::CorrectionMode::None);

  odometry.setPose({0_in, 0_in, 0_deg});
  ramseteProfile.moveToAsync({15.5_in, 0_in, 0_deg});
  pros::delay(500); 

  shooting::scraperSpeed = 50;
  shooting::scraperTarget = -220;
  shooting::changeState(shooting::indexerReverse);

  ramseteProfile.waitUntilSettled();
  shooting::scraperSpeed = 200;
  pros::delay(100);

  // profileController.setTarget("To Cap 1", true);
  // profileController.waitUntilSettled();
  // profileController.removePath("To Cap 1");
  //pros::delay(300);
  discoChassis.setMaxVelocity(100);
  discoChassis.moveDistanceAsync(-19_in);
  pros::delay(300);
  intake::changeState(intake::forceIntaking);
  discoChassis.waitUntilSettled();
  discoChassis.setMaxVelocity(200);

  //shooting::scraperTarget = -220;
  shooting::changeState(shooting::indexerReleased);
  odometry.setPose({0_in, 0_in, 0_deg});
  ramseteProfile.moveTo({5_in, 0_in, 0_deg});

  odometry.setPose({0_in, 0_in, 0_deg});
  angler::changePosition(30);
  ramseteProfile.moveToAsync({-6_in, 0_in, 0_deg});
  //motioncontrol::turnAbsolute(40_deg);
  ramseteProfile.waitUntilSettled();
  shooting::doubleshotCustom(30, 52);
  pros::delay(1000);

  // auto toLastFlag = new path::Line(
  //     {motioncontrol::currX, motioncontrol::currY},
  //     {3_ft, 3.8_ft},
  //     1000,
  //     1000);
  shooting::changeState(shooting::indexerReleased);

  auto toLastFlag = new path::Bezier({{motioncontrol::currX, motioncontrol::currY},
                                      {motioncontrol::currX + 0.5_ft, motioncontrol::currY + 1_ft},
                                      {-0.5_ft, 1.2_ft},
                                      {-2.8_ft, 1.2_ft}},
                                     6000,
                                     1100);
  appController.setStraightGains(0.1, 0.0, 0.0);
  appController.runPathAsync(toLastFlag);
  intake::changeState(intake::released);
  IntakeMotor.moveVelocity(-20);

  while (motioncontrol::distanceToPoint(-2.8_ft, 1.2_ft) > 5_in)
  {
    pros::delay(20);
  }
  shooting::scraperTarget = -110;
  shooting::changeState(shooting::indexerReverse);
  appController.waitUntilSettled();
  pros::delay(100);
  shooting::changeState(shooting::indexerReleased);
}