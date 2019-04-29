#include "robotIO.hpp"
#include "auton/autons.hpp"
#include "lib/lcd/select.hpp"
//#include "initialize.cpp"
/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
extern int autonSelect;
void RedLeftMidFlag()
{
    IntakeMotor.moveVelocity(200);
    profileController.setTarget("TileToFirstBall");
    profileController.waitUntilSettled();
    profileController.removePath("TileToFirstBall");
    profileController.setTarget("FirstBallToMidCap", true);
}
void appcSquare()
{
    path::Line testus(
        {0_in, 0_in},
        {0_in, 60_in},
        200,  // resolution
        200); // lookahead

    path::Line testusProcedural(
        {0_in, 60_in},
        {-80_in, 60_in},
        200,
        200);

    path::Line testusProceduralProcedural(
        {-80_in, 60_in},
        {-80_in, 0_in},
        200,
        200);

    path::Line testusProceduralProceduralProcedural(
        {-80_in, 0_in},
        {0_in, 0_in},
        200,
        200);

    appController.setPath(&testus);

    pros::Controller master(pros::E_CONTROLLER_MASTER);

    while (!appController.isSettled())
    {
        //std::string targAngle("Targ: " + std::to_string(appController.getAngleTarget().convert(okapi::degree)));
        //master.set_text(2, 0, targAngle.c_str());
        appController.loop();
        pros::delay(10);
    }

    appController.setPath(&testusProcedural);
    while (!appController.isSettled())
    {
        appController.loop();
        pros::delay(10);
    }
    appController.setPath(&testusProceduralProcedural);
    while (!appController.isSettled())
    {
        appController.loop();
        pros::delay(10);
    }
    appController.setPath(&testusProceduralProceduralProcedural);
    while (!appController.isSettled())
    {
        appController.loop();
        pros::delay(10);
    }
    discoChassis.stop();
}

path::Line toBall(
    {0_in, 0_in},
    {0_in, 3_ft},
    200,
    200);

path::Line backtoBall(
    {0_in, 3_ft},
    {0_in, 0_ft},
    200,
    200);

void RedLeft3FlagPlatform() {}

void BlueRightMidFlag() {}

void BlueRight3FlagPlatform() {}

void BackCapBall() {}

void autonomous()
{
    odometry.setPose({0_in, 0_in, 0_deg});
    motioncontrol::angleMutex.take(TIMEOUT_MAX);
    motioncontrol::currX = 0_in;
    motioncontrol::currY = 0_in;
    motioncontrol::currAngle = 0_deg;
    motioncontrol::angleMutex.give();

    lcd::runAuton();

    // auto ling_long = new path::Line(
    //     {0_in, 0_in},
    //     {10_in, 20_in},
    //     1000,
    //     1000
    // );
    // appController.runPathAsync(ling_long);
    // appController.waitUntilSettled();

    // ramseteProfile.moveTo({20_in, 0_in, 0_deg});
    // ramseteProfile.moveTo({0_in, 0_in, 0_deg});
    // motioncontrol::turnAbsolute(90_deg, 1400_ms);
    // auto pose = odometry.getPose();
    // odometry.setPose({pose.position.getX(), pose.position.getY(), 0_deg});
    //ramseteProfile.moveTo({0_in, 20_in, 90_deg});

    // discoChassis.setMaxVelocity(130);
    // motioncontrol::turnAbsoluteIntegrated(90_deg);
    // discoChassis.setMaxVelocity(200);

    //blueFront();
    //blueBack();
    //redBack();
    //redFront();
    // ramseteProfile.setCorrectionMode(mlib::RamseteProfileController::CorrectionMode::None);
    // ramseteProfile.moveToAsync({18_in, -24_in, 90_deg});
    //pros::delay(700);

    //subsystems::intake::changeState(subsystems::intake::stopped);

    // ramseteProfile.waitUntilSettled();


//   discoChassis.moveDistance(19_in);

//   discoChassis.setMaxVelocity(100);
//   discoChassis.moveDistanceAsync(-19_in);
//   pros::delay(300);
//   subsystems::intake::changeState(subsystems::intake::forceIntaking);
//   discoChassis.waitUntilSettled();


    // discoChassis.setMaxVelocity(70);
    // discoChassis.turnAngle(90_deg);
    // discoChassis.setMaxVelocity(200);

    // auto toLastFlag = new path::Bezier({{1.5_ft, 1_ft},
    //                                     {1.15_ft, 1.7_ft},
    //                                     {1.2_ft, 2.5_ft},
    //                                     {2.4_ft, 2.6_ft}},
    //                                    200,
    //                                    6000);
    // appController.setStraightGains(0.1, 0.0, 0.0);
    // appController.runPathAsync(toLastFlag);
    // appController.waitUntilSettled();

    // discoChassis.setMaxVelocity(90);
    // discoChassis.turnAngle(90_deg);
    // discoChassis.setMaxVelocity(200);
    // while (true) {
    //     motioncontrol::turnAbsolute(90_deg, 6000_ms);
    //     pros::delay(500);
    //     motioncontrol::turnAbsolute(0_deg, 6000_ms);
    // }
    /*
  switch(autonSelect) {
 	case 0: RedLeftMidFlag(); break;
 	case 1: RedLeft3FlagPlatform(); break;
  case 2: BlueRightMidFlag(); break;
 	case 3: BlueRight3FlagPlatform(); break;
 	case 4: BackCapBall(); break;
 	//case 5: ProgSkills(); break;
}*/

    //discoChassis.turnAngle(90_deg);
    //motioncontrol::turnAbsolute(340_deg);
    //discoChassis.setMaxVelocity(140);
    //motioncontrol::turnAbsolute(0_deg);
    //discoChassis.waitUntilSettled();

    //pros::delay(2000);
    //discoChassis.waitUntilSettled();
}
