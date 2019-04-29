#include "robotIO.hpp"
//#include "main.h"

pathfollowing::AdaptivePurePursuit appController(
		std::make_unique<IterativePosPIDController>(0.1, 0.0, 0.0, 0.0, TimeUtilFactory::create(), std::make_unique<AverageFilter<5>>()), // straight //0.13
		std::make_unique<IterativePosPIDController>(0.5, 0.0, 0.0, 0.0, TimeUtilFactory::create(), std::make_unique<AverageFilter<5>>()), // turn
		200, 20.0); // turn was 0.6 // was 10

pros::Controller controller(pros::E_CONTROLLER_MASTER);
okapi::Controller okapicontroller;

pros::ADIDigitalIn LeftButton('E');
pros::ADIDigitalIn CenterButton('F');
pros::ADIDigitalIn RightButton('G');

okapi::ADIButton okapiLeftButton('E');
okapi::ADIButton okapiCenterButton('F');
okapi::ADIButton okapiRightButton('G');

pros::ADIDigitalIn BallDetect(8);
okapi::ADIButton BallDetectOkapi(8);

ADIEncoder rightEnc('C', 'D', true);
ADIEncoder leftEnc('A', 'B', true);

pros::Vision VisionSensor(16);

okapi::Motor FlywheelMotor(15, false, okapi::AbstractMotor::gearset::blue);

//okapi::Motor IntakeMotor(14,false);
okapi::Motor IntakeMotor(14);
okapi::Motor IndexerMotor(8);
okapi::Motor CapFlipMotor(-20);

pros::vision_signature_s_t BLUE_DETECTED;
pros::vision_signature_s_t GREEN_TARGET;
pros::vision_signature_s_t RED_DETECTED;

pros::vision_color_code_t BLUE_FLAG;
pros::vision_color_code_t RED_FLAG;

lv_obj_t *tabs;
lv_obj_t *softwareTab;
lv_obj_t *autonTab;
lv_obj_t *flywheelTab;
lv_obj_t *temperatureTab;

void initTabs()
{
	tabs = lv_tabview_create(lv_scr_act(), NULL);

	autonTab = lv_tabview_add_tab(tabs, "Auton");
	softwareTab = lv_tabview_add_tab(tabs, "Software");
	flywheelTab = lv_tabview_add_tab(tabs, "Flywheel");
	temperatureTab = lv_tabview_add_tab(tabs, "Temps");
}

using namespace okapi;

ChassisScales driveScales = {4.125_in, (9.2 * (90.4 / 82.4) * (90.0 / 70.89)) * inch}; // 4 inch wheels, 12.5 inch wheelbase width

ChassisScales trackingScales = {2.75_in, 4.6118519_in * 2.2 * (50.0 / 90.0) * (111.0 / 90.0) * (70.3 / 90.0)}; // was 4.1151

ChassisControllerIntegrated discoChassis = ChassisControllerFactory::create(
		{3, 10},											 // Left motors
		{-6, -1},											 // Right motors
		AbstractMotor::gearset::green, // Default gearset
		driveScales);

ChassisControllerPID discoPIDChassis = ChassisControllerFactory::create(
    // { 1, 2}, {3, 4},
    {3, 10}, {-6, -1},
    leftEnc, rightEnc,
    // okapi::ADIEncoder{'A', 'B'}, okapi::ADIEncoder{'C', 'D'},
    { 0.0, 0.0, 0.0 },
    { 0.0, 0.0, 0.0 },
    { 0.0, 0.0, 0.0 },
    AbstractMotor::gearset::green,
    trackingScales);

AsyncMotionProfileController profileController = AsyncControllerFactory::motionProfile(
		1.5,				 // Maximum linear velocity of the Chassis in m/s
		2.2,				 // Maximum linear acceleration of the Chassis in m/s/s
		10.0,				 // Maximum linear jerk of the Chassis in m/s/s/s
		discoChassis // Chassis Controller
);

alliance currentAlliance = blue;

mlib::IterativeTurnProfileController turnProfileController(mlib::KinematicConstraints(1.0, 0.00386), mlib::MotionProfileFollower(0.01, 0.2, 13.0, 0.6));

auto modelPtr = discoPIDChassis.getChassisModel();

mlib::TwoWheelOdometry odometry(modelPtr, trackingScales);
std::shared_ptr<mlib::PoseEstimator> poseEstimator = std::shared_ptr<mlib::PoseEstimator>(&odometry);

mlib::KinematicConstraints driveConstraints(1.05, 2.5, 10.0); // vel, accel, jerk
mlib::RamseteGains ramseteGains(0.0, 0.01, 0.0);  // zeta, beta, position derivative
// zeta between 0.0 and 1.0, dampens
// beta greater than 0.0, increases correction

mlib::RamseteProfileController ramseteProfile(ramseteGains, 1.0, modelPtr, poseEstimator, driveConstraints, driveScales, okapi::AbstractMotor::gearset::green);

pros::Task odometryTask([](void*) {
    while (true) {
        poseEstimator->update();
        pros::delay(2);
    }
});

// pros::Task odometryPrintTask([](void*) {
//     while (true) {
//         auto pose = poseEstimator->getPose();
//         controller.print(0, 0, "X: %1.2f Y: %1.2f", pose.position.getX().convert(okapi::inch), pose.position.getY().convert(okapi::inch));
//         //controller.print(0, 0, "Amc: %1.2f", motioncontrol::currAngle.convert(okapi::degree));
//         pros::delay(51);
//         controller.print(1, 0, "A: %1.2f", pose.heading.convert(degree));
//         pros::delay(51);
//         controller.print(2, 0, "!");
//         pros::delay(51);
//     }
// });

//int autonSelect= 0;

/*
using namespace okapi;
auto mainChassis = ChassisControllerFactory::create(
  {3, 10}, // Left motors
  {-6, -1},   // Right motors
  AbstractMotor::gearset::green, // Torque gearset
  {4_in, 16_in} // 4 inch wheels, 12.5 inch wheelbase width
);

*/
