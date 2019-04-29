//#include "main.h"
#include "robotIO.hpp"
#include "backshot/executor.hpp"
#include "auton/autons.hpp"
#include "lib/lcd/select.hpp"
/*
pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::ADIDigitalIn LeftButton(1);
pros::ADIDigitalIn CenterButton(2);
pros::ADIDigitalIn RightButton(3);

*/

int autonSelect = 0;



bool picked = false;

void ProgramChooser(void *x){


	int max = 5;
	int min = 0;
	lv_obj_t * title = lv_label_create(autonTab, NULL);
	lv_label_set_text(title, "");
	lv_obj_align(title, autonTab, LV_ALIGN_IN_TOP_MID, 0, 20);
	lv_obj_t *label = lv_label_create(autonTab, NULL);
	lv_obj_align(label, autonTab, LV_ALIGN_CENTER, 0, 0);
	while(!picked && pros::competition::is_disabled()) {

		if(LeftButton.get_value()) {
			while(LeftButton.get_value()) {}
			autonSelect --;
		}
		if(RightButton.get_value()) {
			while(RightButton.get_value()) {}
			autonSelect ++;
		}
		if(CenterButton.get_value()) {
			while(CenterButton.get_value()) {}
			picked = true;
		}
        //Brain.Screen.printAt(20,20,"LOL");
		if(autonSelect < min) autonSelect = max;
		if(autonSelect > max) autonSelect = min;
		//std::string autoName = "";
		switch(autonSelect) {
		case 0: //pros::lcd::set_text(1, "[RED][LEFT][MID+CAP]")
				lv_label_set_text(title, "[RED][LEFT][MID+CAP]");
                break;
		case 1: //pros::lcd::set_text(1 ,"[RED][LEFT][3Flag+Ramp]");
				lv_label_set_text(title, "[RED][LEFT][3Flag+Ramp]");
                break;
		case 2: //pros::lcd::set_text(1,"[BLUE][RIGHT][MID+CAP]");
				lv_label_set_text(title, "[BLUE][RIGHT][MID+CAP]");
                break;
		case 3: //pros::lcd::set_text(1,"[BLUE][RIGHT][3Flag+Ramp]");
				lv_label_set_text(title, "[BLUE][RIGHT][3Flag+Ramp]");
                break;
		case 4: //pros::lcd::set_text(1,"[BACK][Cap+Ball]");
				lv_label_set_text(title, "[BACK][Cap+Ball]");
                break;
		//case 5: autoName = "[PROG][SKILLS]"; break;
        default: //pros::lcd::set_text(1,"         None           ");
				lv_label_set_text(title, "  NONE   ");
                break;
		}
        pros::delay(20);
        //Brain.Screen.clearScreen();
	}
	if(autonSelect = 0||1||2||3){
		//pros::lcd::set_text(2,"PATH GEN INPROG");
		lv_label_set_text(label, "  PATH GEN INPROG   ");
		profileController.generatePath({
  		Point{0_in, 0_in, 0_deg},  // Profile starting position, this will normally be (0, 0, 0)
  		Point{30_in, 0_in, 0_deg}}, // The next point in the profile, 54 inch forward
  		"TileToFirstBall" // Profile name
		);
		if(autonSelect = 0||2){
			profileController.generatePath({
	  		Point{30_in, 0_in, 0_deg},  // Profile starting position, this will normally be (0, 0, 0)
	  		Point{15_in, 0_in, 0_deg}}, // The next point in the profile, 54 inch forward
	  		"FirstBallToMidCap" // Profile name
			);
		}
		if(autonSelect = 1||3||4){
			profileController.generatePath({
	  		Point{30_in, 0_in, 0_deg},  // Profile starting position, this will normally be (0, 0, 0)
	  		Point{0_in, 0_in, 0_deg}}, // The next point in the profile, 54 inch forward
	  		"FirstBallToTile" // Profile name
			);
		}

	}
    //pros::lcd::set_text(2,"AUTON SELECTED");
	lv_label_set_text(label, " AUTON SELECTED   ");

}

// /using namespace okapi;

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	lv_theme_t* alien = lv_theme_alien_init(210, &lv_font_dejavu_20);
	lv_theme_set_current(alien);

	initTabs();

	motioncontrol::init();
	pros::Task odometryTask(motioncontrol::run, nullptr, TASK_PRIORITY_DEFAULT + 1, TASK_STACK_DEPTH_DEFAULT, "Odometry");
	//pros::Task appTask(motioncontrol::runApp, nullptr, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Adaptive Pure Pursuit");
	appController.startThread();
	pros::Task anglerTask(subsystems::angler::run);
	backshot::init();

	lcd::graph::init();
	subsystems::flywheel::init();
	shooting::init();

	pros::Task intakeTask(subsystems::intake::run);
	
	pros::Task shootingTask(shooting::runTask);

	discoChassis.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);

	FlywheelMotor.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
	IntakeMotor.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
	IndexerMotor.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
	CapFlipMotor.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
	CapFlipMotor.set_encoder_units(pros::motor_encoder_units_e::E_MOTOR_ENCODER_DEGREES);
	discoChassis.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);


	// BLUE_DETECTED = pros::Vision::signature_from_utility(1, -3985, -2731, -3358, 12955, 15791, 14373, 5.7, 1);
	// GREEN_TARGET = pros::Vision::signature_from_utility(2, -3143, -2535, -2839, -4193, -3125, -3659, 3.200000047683716, 1);
	// RED_DETECTED = pros::Vision::signature_from_utility(3, 7855, 9799, 8827, -303, 335, 16, 3, 1);
	// VisionSensor.set_signature(1, &BLUE_DETECTED);
	// VisionSensor.set_signature(2, &GREEN_TARGET);
	// VisionSensor.set_signature(3, &RED_DETECTED);



	// BLUE_FLAG = VisionSensor.create_color_code(2, 1);
	// RED_FLAG = VisionSensor.create_color_code(3, 2);


	//pros::Task AutonSelector(ProgramChooser, NULL);

	lcd::initSelection();

}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}
