#pragma once
#ifndef ___ROBOTIO_HPP

#include "main.h"
#include "adaptivepurepursuit.hpp"
#include "lib/motion/iterativeTurnProfileController.hpp"
#include "lib/structs.hpp"
#include "lib/motion/ramseteProfileFollower.hpp"
#include "lib/motion/odometry.hpp"

//#include "/Users/abhinavsaxena/Documents/mopro/include/main.h"



#define ___ROBOTIO_HPP
extern pathfollowing::AdaptivePurePursuit appController;
extern const lv_img_t EthansMeme;

extern pros::Controller controller;
extern pros::ADIDigitalIn LeftButton;
extern pros::ADIDigitalIn CenterButton;
extern pros::ADIDigitalIn RightButton;
extern pros::ADIDigitalIn BallDetect;
extern okapi::ADIButton okapiLeftButton;
extern okapi::ADIButton okapiCenterButton;
extern okapi::ADIButton okapiRightButton;
extern okapi::ADIEncoder rightEnc;
extern okapi::ADIEncoder leftEnc;
extern okapi::ADIButton BallDetectOkapi;
extern okapi::Motor FlywheelMotor;
extern okapi::Motor IntakeMotor;
extern okapi::Motor IndexerMotor;
extern okapi::Motor CapFlipMotor;
extern pros::Vision VisionSensor;
extern okapi::ControllerButton upButton;
extern okapi::ControllerButton downButton;
extern okapi::Controller okapicontroller;

extern pros::vision_signature_s_t BLUE_DETECTED;
extern pros::vision_signature_s_t GREEN_TARGET;
extern pros::vision_signature_s_t RED_DETECTED;

extern pros::vision_color_code_t BLUE_FLAG;
extern pros::vision_color_code_t RED_FLAG;

extern mlib::IterativeTurnProfileController turnProfileController;
extern mlib::RamseteProfileController ramseteProfile;
extern mlib::TwoWheelOdometry odometry;

extern lv_obj_t* tabs;
extern lv_obj_t* flywheelTab;
extern lv_obj_t* autonTab;
extern lv_obj_t* softwareTab;
extern lv_obj_t* temperatureTab;

void initTabs();

const int ENC_WHEEL = 2.75;

enum alliance {
  blue,
  red
};

extern alliance currentAlliance; 

using namespace okapi;

extern ChassisScales driveScales;
extern ChassisControllerIntegrated discoChassis;
extern AsyncMotionProfileController profileController;

#endif
