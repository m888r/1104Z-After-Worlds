//#include "main.h"
#include "backshot/executor.hpp"
#include "robotIO.hpp"

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
//#define BLUE_DETECTED 1
//#define GREEN_TARGET 2
//#define RED_DETECTED 3

// using namespace okapi;
// auto

// auto

typedef unsigned int WORD;

#define VISION_FOV_WIDTH 316
#define VISION_FOV_HEIGHT 212

int BallValue = 60;

bool isRed = false;

// int desiredRpm = 0;

bool VisionBlue = false;
bool VisionRed = true;
// lv_obj_t *visionColorDisplay;

static lv_style_t redStyle;
static lv_style_t blueStyle;

void setVisionLEDRed() {
  if (!VisionRed) {
    VisionSensor.set_led(COLOR_RED);
    // redStyle.body.main_color = LV_COLOR_HEX(0xFF0000);
    // blueStyle.body.main_color = LV_COLOR_HEX(0x0000FF);
    // visionColorDisplay = lv_obj_create(lv_scr_act(), NULL);
    // lv_obj_set_size(visionColorDisplay, 480, 240);
    // lv_obj_set_style(visionColorDisplay, &redStyle);

    // Brain.Screen.clearScreen();
    // Brain.Screen.drawRectangle(0,0,600,600,color::red);
    // Brain.Screen.render();
    VisionRed = true;
    VisionBlue = false;
  }
}

void setVisionLEDBlue() {
  if (!VisionBlue) {
    VisionSensor.set_led(COLOR_BLUE);
    // redStyle.body.main_color = LV_COLOR_HEX(0xFF0000);
    // blueStyle.body.main_color = LV_COLOR_HEX(0x0000FF);
    //  visionColorDisplay = lv_obj_create(lv_scr_act(), NULL);
    // lv_obj_set_size(visionColorDisplay, 480, 240);
    // lv_obj_set_style(visionColorDisplay, &blueStyle);
    VisionBlue = true;
    VisionRed = false;
  }
}

int motorPower;
int motorPowerRight;
bool alligned = false;
void VisionProcessingTask(void *x) {
  // int objDetected;
  // Vision.objects[0]
  int xerror = 0;
  pros::vision_object_s_t blueRtn;
  pros::vision_object_s_t redRtn;
  double kP = 0.22;
  while (true) {
    if (VisionBlue) {
      blueRtn = VisionSensor.get_by_code(0, BLUE_FLAG);
    } else {
      redRtn = VisionSensor.get_by_code(0, RED_FLAG);
    }

    // Brain.Screen.clearLine;
    // Brain.Screen.printAt(20,20,"%4.2f",Vision.largestObject.width);
    // Controller.Screen.print("%4.2f %4.2f err ",Vision.largestObject.exists,
    // xerror);
    // printf("%d", blueRtn.signature);
    if (VisionSensor.get_object_count() > 0) {
      if (VisionBlue) {
        xerror = blueRtn.x_middle_coord - VISION_FOV_WIDTH / 2;
        // printf("%i",blueRtn.x_middle_coord);
      } else {
        xerror = redRtn.x_middle_coord - VISION_FOV_WIDTH / 2;
      }

      // Brain.Screen.setCursor(0,0);
      // Brain.Screen.clearLine();
      // Brain.Screen.print(xerror);
      // Controller.Screen.clearScreen();
      // Controller.Screen.print(xerror);

      motorPower = xerror * kP;
      motorPowerRight = xerror * -kP;
      // printf("%i",errno);
      if (xerror < 3 && (abs(motorPower)) < 5) {
        alligned = true;
      } else {
        alligned = false;
      }
    }

    pros::delay(20);
  }
}

int desiredRpm;
int buttonPressed;
int buttonToggle;
int ballCounter;
// lv_obj_t * title = lv_label_create(lv_scr_act(), NULL);
// lv_label_set_text(title, "Auton Selction");

bool isHolding = true;

static lv_res_t anglerTest(lv_obj_t *button) {
  printf("I AM IN THE ANGLER FUNCTION");
  if (!isHolding) {  // CapFlipMotor.getBrakeMode() ==
                     // okapi::AbstractMotor::brakeMode::coast;
    isHolding = false;
    CapFlipMotor.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
  } else {
    isHolding = true;
    CapFlipMotor.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
  }
  return LV_RES_OK;
}

void opcontrol() {
  // lv_obj_t *dataPage = lv_label_create(flywheelTab, NULL);
  // lv_label_set_text(dataPage,
  //                   "Flywheel Temp: \n"
  //                   "Efficiency: \n"
  //                   "Torque: \n"
  //                   "Vel: \n");
  // lv_obj_align(dataPage, NULL, LV_ALIGN_IN_TOP_LEFT, 20, 20);
  // lv_label_set_recolor(dataPage, true);

  // lv_obj_t* motorTemps = lv_label_create(temperatureTab, NULL);
  // lv_label_set_text(motorTemps,
  //                   "Right: \n"
  //                   "Left: \n");
  // lv_obj_align(motorTemps, NULL, LV_ALIGN_IN_TOP_LEFT, 20, 20);
  // lv_label_set_recolor(motorTemps, true);

  long currentTime;
  pros::Task VisionProcessing(VisionProcessingTask, NULL);
  pros::Task turnshotTask(backshot::runBackshot, nullptr, TASK_PRIORITY_DEFAULT,
                          TASK_STACK_DEPTH_DEFAULT, "Turn Shots");
  backshot::isTurnShotting = false;
  // profileController.generatePath({Point{0_ft, 0_ft, 0_deg}, Point{4_ft, 4_ft,
  // 0_deg}, Point{5_ft,5_ft,270_deg}},"A");
  // profileController.generatePath({Point{-3_ft, 4_ft, 0_deg}, Point{0_ft,
  // 0_ft, 0_deg}}, "B"); profileController.setTarget("A");
  // profileController.waitUntilSettled();
  // profileController.setTarget("B",true);
  // profileController.waitUntilSettled();
  /// profileController.
  // lv_obj_t * ethanAlien = lv_img_create(lv_scr_act(), NULL);
  // lv_img_set_src(ethanAlien, &EthansMeme);
  // lv_obj_align(ethanAlien, NULL, LV_ALIGN_CENTER, 0, 0);

  okapi::ControllerButton leftUpperTrigger(okapi::ControllerDigital::L1);
  okapi::ControllerButton leftArrow(okapi::ControllerDigital::left);

  okapi::ControllerButton executeTurnShot(okapi::ControllerDigital::L2);
  // lcd::graph::display();

  okapi::ControllerButton toggleArea(okapi::ControllerDigital::right);

  CapFlipMotor.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);

  // lv_obj_t *anglerTestButton = lv_btn_create(flywheelTab, NULL);
  // lv_cont_set_fit(anglerTestButton, true,
  //                 true); /*Enable resizing horizontally and vertically*/
  // lv_obj_align(anglerTestButton, NULL, LV_ALIGN_IN_LEFT_MID, 300, 30);
  // lv_btn_set_action(anglerTestButton, LV_BTN_ACTION_CLICK, anglerTest);
  // // lv_btn_set_toggle(anglerTestButton, true);
  // lv_obj_t *anglerTestLabel = lv_label_create(anglerTestButton, NULL);
  // lv_label_set_text(anglerTestLabel, "Floppy Angler");

  okapi::Motor left1(3);
  okapi::Motor left2(10);
  okapi::Motor right1(-6);
  okapi::Motor right2(-1);

  okapi::Controller partner(okapi::ControllerId::partner);

  while (true) {
    // std::string data =
    //     "Flywheel Temp: " + std::to_string(FlywheelMotor.getTemperature()) +
    //     "\n"
    //     "Efficiency: " +
    //     std::to_string(FlywheelMotor.getEfficiency()) +
    //     "\n"
    //     "Torque: " +
    //     std::to_string(FlywheelMotor.getTorque()) +
    //     "\n"
    //     "Vel: " +
    //     std::to_string(FlywheelMotor.getActualVelocity() * 7) + "\n";
    // lv_label_set_text(dataPage, data.c_str());

    // std::string motorTemp = 
    //     "Right 6: " + std::to_string(right1.getTemperature()) + "\n" + 
    //     "Right 1: " + std::to_string(right2.getTemperature()) + "\n" + 
    //     "Left 3: " + std::to_string(left1.getTemperature()) + "\n" + 
    //     "Left 10: " + std::to_string(left2.getTemperature()) + "\n";

    // lv_label_set_text(motorTemps, motorTemp.c_str());


    if (currentAlliance == red) {
      setVisionLEDRed();
    } else if (currentAlliance == blue) {
      setVisionLEDBlue();
    }

    if (partner.getDigital(okapi::ControllerDigital::up)) {
      shooting::currArea = shooting::front;
    } else if (partner.getDigital(okapi::ControllerDigital::down)) {
      shooting::currArea = shooting::back;
    } else if (partner.getDigital(okapi::ControllerDigital::left)) {
      shooting::currArea = shooting::platform;
    }

     

    // if (controller.get_digital(DIGITAL_RIGHT)) {
    //   if (!buttonPressed) {
    //     buttonToggle = 1 - buttonToggle;
    //     buttonPressed = 1;
    //   }
    // } else {
    //   buttonPressed = 0;
    // }

    // if (buttonToggle) {  // buttonToggle = 1 is shooting at red flags
    //   isRed = false;
    // } else {
    //   isRed = true;
    // }

    if (toggleArea.changedToPressed()) {
      if (shooting::currArea == shooting::front) {
        shooting::currArea = shooting::back;
      } else {
        shooting::currArea = shooting::front;
      }
    }

    if (!backshot::isTurnShotting) {
      if (/*controller.get_digital(DIGITAL_L2)*/ false) {
        discoChassis.rotate((motorPower / 100));
      }

      else {
        if (abs(okapicontroller.getAnalog(okapi::ControllerAnalog::leftY)) >
                0.05 ||
            abs(okapicontroller.getAnalog(okapi::ControllerAnalog::leftX)) >
                0.05) {
          //  if(abs(Controller.Axis4.value()) >= 90.00){
          //   if((double)Controller.Axis4.value() > 0.00){
          //   rDriveV = (((double)Controller.Axis4.value()*1.65)-78.5) * 12.0
          //   /127;
          //   }
          //   else{
          //   rDriveV = (((double)Controller.Axis4.value()*1.65)+78.5) * 12.0
          //   /127;
          //   }
          //  }
          //  else if(abs(Controller.Axis4.value()) >= 40.00 &&
          //  (abs(Controller.Axis4.value())) <90.00) {
          //      if((double)Controller.Axis4.value() > 0.00){
          //      rDriveV = ((double)Controller.Axis4.value()-20) * 12.0 /127;
          //      }
          //      else{
          //          rDriveV = ((double)Controller.Axis4.value()+20) * 12.0
          //          /127;
          //      }
          //      //Controller.Screen.print("%d",abs(Controller.Axis4.value()));
          //  }
          //  else{
          //       rDriveV = ((double)Controller.Axis4.value()*0.5) * 12.0 /127;
          //  }
          double xAxisValue =
              okapicontroller.getAnalog(okapi::ControllerAnalog::leftX) * 127.0;
          double turnPower = xAxisValue;
          if (fabs(xAxisValue) >= 90.00) {
            if (xAxisValue > 0.00) {
              turnPower = ((xAxisValue * 1.65) - 78.5) * 12.0 / 127.0;
            } else {
              turnPower = ((xAxisValue * 1.65) + 78.5) * 12.0 / 127.0;
            }
          } else if (fabs(xAxisValue) >= 40.0 && (fabs(xAxisValue) < 90.0)) {
            if (xAxisValue > 0.0) {
              turnPower = (xAxisValue - 20) * 12.0 / 127.0;
            } else {
              turnPower = (xAxisValue + 20) * 12.0 / 127.0;
            }
          } else {
            turnPower = (xAxisValue * 0.5) * 12.0 / 127.0;
          }

          discoChassis.arcade(
              okapicontroller.getAnalog(okapi::ControllerAnalog::leftY),
              /*okapicontroller.getAnalog(okapi::ControllerAnalog::leftX)*/
                  turnPower / 12.0);
        } else {
          discoChassis.stop();
        }
      }
    }

    if (!backshot::isTurnShotting) {
      IntakeMotor.moveVoltage(
          (okapicontroller.getDigital(okapi::ControllerDigital::R1) -
           okapicontroller.getDigital(okapi::ControllerDigital::R2)) *
          12000);
    }

    if (!backshot::isTurnShotting) {
      shooting::update(&leftUpperTrigger, &leftArrow);
    }

    //shooting::run();

    {
      using namespace subsystems;

      // angler::run(); ran as task in initialize()
      flywheel::run();

      okapi::ControllerButton btnNormal(okapi::ControllerDigital::up);
      okapi::ControllerButton btnStop(okapi::ControllerDigital::down);

      if (btnNormal.changedToPressed()) {
        flywheel::changeSpeed(flywheel::normal);
      }
      if (btnStop.changedToPressed()) {
        flywheel::changeSpeed(flywheel::stop);
      }
    }

    // FOR ANGLER ANGLE TESTING
    // okapi::ControllerButton
    // btnAnglerBrakemode(okapi::ControllerDigital::right); if
    // (btnAnglerBrakemode.changedToReleased()) { if
    // (CapFlipMotor.getBrakeMode() ==
    //     okapi::AbstractMotor::brakeMode::coast) {
    //   CapFlipMotor.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
    // } else {
    //   CapFlipMotor.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
    // }
    // }

    // if (backshot::record()) {
    //   // if (pros::millis() /*and add starttime*/ % 50 == 0)
    //   controller.rumble(".");
    //   backshot::print();
    // }
    // backshot::print();

    if (backshot::recordButtons()) {
      controller.rumble(".");
      backshot::print();
    }
    backshot::setCombos();

    if (executeTurnShot.changedToPressed()) {
      backshot::startTurnShots();
    } else if (!executeTurnShot.isPressed()) {
      backshot::isTurnShotting = false;
      backshot::changeState(backshot::nothing);
    }
    //lcd::graph::addData(FlywheelMotor.getActualVelocity() * 7);
    //lcd::graph::display();

    // if (controller.get_digital(DIGITAL_L1))
    // {
    //   IndexerMotor.move_velocity(200);
    //   if (desiredRpm == 395)
    //   { //was 525
    //     //FwVelocitySet(&flywheel, desiredRpm-5, 0.40);
    //     //currentTime = pros::millis();
    //     //lv_label_set_text(label,string(currentTime));
    //     IndexerMotor.move_velocity(200);
    //     // while(currentTime < (pros::millis() - 100)){
    //     //pros::delay(90);
    //     CapFlipMotor.move_absolute(-55, 25);
    //   }
    //   // IndexerMotor.spin(directionType::fwd,100,velocityUnits::pct);
    // }
    // else if (controller.get_digital(DIGITAL_LEFT))
    // {
    //   IndexerMotor.move_velocity(-200);
    //   // IndexerMotor.spin(directionType::rev,100,velocityUnits::pct);
    // }
    // else
    // {
    //   IndexerMotor.move_voltage(0);
    //   CapFlipMotor.move_absolute(0, 200);
    //   //IndexerMotor.stop(brakeType::coast);
    // }

    // if(abs(controller.get_analog(ANALOG_RIGHT_Y))>10){
    // CapFlipMotor.move_velocity(controller.get_analog(ANALOG_RIGHT_Y));
    //}
    // else{
    // CapFlipMotor.move_velocity(0);
    //}

    pros::Task::delay(10);
  }
}
