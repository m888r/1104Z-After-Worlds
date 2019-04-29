#include "motioncontrol.hpp"
#include "lib/motion/motionProfileFollower.hpp"
#include "lib/motion/sCurveProfile.hpp"
#include "lib/motion/trapezoidalProfile.hpp"
#include "lib/structs.hpp"

namespace motioncontrol
{

using namespace mlib;

okapi::MotorGroup leftDrive({3, 10});
okapi::MotorGroup rightDrive({-6, -1});

double rEncLast;
double lEncLast;

const double discoChassisWIDTH = 4.5 * (85.0 / 90.0) * (90.0 / 97.0); // was 6.42
const double TICKSINCH = ENC_WHEEL * PI / 360.0;
const okapi::QLength TRACKING_CENTER_OFFSET = 5.5_in;

pros::Mutex angleMutex;

QLength currX;
QLength currY;
QAngle currAngle;

bool appOn;

void init()
{
  rightEnc.reset();
  leftEnc.reset();
  currX = 0_ft;
  currY = 0_ft;
  currAngle = 0_deg;

  rEncLast = rightEnc.get() * TICKSINCH;
  lEncLast = leftEnc.get() * TICKSINCH;

  appOn = false;
}

/**
 * Iterate
 */
void calculate()
{
  using namespace okapi;

  double dX = 0.0;
  double dY = 0.0;
  double dTheta = 0.0;

  double rCurrEnc = rightEnc.get() * TICKSINCH;
  double lCurrEnc = leftEnc.get() * TICKSINCH;

  double rDEnc = rCurrEnc - rEncLast;
  double lDEnc = lCurrEnc - lEncLast;

  double dCenterArc = (rDEnc + lDEnc) / 2.0;
  // dCenterArc *= TICKSINCH;

  dTheta = (lDEnc - rDEnc) / discoChassisWIDTH /** PI / 180.0*/;

  double radius = (dTheta == 0) ? 0 : dCenterArc / dTheta;
  dX = dTheta == 0 ? 0 : (radius - radius * cos(dTheta));
  dY = dTheta == 0 ? dCenterArc : radius * sin(dTheta);

  // offset tracking center
  //dX += TRACKING_CENTER_OFFSET.convert(okapi::inch) * cos(dTheta);
  //dY += TRACKING_CENTER_OFFSET.convert(okapi::inch) * sin(dTheta);

  angleMutex.take(20);
  currX = (dX * cos(currAngle.convert(radian)) +
           dY * sin(currAngle.convert(radian)) + currX.convert(inch)) *
          inch;
  currY = (dY * cos(currAngle.convert(radian)) -
           dX * sin(currAngle.convert(radian)) + currY.convert(inch)) *
          inch;
  angleMutex.give();

  QAngle tempCurrAngle =
      ((dTheta * 180.0 / PI) + currAngle.convert(degree)) * degree;

  rEncLast = rCurrEnc;
  lEncLast = lCurrEnc;

  while (tempCurrAngle.convert(degree) >= 360.0)
  {
    tempCurrAngle = (tempCurrAngle.convert(degree) - 360.0) * degree;
  }
  while (tempCurrAngle.convert(degree) < 0.0)
  {
    tempCurrAngle = (tempCurrAngle.convert(degree) + 360.0) * degree;
  }

  angleMutex.take(20);
  currAngle = tempCurrAngle;
  angleMutex.give();

  //printf("Curr Angle: %1.2f, temp curr angle: %1.2f\n", currAngle.convert(degree), tempCurrAngle.convert(degree));
}

QLength distanceToPoint(QLength x, QLength y)
{
  return (sqrt(pow((currX.convert(inch) - x.convert(inch)), 2) +
               pow((currY.convert(inch) - y.convert(inch)), 2))) *
         inch;
}

QAngle angleToPoint(QLength x, QLength y) {}

std::tuple<QLength, QAngle> distanceAndAngleToPoint(QLength x, QLength y)
{
  return std::tuple<QLength, QAngle>(distanceToPoint(x, y), angleToPoint(x, y));
}

void printPosition(void *p)
{
  pros::Controller controller(pros::E_CONTROLLER_MASTER);
  controller.clear();
  int count = 0;
  int startTime = 0;
  using namespace okapi;

  while (true)
  {
    double x = currX.convert(okapi::inch);
    double y = currY.convert(okapi::inch);
    int left = leftEnc.get();
    int right = rightEnc.get();
    double angle = currAngle.convert(degree);
    //controller.print(0, 0, "X: %.2f, Y: %1.2f", x, y);

    //controller.print(0, 0, "X: %.2f, Y: %1.2f", lY, lX);
    controller.print(0, 0, "L: %d", left);
    if (pros::competition::is_autonomous() && count < 1 &&
        !pros::competition::is_disabled())
    {
      count++;
      startTime = pros::millis();
    }
    else if (pros::competition::is_autonomous() &&
             !pros::competition::is_disabled())
    {
      double deltaT = ((double)(pros::millis() - startTime)) / 1000.0;
      // controller.print(0, 0, "Time: %1.2f", deltaT);
    }
     pros::delay(51);
    //controller.print(1, 0, "Y: %.2f", y);
    controller.print(1, 0, "R: %d", right);
    pros::delay(51);
    controller.print(2, 0, "A: %1.2f", angle);
    // controller.print(2, 0, "app: %d", (appOn) ? 1 : 0);
    pros::delay(51);
    //controller.print(1, 0, "Batt: %1.2f%%", pros::battery::get_capacity());
    pros::delay(51);
  }
}

void run(void *p)
{
  pros::Task odometryPrint(printPosition, nullptr, TASK_PRIORITY_DEFAULT,
                           TASK_STACK_DEPTH_DEFAULT,
                           "Position Print --> Controller");
  // lv_obj_t *position = lv_label_create(softwareTab, NULL);
  // lv_label_set_text(position,
  //                   "Right: \n"
  //                   "Left: \n"
  //                   "Position: \n"
  //                   "Angle: \n");
  // lv_obj_align(position, softwareTab, LV_ALIGN_IN_TOP_LEFT, 20, 20);
  while (true)
  {
    calculate();
    // std::string posStr = "Right: " + std::to_string(rightEnc.get()) + "\n" +
    //                      "Left: " + std::to_string(leftEnc.get()) + "\n" +
    //                      "Position: " + std::to_string(currX.convert(inch)) +
    //                      ", " + std::to_string(currY.convert(inch)) + "\n" +
    //                      "Angle: " + std::to_string(currAngle.convert(degree)) +
    //                      "\n";
    // lv_label_set_text(position, posStr.c_str());
    pros::delay(5);
  }
}

void turnAbsolute(QAngle target, QTime timeout)
{
  // while (true) {
  //   angleMutex.take(TIMEOUT_MAX);
  //   QAngle tCurrAngle = currAngle;
  //   angleMutex.give();
  //   printf("Turning, Current angle is: %1.2f\n", currAngle.convert(degree));
  //   printf("Turning, Temporarily set Current angle is: %1.2f\n", tCurrAngle.convert(degree));
  //   printf("Current target angle is: %1.2f\n", target.convert(degree));
  //   double angleError =
  //       target.convert(okapi::radian) - currAngle.convert(okapi::radian);
  //   angleError = atan2(sin(angleError), cos(angleError));

  //   // pros::Controller controller(pros::E_CONTROLLER_MASTER);

  //   // pros::delay(51);
  //   // controller.print(1, 0, "AE: %1.2f", angleError * 180.0/PI);
  //   // pros::delay(51);
  //   // controller.print(2, 0, "A: %1.2f", currAngle.convert(degree));
  //   // pros::delay(51);
  //   QAngle targetAng = (angleError * 180.0 / PI) * degree;
  //   // controller.print(0, 0, "AE2: %1.2f", targetAng.convert(degree));
  //   printf("Turning %1.2f degrees from current position\n", targetAng.convert(degree));

  //   discoChassis.turnAngleAsync(-targetAng);

  //   pros::delay(200);
  // }

  // okapi::IterativePosPIDController tc =
  //     okapi::IterativeControllerFactory::posPID(1.1, 0.066, 0.03, 0, // kp kd ki bias
  //                                               std::make_unique<AverageFilter<3>>());

  okapi::Timer timer;

  okapi::IterativePosPIDController tc =
      okapi::IterativeControllerFactory::posPID(1.3, 0.025, 0.035, 0, // kp ki kd bias //was 1.1
                                                std::make_unique<AverageFilter<3>>());

  double normalizedTarget = atan2(sin(target.convert(okapi::radian)), cos(target.convert(okapi::radian)));
  double normalizedCurrAngle = atan2(sin(currAngle.convert(okapi::radian)), cos(currAngle.convert(okapi::radian)));
  double angleError = target.convert(okapi::radian) -
                      currAngle.convert(okapi::radian);
  angleError = atan2(sin(angleError),
                     cos(angleError));

  if (angleError * 180.0 / 1_pi <= 25)
  {
    tc.setGains(1.4, 0.8, 0.05, 0); // kp ki kd bias

  } else if (angleError * 180.0 / 1_pi <= 40) {
    tc.setGains(1.1, 0.066, 0.03, 0); // kp ki kd bias
  }
  else if (angleError * 180.0 / 1_pi <= 60)
  {
    tc.setGains(1.13, 0.066, 0.04, 0); // kp ki kd bias
  }

  tc.setTarget(0);
  okapi::SettledUtil su =
      okapi::SettledUtilFactory::create(2, 1, 10_ms); // target Error, target derivative, settle time 
  bool disabled = false;

  timer.placeMark();

  while (!su.isSettled(angleError * 180.0 / PI))
  {
    if (timer.getDtFromMark() >= timeout) {
      break;
    }
    angleError = target.convert(okapi::radian) -
                 currAngle.convert(okapi::radian);
    angleError = atan2(sin(angleError),
                       cos(angleError));
    tc.setTarget(0);
    double power = tc.step(-angleError);
    printf("Angle Error: %1.2f, ", angleError * 180.0 / 1_pi);
    printf("Power: %1.2f\n", power);

    //okapi::Motor driveMotor(1);
    // if (abs(driveMotor.getActualVelocity()) < 10 && abs(angleError * 180.0 / PI) < 0.4)
    // {
    //   disabled = true;
    //   //discoChassis.rotate(power);
    // }
    // else
    // {
    //   discoChassis.arcade(0, power);
    // }
    discoChassis.arcade(0, power);
    pros::delay(10);
    if (disabled)
    {
      break;
    }
  }
  discoChassis.stop();
}

void turnRelative(QAngle target, QTime timeout)
{
  turnAbsolute(
      (currAngle.convert(okapi::degree) + target.convert(okapi::degree)) *
      okapi::degree, timeout);
}

void turnAbsoluteIntegrated(QAngle angle) {
    QAngle tCurrAngle = odometry.getPose().heading;

    printf("Turning, Current angle is: %1.2f\n", currAngle.convert(degree));
    printf("Turning, Temporarily set Current angle is: %1.2f\n", tCurrAngle.convert(degree));
    printf("Current target angle is: %1.2f\n", angle.convert(degree));
    tCurrAngle = atan2(sin(tCurrAngle.convert(radian)), cos(tCurrAngle.convert(radian))) * radian;
    QAngle normalizedAngle = atan2(sin(angle.convert(radian)), cos(angle.convert(radian))) * radian;
    double angleError =
        normalizedAngle.convert(okapi::radian) - tCurrAngle.convert(okapi::radian);
    angleError = atan2(sin(angleError), cos(angleError));

    // pros::Controller controller(pros::E_CONTROLLER_MASTER);

    // pros::delay(51);
    // controller.print(1, 0, "AE: %1.2f", angleError * 180.0/PI);
    // pros::delay(51);
    // controller.print(2, 0, "A: %1.2f", currAngle.convert(degree));
    // pros::delay(51);
    QAngle targetAng = (angleError * 180.0 / PI) * degree;
    // controller.print(0, 0, "AE2: %1.2f", targetAng.convert(degree));
    printf("Turning %1.2f degrees from current position\n", targetAng.convert(degree));

    discoChassis.turnAngleAsync(targetAng);
    discoChassis.waitUntilSettled();
}

// KinematicConstraints turnConstraints;
// TrapezoidalProfile profile;
// QAngle startAngle;

// void initStepTurnProfile(QAngle angle) {

// }
// return true if settled (for now this doesn't work since need to initialize the stuff instead)
bool stepTurnProfile(int dT, QAngle angle)
{

  int direction = (angle.convert(radian) > 0) ? 1 : -1;
  KinematicConstraints turnConstraints(1.0, 0.0116);
  okapi::AbstractMotor::GearsetRatioPair driveGearset =
      AbstractMotor::gearset::green;
  auto profileDisplacement = direction * angle.convert(radian) *
                             driveScales.wheelbaseWidth.convert(meter) /
                             driveScales.wheelDiameter.convert(meter) /
                             (driveGearset.ratio * rpm).convert(radps);

  auto profile =
      TrapezoidalProfile(turnConstraints, profileDisplacement * 10.0);
  MotionProfileFollower follower(0.03, 0.2, 0.0, 1.0); // kP, kD, kA, kV

  QAngle startAngle = currAngle;

  //for (double i = 0; i < profile.getTotalTime(); i++) {
  pros::Motor mingmong(1);

  auto output = profile.calculate(dT);
  printf("Output Pos: %1.2f, Vel: %1.2f, Accel: %1.5f, ", output.position * 0.20024474357, output.velocity, output.accel);
  double currHeading = currAngle.convert(radian);
  double startHeading = startAngle.convert(radian);
  double difference = currHeading - startHeading;
  difference = atan2(sin(difference), cos(difference));
  difference = fabs(difference);
  printf("Angle Error: %1.2f", (output.position * 0.20024474357) - difference * 180.0 / 1_pi);
  printf(", Angle relative to start: %1.2f", difference * 180.0 / 1_pi);

  double power = follower.follow(output, difference * 180.0 / 1_pi * 4.99388888889);
  //discoChassis.rotate(power * direction);
  printf(", Power: %1.2f", power);
  double radpsVel = mingmong.get_actual_velocity() * direction * driveScales.wheelDiameter.convert(meter) / 60.0 / driveScales.wheelbaseWidth.convert(meter);
  printf(", Actual Vel: %1.2f\n", radpsVel);
  // if (output.velocity <= 0.001 && output.position > 0) {
  //   break;
  // }
  bool isSettled = abs(mingmong.get_actual_velocity()) < 10 && abs(angle.convert(degree) - (difference * 180.0 / 1_pi)) < 0.5;

  if (isSettled)
  {
    discoChassis.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
    discoChassis.stop();
  }
  else
  {
    discoChassis.arcade(0, power * direction);
  }
  return isSettled;
}

// return true if settled
bool stepTurnProfileAbsolute(int dT, QAngle angle)
{
  QAngle difference = angle - currAngle;
  double normalized = atan2(sin(difference.convert(radian)), cos(difference.convert(radian)));

  return stepTurnProfile(dT, normalized * radian);
}

void turnProfileTestAbsolute(QAngle angle)
{
  turnProfileController.setTargetAbsolute(angle, currAngle);

  while (!turnProfileController.isSettled())
  {
    discoChassis.arcade(0, turnProfileController.step(currAngle));
    pros::delay(5);
  }

  discoChassis.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
  discoChassis.stop();
}

void turnProfile(QAngle angle)
{ // todo make absolute
  okapi::QLength actualChassisWidth = 11.4_in;

  int direction = (angle.convert(radian) > 0) ? 1 : -1;
  KinematicConstraints turnConstraints(1.0, 0.00386);
  // 1.0 0.0116 god gains
  // 1.0 0.00386 was good
  // wip gains 1.4 0.055
  okapi::AbstractMotor::GearsetRatioPair driveGearset =
      AbstractMotor::gearset::green;
  auto profileDisplacement = direction * angle.convert(radian) *
                             driveScales.wheelbaseWidth.convert(meter) /
                             driveScales.wheelDiameter.convert(meter) /
                             (driveGearset.ratio * rpm).convert(radps);

  auto profile =
      TrapezoidalProfile(turnConstraints, profileDisplacement * 10.0);
  MotionProfileFollower follower(0.01, 0.2, 13.0, 0.6); // kP, kD, kA, kV
  // godus/good gains 0.01 0.2 13.0 0.6
  // god gains 0.03 0.2 0.0 1.0
  // wip gains 0.007 0.2 22.0 0.6
  QAngle startAngle = currAngle;

  //for (double i = 0; i < profile.getTotalTime(); i++) {
  okapi::Timer timer;
  pros::Motor mingmong(1);
  while (true)
  {
    int dT = timer.getDtFromStart().convert(millisecond);
    if (dT > profile.getTotalTime())
      dT = profile.getTotalTime();
    auto output = profile.calculate(dT);
    printf("%dms: ", dT);
    printf("Output Pos: %1.2f, Vel: %1.2f, Accel: %1.5f, ", output.position * 0.20024474357, output.velocity, output.accel);
    double currHeading = currAngle.convert(radian);
    double startHeading = startAngle.convert(radian);
    double difference = currHeading - startHeading;
    difference = atan2(sin(difference), cos(difference));
    difference = fabs(difference);
    printf("Angle Error: %1.2f", (output.position * 0.20024474357) - difference * 180.0 / 1_pi);
    printf(", Angle relative to start: %1.2f", difference * 180.0 / 1_pi);

    double power = follower.follow(output, difference * 180.0 / 1_pi * 4.99388888889);
    double kSF = (0.18 * direction * ((output.accel >= 0) ? 1 : -1));
    if (output.accel == 0)
    {
      kSF = 0;
    }
    kSF = 0;
    power = power * direction + kSF;
    discoChassis.arcade(0, power);
    //discoChassis.rotate(power * direction);
    printf(", Power: %1.2f", power);

    double radpsVel = mingmong.get_actual_velocity() * direction * driveScales.wheelDiameter.convert(meter) / 60.0 / driveScales.wheelbaseWidth.convert(meter);
    printf(", Actual Vel: %1.2f\n", mingmong.get_actual_velocity() / 200.0);
    pros::delay(5);
    // if (output.velocity <= 0.001 && output.position > 0) {
    //   break;
    // }
    double angleError = angle.convert(radian) - difference;
    angleError = atan2(sin(angleError), cos(angleError));
    if (mingmong.get_actual_velocity() * direction < 2 && abs(angleError * 180.0 / 1_pi) < 0.75)
    {
      break;
    }
  }
  printf("Current Angle after break: %1.2f\n", currAngle.convert(degree));

  discoChassis.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
  discoChassis.stop();
}

void turnProfileAbsolute(QAngle angle)
{
  QAngle difference = angle - currAngle;
  double normalized = atan2(sin(difference.convert(radian)), cos(difference.convert(radian)));
  turnProfile(-normalized * radian);
}

void driveApp() { appOn = true; }

void waitUntilSettled()
{
  while (!appController.isSettled())
  {
    pros::delay(40);
  }
  appOn = false;
  discoChassis.stop();
}

void waitUntilSettled(int timeout)
{
  std::uint32_t startTime = pros::millis();
  while (!appController.isSettled() ||
         (pros::millis() - startTime) <= timeout)
  {
    pros::delay(40);
  }
  appOn = false;
  discoChassis.stop();
}

void runApp(void *p)
{
  while (true)
  {
    if (appOn && pros::competition::is_autonomous())
    {
      appController.loop();
    }
    pros::delay(10);
  }
}
} // namespace motioncontrol