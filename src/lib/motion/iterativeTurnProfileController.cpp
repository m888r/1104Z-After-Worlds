#include "lib/motion/iterativeTurnProfileController.hpp"
#include "robotIO.hpp"

namespace mlib {

IterativeTurnProfileController::IterativeTurnProfileController(
    KinematicConstraints turnConstraints, MotionProfileFollower profileFollower)
    : constraints(turnConstraints),
      follower(profileFollower),
      profile(turnConstraints, 0),
      direction(1),
      currTarget(0_deg),
      startAngle(0_deg) {}

// Needs to be immediately followed by stepping or bad things will happen
void IterativeTurnProfileController::setTargetRelative(okapi::QAngle target,
                                                       okapi::QAngle currAngle) {
  settled = false;
  currTarget = target;
  direction = (target.convert(degree) >= 0) ? 1 : -1;

  startAngle = currAngle;
  okapi::AbstractMotor::GearsetRatioPair driveGearset =
      okapi::AbstractMotor::gearset::green;
  timer.placeMark();

  auto profileDisplacement = direction * target.convert(radian) *
                             driveScales.wheelbaseWidth.convert(meter) /
                             driveScales.wheelDiameter.convert(meter) /
                             (driveGearset.ratio * okapi::rpm).convert(radps);

  profile = TrapezoidalProfile(constraints, profileDisplacement * 10.0);
  printf("Turning, target: %1.2f, startAngle: %1.2f, currAngle: %1.2f\n", target.convert(degree), startAngle.convert(degree), currAngle.convert(degree));
}

// Needs to be immediately followed by stepping or bad things will happen
void IterativeTurnProfileController::setTargetAbsolute(okapi::QAngle target,
                                                       okapi::QAngle currAngle) {
  QAngle difference = target - currAngle;
  double normalized = atan2(sin(difference.convert(okapi::radian)),
                            cos(difference.convert(okapi::radian)));
  setTargetRelative(-normalized * radian, currAngle);
}

double IterativeTurnProfileController::step(okapi::QAngle currAngle) {
  int dT = timer.getDtFromMark().convert(millisecond);
  if (dT > profile.getTotalTime()) dT = profile.getTotalTime();
  auto output = profile.calculate(dT);
  printf("%dms: ", dT);
  printf("Output Pos: %1.2f, Vel: %1.2f, Accel: %1.5f, ",
         output.position * 0.20024474357, output.velocity, output.accel);
  double currHeading = currAngle.convert(radian);
  double startHeading = startAngle.convert(radian);
  double difference = currHeading - startHeading;
  difference = atan2(sin(difference), cos(difference));
  difference = fabs(difference);
  //printf("currAngle: %1.2f, ", currHeading * 180.0 / 1_pi);
  //printf("startAngle: %1.2f, ", startHeading * 180.0 / 1_pi);
  printf("Angle Error: %1.2f",
         (output.position * 0.20024474357) - difference * 180.0 / 1_pi);
  printf(", Angle relative to start: %1.2f", difference * 180.0 / 1_pi);

  double power =
      follower.follow(output, difference * 180.0 / 1_pi * 4.99388888889);
  // discoChassis.arcade(0, power * direction);
  // discoChassis.rotate(power * direction);
  printf(", Power: %1.2f", power);

  pros::Motor mingmong(1);
  double radpsVel = mingmong.get_actual_velocity() * direction *
                    driveScales.wheelDiameter.convert(meter) / 60.0 /
                    driveScales.wheelbaseWidth.convert(meter);
  printf(", Actual Vel: %1.2f, ", radpsVel);
  pros::delay(5);
  // if (output.velocity <= 0.001 && output.position > 0) {
  //   break;
  // }
  double angleError = currTarget.convert(radian) - difference;
  angleError = atan2(sin(angleError), cos(angleError));
  printf("Total Error: %1.2f\n", angleError * 180.0 / 1_pi);
  settled = mingmong.get_actual_velocity() * direction < 2 &&
            fabs(angleError * 180.0 / 1_pi) < 1;

  // double kSF =  (0.18 * direction * ((output.accel >= 0) ? 1 : -1));
  // if (output.accel == 0) {
  //   kSF = 0;
  // }


  return power * direction;
}

bool IterativeTurnProfileController::isSettled() { return settled; }
}  // namespace mlib