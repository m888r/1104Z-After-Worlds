#include "subsystems/intake.hpp"
#include "robotIO.hpp"

namespace subsystems {
  namespace intake {
    intakeState currState = stopped;

    void changeState(intakeState state) {
      currState = state;
    }

    void run(void* p) {
      while (true) {
        if (!pros::competition::is_autonomous() || pros::competition::is_disabled()) {
          pros::delay(20);
          continue;
        }

        switch (currState) {
          case stopped:
            IntakeMotor.moveVoltage(0);
            break;
          case forceIntaking:
            IntakeMotor.moveVoltage(12000);
            break;
          case intaking:
            if (BallDetectOkapi.isPressed()) {
              IntakeMotor.moveVoltage(0);
            } else {
              IntakeMotor.moveVoltage(12000);
            }
            break;
          case outtaking:
            IntakeMotor.moveVoltage(-12000);
            break;
          case released:
            break;
        }

        pros::delay(20);
      }
    }
  }
}