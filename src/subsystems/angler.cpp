#include "subsystems/angler.hpp"

namespace subsystems {

namespace angler {
anglerState currState = frontTop;
int position = 0;

void changeState(anglerState state) {
  currState = state;
  switch (currState) {
    case frontTop:
      position = 0;
      break;
    case frontMid:
      position = 52;  // was 50
      break;
    case platformMid:  // on middle platform
      position = 80;
      break;
    case platformTop:
      position = 60;
      break;
    case sideMid:  // behind side platforms side poles
      position = 60;
      break;
    case sideTop:
      position = 0;
      break;
    case midTop:  // behind side platforms mid pole
      position = 0;
      break;
    case midMid:
      position = 60;
      break;
    case backTop:  // behind middle platform
      position = 0;
      break;
    case backMid:
      position = 20;
      break;
    case turnshotTop:
      position = 10;
      break;
    case turnshotMid:
      position = 54;
      break;
  }
}

void changePosition(int pos) {
  position = pos;
}

void run(void* p) {  // remembr to replace a lot with the trig if the shot is
                     // linear enough
  changeState(currState);
  while (true) {
    CapFlipMotor.moveAbsolute(position, 200);
    pros::delay(20);
  }
}
}  // namespace angler
}  // namespace subsystems