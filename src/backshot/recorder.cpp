#include "backshot/recorder.hpp"

namespace backshot {

// backshots backshotRecord[2] = {};
std::tuple<flag, pole> comboRecord[2] = {
    std::make_tuple(flag::nothing, pole::nothing),
    std::make_tuple(flag::nothing, pole::nothing)};

std::tuple<flag, pole> currentCombo = {flag::nothing, pole::nothing};

std::vector<button> buttonsPressed = {};

platform currPlatform = platform::nothing;

int currentPosition = 0;
bool canRecord = true;
uint32_t startTime = 0;

okapi::ControllerButton btnA(okapi::ControllerDigital::A);
okapi::ControllerButton btnB(okapi::ControllerDigital::B);
okapi::ControllerButton btnX(okapi::ControllerDigital::X);
okapi::ControllerButton btnY(okapi::ControllerDigital::Y);

bool record() {
  if (!canRecord) {
    if (pros::millis() - startTime < 200) {
      printf("You aren't allowed to do anything yet.\n");
      return true;
    } else {
      printf("You're allowed now.\n");
      canRecord = true;
    }
  }

  auto currPole = std::get<1>(currentCombo);
  auto currFlag = std::get<0>(currentCombo);

  // if a pole hasn't been selected currently
  if (std::get<1>(currentCombo) == pole::nothing) {
    switch (std::get<1>(comboRecord[0])) {
      case pole::right:
        if (btnY.changedToPressed()) {
          currPole = pole::middle;
        }
        if (btnA.changedToPressed()) {
          currPole = pole::right;
        }
        break;
      case pole::left:
        if (btnY.changedToPressed()) {
          currPole = pole::left;
        }
        if (btnA.changedToPressed()) {
          currPole = pole::middle;
        }
        break;
      case pole::middle:
        // should never happen
        printf("FIRST SLOT POLE IS MIDDLE AND THAT IS BAD\n");
        break;
      case pole::nothing:
        if (btnY.changedToPressed()) {
          currPole = pole::left;
        }
        if (btnA.changedToPressed()) {
          currPole = pole::right;
        }
        break;
    }
  }

  // if a flag hasn't been selected
  if (std::get<0>(currentCombo) == flag::nothing) {
    if (btnX.changedToPressed()) {
      currFlag = flag::high;
      // std::string printStr = "Selected Flag " + getFlagString(currFlag) +
      // "\n"; printf(printStr.c_str()); currentCombo =
      // std::make_tuple(currFlag, currPole); print();
    }
    if (btnB.changedToPressed()) {
      currFlag = flag::middle;
      // std::string printStr = "Selected Flag " + getFlagString(currFlag) +
      // "\n"; printf(printStr.c_str()); currentCombo =
      // std::make_tuple(currFlag, currPole); print();
    }
  }

  currentCombo = std::make_tuple(currFlag, currPole);

  // if they're both not nothing, you can then add it to the whole
  if (currFlag != flag::nothing && currPole != pole::nothing) {
    // if there is something in the first slot
    if (currentPosition > 1) {
      comboRecord[0] = std::make_tuple(flag::nothing, pole::nothing);
      comboRecord[1] = std::make_tuple(flag::nothing, pole::nothing);
      currentCombo = std::make_tuple(flag::nothing, pole::nothing);
      canRecord = false;
      currentPosition = 0;
      startTime = pros::millis();
      return true;
    } else {
      // add the boi
      comboRecord[currentPosition] = std::make_tuple(currFlag, currPole);
      currFlag = flag::nothing;
      currPole = pole::nothing;
      currentCombo = std::make_tuple(flag::nothing, pole::nothing);
      // printf("ADDED COMBO TO THE MAIN THING: ");
      // print();
      currentPosition++;
    }
  }

  // print();

  return false;
}

std::string getButtonString(button button) {
  switch(button) {
    case button::up:
      return "u";
      break;
    case button::down:
      return "d";
      break;
    case button::left:
      return "l";
      break;
    case button::right:
      return "r";
      break;
  }
}

okapi::Timer buttonRecordTimer;
bool hasReset = false;
bool recordButtons() {
  if (buttonRecordTimer.getDtFromMark().convert(okapi::millisecond) < 200 && hasReset) {
    return true;
  }
  hasReset = false;
  if (btnX.changedToPressed()) {
    buttonsPressed.push_back(button::up);
  } else if (btnB.changedToPressed()) {
    buttonsPressed.push_back(button::down);
  } else if (btnA.changedToPressed()) {
    buttonsPressed.push_back(button::right);
  } else if (btnY.changedToPressed()) {
    buttonsPressed.push_back(button::left);
  }
  
  bool reset = buttonsPressed.size() > 3;
  if (reset) {
    buttonRecordTimer.placeMark();
    buttonsPressed.clear();
    hasReset = true;
  }
  return reset;
}

std::string getComboString(std::vector<button> buttons) {
  std::string returnString = "";
  for (auto button : buttons) {
    returnString += getButtonString(button);
  }
  return returnString;
}

void setCombos() {
  std::string currentComboString = getComboString(buttonsPressed);

  if (currentComboString == "ll" || currentComboString == "llu" || currentComboString == "lld" || currentComboString == "lll" || currentComboString == "llr") {
    comboRecord[0] = std::make_tuple(flag::high, pole::left);
    comboRecord[1] = std::make_tuple(flag::middle, pole::left);
    currPlatform = platform::left;
  }
  else if (currentComboString == "rr" || currentComboString == "rru" || currentComboString == "rrd" || currentComboString == "rrl" || currentComboString == "rrr") {
    comboRecord[0] = std::make_tuple(flag::high, pole::right);
    comboRecord[1] = std::make_tuple(flag::middle, pole::right);
    currPlatform = platform::right;
  }
  else if(currentComboString == "lud") {
    comboRecord[0] = std::make_tuple(flag::high, pole::left);
    comboRecord[1] = std::make_tuple(flag::middle, pole::middle);
    currPlatform = platform::left;
  }
  else if(currentComboString == "luu") {
    comboRecord[0] = std::make_tuple(flag::high, pole::left);
    comboRecord[1] = std::make_tuple(flag::high, pole::middle);
    currPlatform = platform::left;
  }
  else if(currentComboString == "ldu") {
    comboRecord[0] = std::make_tuple(flag::middle, pole::left);
    comboRecord[1] = std::make_tuple(flag::high, pole::middle);
    currPlatform = platform::left;
  }
  else if(currentComboString == "rud") {
    comboRecord[0] = std::make_tuple(flag::high, pole::right);
    comboRecord[1] = std::make_tuple(flag::middle, pole::middle);
    currPlatform = platform::right;
  }
  else if(currentComboString == "ruu") {
    comboRecord[0] = std::make_tuple(flag::high, pole::right);
    comboRecord[1] = std::make_tuple(flag::high, pole::middle);
    currPlatform = platform::right;
  }
  else if(currentComboString == "rdu"){
    comboRecord[0] = std::make_tuple(flag::middle, pole::right);
    comboRecord[1] = std::make_tuple(flag::high, pole::middle);
    currPlatform = platform::right;
  }
  else if(currentComboString == "lur"){
    comboRecord[0] = std::make_tuple(flag::high, pole::left);
    comboRecord[1] = std::make_tuple(flag::high, pole::right);
    currPlatform = platform::left;
  }
  else if(currentComboString == "ldd") {
    comboRecord[0] = std::make_tuple(flag::middle, pole::left);
    comboRecord[1] = std::make_tuple(flag::middle, pole::middle);
    currPlatform = platform::left;
  }
  else if(currentComboString == "rdd") {
    comboRecord[0] = std::make_tuple(flag::middle, pole::right);
    comboRecord[1] = std::make_tuple(flag::middle, pole::middle);
    currPlatform = platform::right;
  }
  
}

std::string getFlagString(flag flag) {
  std::string returnString = "";

  switch (flag) {
    case flag::high:
      returnString = "High Flag";
      break;
    case flag::middle:
      returnString = "Middle Flag";
      break;
    case flag::nothing:
      returnString = "No Flag";
      break;
  }

  return returnString;
}

std::string getPoleString(pole pole) {
  std::string returnString = "";

  switch (pole) {
    case pole::left:
      returnString = "Left Pole";
      break;
    case pole::middle:
      returnString = "Middle Pole";
      break;
    case pole::right:
      returnString = "Right Pole";
      break;
    case pole::nothing:
      returnString = "No Pole";
      break;
  }

  return returnString;
}

std::tuple<flag, pole> getFirstTurnShot() {
  buttonsPressed.clear();
  std::tuple<flag, pole> first = comboRecord[0];
  std::tuple<flag, pole> nothing =
      std::make_tuple(flag::nothing, pole::nothing);
  comboRecord[0] = nothing;
  std::string str = "1st turn shot: " + getFlagString(std::get<0>(first)) +
                    ", " + getPoleString(std::get<1>(first)) + "\n";
  printf(str.c_str());

  return first;
}

std::tuple<flag, pole> getSecondTurnShot() {
  std::tuple<flag, pole> second = comboRecord[1];
  std::tuple<flag, pole> nothing =
      std::make_tuple(flag::nothing, pole::nothing);
  comboRecord[1] = nothing;
  std::string str = "2nd turn shot: " + getFlagString(std::get<0>(second)) +
                    ", " + getPoleString(std::get<1>(second)) + "\n";
  printf(str.c_str());

  return second;
}

void resetCombos() {
  currentCombo = {flag::nothing, pole::nothing};
  comboRecord[0] = {flag::nothing, pole::nothing};
  comboRecord[1] = {flag::nothing, pole::nothing};
}

void printButtons() {
  std::string comboString = getComboString(buttonsPressed) + "\n";
  printf(comboString.c_str());
}

void print() {
  std::string recordStr = "(";
  auto firstCombo = comboRecord[0];
  recordStr += getFlagString(std::get<0>(firstCombo)) + ", " +
               getPoleString(std::get<1>(firstCombo));

  recordStr += "); (";

  auto secondCombo = comboRecord[1];
  recordStr += getFlagString(std::get<0>(secondCombo)) + ", " +
               getPoleString(std::get<1>(secondCombo));

  recordStr += ") combo: (";

  recordStr += getFlagString(std::get<0>(currentCombo)) + ", " +
               getPoleString(std::get<1>(currentCombo));

  recordStr += ")\n";

  printf(recordStr.c_str());
}

}  // namespace backshot