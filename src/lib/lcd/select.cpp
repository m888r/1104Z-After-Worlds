#include "select.hpp"
#include "api.h"
#include "okapi/api.hpp"
#include "robotIO.hpp"
#include "auton/autons.hpp"

namespace lcd {
int currCount = 4;
bool selected = false;
int numAutons = 5;

lv_obj_t* autonLabel;
lv_obj_t* note;

void initSelection() {
  autonLabel = lv_label_create(autonTab, NULL);
  lv_label_set_text(autonLabel, "");
  lv_obj_align(autonLabel, autonTab, LV_ALIGN_IN_TOP_MID, 0, 20);
  note = lv_label_create(autonTab, NULL);
  lv_obj_align(note, autonTab, LV_ALIGN_CENTER, 0, 0);

  pros::Task selectionTask(run);
}


void scrollRight() { 
  currCount++;
  if (currCount > numAutons - 1) {
    currCount = 0;
  }
}

void scrollLeft() {
  currCount--;
  if (currCount < 0) {
    currCount = numAutons - 1;
  }
}

void confirm() { selected = true; }

void run(void* p) {
  while (!selected) {
    if (okapiLeftButton.changedToPressed()) {
      currCount--;
      if (currCount < 0) {
        currCount = numAutons - 1;
      }
    }
    if (okapiCenterButton.changedToPressed()) {
      selected = true;
    }
    if (okapiRightButton.changedToPressed()) {
      currCount++;
      if (currCount > numAutons - 1) {
        currCount = 0;
      }
    }

    switch (currCount) {
      case eBlueFront:
        lv_label_set_text(autonLabel, "[BLUE][FRONT][6FLAG]");
        break;
      case eBlueBack:
        lv_label_set_text(autonLabel, "[BLUE][BACK][4FLAG][PARK]");
        break;
      case eRedFront:
        lv_label_set_text(autonLabel, "[RED][FRONT][6Flag]");
        break;
      case eRedBack:
        lv_label_set_text(autonLabel, "[RED][BACK][4FLAG][PARK]");
        break;
      default:
        lv_label_set_text(autonLabel, "[NONE]");
        break;
    }

    if (selected) {
      lv_label_set_text(note, "AUTO SELECTED");
    }

    pros::delay(20);
  }
}

void runAuton() {
  switch (currCount) {
    case eBlueFront:
      blueFront();
      break;
    case eBlueBack:
      blueBack();
      break;
    case eRedFront:
      redFront();
      break;
    case eRedBack:
      redBack();
      break;
    default:
      break;
  }
}

}  // namespace lcd