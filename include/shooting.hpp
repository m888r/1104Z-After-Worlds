#pragma once

#include "robotIO.hpp"
#include "subsystems/flywheel.hpp"
#include "subsystems/angler.hpp"
#include "okapi/api.hpp"

namespace shooting {
    enum shootingState {
        first = 0,
        second = 1,
        notShooting = 2,
        disabled = 3,
        singleShot,
        indexerReleased,
        indexerReverse,
        customFirst,
        customSecond
    };
    
    enum shootingArea {
        front,
        back,
        platform
    };
    
    extern shootingState currState;
    extern shootingState lastState;
    extern shootingArea currArea;

    extern int scraperSpeed;
    extern int scraperTarget;
    //extern bool isShooting;

    extern std::uint32_t lastShot;

    extern bool hasDoubleShot;

    extern lv_obj_t *limitPressedLastShotLabel;
    extern lv_obj_t *limitReleasedLastShotLabel;

    void init();

    void update(okapi::ControllerButton* trigger, okapi::ControllerButton* indexReverse);

    void run();

    void runTask(void* p);

    void waitUntilDoubleshot();

    void doubleshot();

    void doubleshotCustom(int firstAngle, int secondAngle);
    void doubleshotCustom(int firstAngle, int secondAngle, int delay);

    void changeState(shootingState state);
}