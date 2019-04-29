#include "shooting.hpp"
#include "backshot/executor.hpp"
#include "subsystems/intake.hpp"

namespace shooting
{

shootingState currState = notShooting;
shootingState lastState = notShooting;
shootingArea currArea = front;

std::uint32_t lastShot = 0;
lv_obj_t *limitPressedLastShotLabel;
lv_obj_t *limitReleasedLastShotLabel;

bool hasDoubleShot = false;
int customPositionFirst = 0;
int customPositionSecond = 0;
int customDoubleshotDelay = 0;
int customDoubleshotIndexerSpeed = 200;

int scraperSpeed = 200;
int scraperTarget = -185;


void init() {
    limitPressedLastShotLabel = lv_label_create(flywheelTab, NULL);
    lv_label_set_text(limitPressedLastShotLabel, "#008000 limit changed to pressed during last shot#");
    lv_obj_align(limitPressedLastShotLabel, NULL, LV_ALIGN_IN_TOP_MID, 40, 140);
    lv_label_set_recolor(limitPressedLastShotLabel, true);

    limitReleasedLastShotLabel = lv_label_create(flywheelTab, NULL);
    lv_label_set_text(limitReleasedLastShotLabel, "#008000 limit changed to released at the end of last shot#"); // if it was released it means angler moved for double
    lv_obj_align(limitReleasedLastShotLabel, NULL, LV_ALIGN_IN_TOP_MID, 40, 180);
    lv_label_set_recolor(limitReleasedLastShotLabel, true);
}

void update(okapi::ControllerButton *trigger, okapi::ControllerButton *indexReverse)
{
    customDoubleshotIndexerSpeed = 200;
    if (trigger->changedToPressed())
    {
        //lv_label_set_text(limitPressedLastShotLabel, "#ff0000 limit did not change to pressed during last shot#");
        //lv_label_set_text(limitReleasedLastShotLabel, "#ff0000 limit did not change to released at the end of last shot#");
        currState = first;
        return;
    }
    if (trigger->changedToReleased())
    {
        currState = notShooting;
        return;
    }
    if (trigger->isPressed())
    {
        return;
    }

    //printf("trigger: %d, indexReverse: %d\n", (trigger->isPressed()) ? 1 : 0, (indexReverse->isPressed()) ? 1 : 0);
    lastState = currState;
    if (indexReverse->changedToPressed())
    {
        if (currState != indexerReverse) {
            scraperSpeed = 200;
            scraperTarget = -185;
            currState = indexerReverse;
        } else {
            currState = notShooting;
        }
    }
    else if (!indexReverse->isPressed() && currState != indexerReverse)
    {
        currState = notShooting;
    }
}

bool indexerReverseIsToggled = false;
void run()
{
    if (currState != indexerReverse) {
        indexerReverseIsToggled = false;
    }
    switch (currState)
    {
    case disabled:
        break;
    case indexerReverse:
        //IndexerMotor.moveVelocity(-200); //absolute meme
        if (!indexerReverseIsToggled) {
            IndexerMotor.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
            IndexerMotor.moveRelative(scraperTarget, scraperSpeed);
            indexerReverseIsToggled = true;
        }
        break;
    case indexerReleased:
        IndexerMotor.moveVoltage(0);
        break;
    case singleShot: // ball needs to be backed up against indexer
        IndexerMotor.moveVelocity(200);
        if (BallDetectOkapi.changedToReleased()) {
            currState = notShooting;
            lastState = singleShot;
        }
        break;
    case notShooting:
        if (!backshot::isTurnShotting) {
            switch (currArea)
            {
            case front:
                subsystems::angler::changeState(subsystems::angler::frontTop);
                break;
            case platform:
                subsystems::angler::changeState(subsystems::angler::platformTop);
                break;
            case back:
                subsystems::angler::changeState(subsystems::angler::backTop);
                break;
            }
        }
        //flywheel::changeSpeed(flywheel::normal);
        IndexerMotor.moveVelocity(0);
        break;
    case first:
        hasDoubleShot = false;
        if (lastState != first)
            lastShot = pros::millis();

        switch (currArea)
        {
        case front:
            subsystems::angler::changeState(subsystems::angler::frontTop);
            break;
        case platform:
            subsystems::angler::changeState(subsystems::angler::platformTop);
            break;
        case back:
            subsystems::angler::changeState(subsystems::angler::backTop);
            break;
        }
        IndexerMotor.moveVelocity(200);
        // if (pros::millis() - lastShot < 100) { // timer
        //     break;
        // }
        if (BallDetectOkapi.isPressed())
        {
            currState = second;
            lastState = first;
            //lv_label_set_text(limitPressedLastShotLabel, "#008000 limit changed to pressed during last shot#");
        }

        break;
    case second:
        if (BallDetectOkapi.changedToReleased())
        {
            //lv_label_set_text(limitReleasedLastShotLabel, "#008000 limit changed to released at the end of last shot#");
            switch (currArea)
            {
            case front:
                subsystems::angler::changeState(subsystems::angler::frontMid);
                break;
            case platform:
                subsystems::angler::changeState(subsystems::angler::platformMid);
                break;
            case back:
                subsystems::angler::changeState(subsystems::angler::backMid);
                break;
            }
            hasDoubleShot = true;
        }
        break;
    case customFirst:
        hasDoubleShot = false;
        if (lastState != first)
            lastShot = pros::millis();
        subsystems::intake::changeState(subsystems::intake::released);
        IntakeMotor.moveVelocity(100);

        subsystems::angler::changePosition(customPositionFirst);

        IndexerMotor.moveVelocity(customDoubleshotIndexerSpeed);
        // if (pros::millis() - lastShot < 100) { // timer
        //     break;
        // }
        if (BallDetectOkapi.changedToReleased()) {
            IndexerMotor.moveVelocity(0);
            customDoubleshotIndexerSpeed = 0;
            pros::delay(customDoubleshotDelay);
            IndexerMotor.moveVelocity(200);
        }
        if (BallDetectOkapi.isPressed())
        {
            currState = customSecond;
            lastState = customFirst;
            //lv_label_set_text(limitPressedLastShotLabel, "#008000 limit changed to pressed during last shot#");
        }

        break;
    case customSecond:
        if (BallDetectOkapi.changedToReleased())
        {
            //lv_label_set_text(limitReleasedLastShotLabel, "#008000 limit changed to released at the end of last shot#");
            subsystems::angler::changePosition(customPositionSecond);
            hasDoubleShot = true;
        }
        break;
    }
}

void runTask(void* p) {
    while (true) {
        run();
        pros::delay(20);
    }
}

void doubleshot() {
    hasDoubleShot = false;
    lastState = currState;
    currState = first;
    while (!hasDoubleShot) {
        pros::delay(20);
    }
    pros::delay(700);
    currState = notShooting;
}

void doubleshotCustom(int firstAngle, int secondAngle) {
    hasDoubleShot = false;
    customPositionFirst = firstAngle;
    customPositionSecond = secondAngle;
    customDoubleshotDelay = 0;
    customDoubleshotIndexerSpeed = 200;
    lastState = currState;
    currState = customFirst;
    while (!hasDoubleShot) {
        pros::delay(20);
    }
    pros::delay(400);
    //pros::delay(500);
    currState = notShooting;
}

void doubleshotCustom(int firstAngle, int secondAngle, int delay) {
    hasDoubleShot = false;
    customPositionFirst = firstAngle;
    customPositionSecond = secondAngle;
    customDoubleshotDelay = delay;
    customDoubleshotIndexerSpeed = 200;
    lastState = currState;
    currState = customFirst;
    while (!hasDoubleShot) {
        pros::delay(20);
    }
    pros::delay(400);
    //pros::delay(500);
    currState = notShooting;
}

void changeState(shootingState state) {
    lastState = currState;
    currState = state;
}
} // namespace shooting