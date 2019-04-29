#include "flywheel.hpp"
#include "tbh.hpp"

namespace subsystems
{

namespace flywheel
{

flywheelSpeed currSpeed = stop;

void init()
{
    pros::Task FlywheelTask(FwControlTask, NULL);
    currSpeed = stop;
    FwVelocitySet(&flywheelController, currSpeed, 0.75);
}
 
void run()   
{    
    //FwVelocitySet(&flywheelController, currSpeed, 0.75);
}

void changeSpeed(flywheelSpeed speed)
{ 
    if (speed != currSpeed)
    {
        currSpeed = speed;
        FwVelocitySet(&flywheelController, currSpeed, 0.75);
    }
}

bool isAtRpm() {
    return (abs(currSpeed - FlywheelMotor.getActualVelocity()) < 10);
}

} // namespace flywheel

} // namespace subsystems