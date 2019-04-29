#pragma once

#include "robotIO.hpp"

namespace subsystems
{

namespace angler
{

enum anglerState : int
{ // positionFlagheight = target
    frontTop = 0,
    frontMid = 1,
    platformTop = 2,
    platformMid = 3,
    backTop = 4, // behind center platform
    backMid = 5,
    sideTop, // behind the side platforms
    sideMid,
    midTop,
    midMid,
    turnshotTop,
    turnshotMid
}; // modify the .cpp file slightly so you don't have to make all to compile this file

void changeState(anglerState desired);

void changePosition(int pos);

void run(void* p);
} // namespace angler
} // namespace subsystems