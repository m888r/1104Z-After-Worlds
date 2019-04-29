#pragma once

#include "robotIO.hpp"
#include "motioncontrol.hpp"
#include "subsystems/intake.hpp"
#include "shooting.hpp"
#include "main.h"

void initBlueFront();
void blueFront();

void initRedFront();
void redFront();

void initBlueBack();
void blueBack();

void initRedBack();
void redBack();

void blueBackMidPoleOnly();

void blueFrontPark();