#pragma once

//#include "executor.hpp"
#include "okapi/api.hpp"
#include "robotIO.hpp"

namespace backshot {

enum class flag { high, middle, nothing };

enum class pole { left, right, middle, nothing };

enum class button { up, down, left, right };

enum class platform { right, left, center, nothing };

extern platform currPlatform;

/**
 * Clear when instructions go past the maximum, and then doesn't allow
 * new instructions for a period of time.
 * @return true if replaced an old recorded value
 */
bool record();

/**
 * Clear when instructions go past 3 button presses
 * Don't allow new instructions for a period of time after u go past 3
 * @return true if cleared the thing
 */
bool recordButtons();

std::tuple<flag, pole> getFirstTurnShot();
std::tuple<flag, pole> getSecondTurnShot();

void resetCombos();

void setCombos();

void print();
void printButtons();

std::string getFlagString(flag flag);

std::string getPoleString(pole pole);
}  // namespace backshot