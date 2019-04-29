#pragma once

namespace lcd {
enum autons { eBlueFront = 0, eRedFront = 1, eBlueBack = 2, eRedBack = 3, noAuton = 4 };

void initSelection();

void scrollRight();

void scrollLeft();

void confirm();

void run(void* p);

void runAuton();
}  // namespace lcd