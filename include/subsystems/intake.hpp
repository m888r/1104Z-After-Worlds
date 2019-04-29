#pragma once

#include "api.h"
#include "okapi/api.hpp"

namespace subsystems {
namespace intake {
  enum intakeState {
    intaking,
    outtaking,
    forceIntaking,
    stopped,
    released
  };

  void changeState(intakeState state);

  void run(void* p);
}
}  // namespace subsystems