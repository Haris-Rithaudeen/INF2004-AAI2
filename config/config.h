#pragma once
#include "pico/stdlib.h"   // keep this if other files depended on it

// ---- Modular includes (non-breaking, safe even if empty) ----
#if __has_include("config/pins.h")
  #include "config/pins.h"
#endif
#if __has_include("config/robot_params.h")
  #include "config/robot_params.h"
#endif
#if __has_include("config/control_params.h")
  #include "config/control_params.h"
#endif
#if __has_include("config/sensors_params.h")
  #include "config/sensors_params.h"
#endif
#if __has_include("config/network.h")
  #include "config/network.h"
#endif
// --------------------------------------------------------------
