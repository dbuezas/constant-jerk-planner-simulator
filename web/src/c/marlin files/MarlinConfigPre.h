#pragma once

#include <cmath>

// Minimal shim for cppyy to compile Marlin trajectory headers.
// Define only what is required by trajectory_generator.h.
#ifndef FTM_RESONANCE_TEST
  #define FTM_RESONANCE_TEST 0
#endif

#ifndef OPTARG
  #define OPTARG(cond, val) , val
#endif

#ifndef sq
  #define sq(x) ((x) * (x))
#endif

#ifndef SQRT
  #define SQRT(x) (sqrtf(x))
#endif
