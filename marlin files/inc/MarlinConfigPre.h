#pragma once

// Minimal shim for cppyy to compile Marlin trajectory headers.
// Define only what is required by trajectory_generator.h.
#ifndef OPTARG
  #define OPTARG(cond, val) , val
#endif
