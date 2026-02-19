# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Constant Jerk Motion Planner — a web app for visualizing S-curve (constant-jerk) trajectories used in robotics/CNC motion control. Core planning algorithms are written in C++ compiled to WebAssembly via Emscripten, with a React/TypeScript frontend for interactive visualization.

## Commands

```bash
# Development (auto-rebuilds WASM on C++ changes)
npm run dev

# Production build (TypeScript check + Vite build)
npm run build

# Lint
npm run lint

# Preview production build
npm run preview

# Manually rebuild WASM (normally automatic via Vite plugin)
bash src/wasm/build-wasm.sh

# Compile and run C++ tests (native, not WASM)
g++ -std=c++17 -O2 -Isrc/c src/c/test_trajectory.cpp -o src/c/test_trajectory && ./src/c/test_trajectory
```

**Emscripten prerequisite:** The WASM build script sources emsdk from `/Users/davidbuezas/code/emsdk/emsdk_env.sh`. Emscripten must be installed there for WASM compilation to work.

## Architecture

### Data Flow

User adjusts parameters → React UI (`App.tsx`) → WASM bridge (`planner_wasm_loader.ts`) → C++ planner → returns sampled trajectory arrays → Plotly.js renders 4 stacked plots (position, velocity, acceleration, jerk)

Parameter changes animate over 500ms using `requestAnimationFrame` with linear interpolation, replanning the trajectory each frame.

### C++ Core (`src/c/`)

- **`trajectory_constant_jerk.h`** — Header-only 7-phase S-curve generator. Phases: [+jerk, cruise_accel, -jerk, cruise_velocity, -jerk, cruise_decel, +jerk]. Uses binary search (48 iterations) to find feasible peak velocity. This is the core algorithm.
- **`constant-jerk-planner.h/.cpp`** — Block-based multi-segment planner. Uses velocity binning (64 bins) with forward/backward propagation through feasible state space. Handles multi-block trajectory sequences.
- **`miniplanner.h/.cpp`** — Marlin-style trapezoidal planner (reference implementation for comparison).
- **`test_trajectory.cpp`** — Native C++ unit tests for the trajectory generator.

### WASM Bridge (`src/wasm/`)

- **`planner_wrapper.cpp/.h`** — C API wrapper exposing planning functions to JavaScript via Emscripten's `cwrap`.
- **`planner_wasm_loader.ts`** — TypeScript module that loads the WASM module and provides typed wrapper functions. Samples trajectories at regular intervals (up to 10,000 points).
- **`build-wasm.sh`** — Emscripten build script (C++17, -O2, ES6 module output).

### Vite WASM Plugin (`vite.config.ts`)

A custom Vite plugin watches C++ source files and automatically triggers WASM rebuild + full browser reload on changes. Watched files: `build-wasm.sh`, `planner_wrapper.cpp/.h`, `trajectory_constant_jerk.h`, `constant-jerk-planner.h/.cpp`.

### Frontend (`src/App.tsx`)

Single-component app with 10 input parameters (entry/exit velocity/acceleration, nominal speed, distance, jerk/acceleration limits, sample rate). Enforces constraints between parameters (e.g., nominal speed ≥ max of entry/exit velocities). Displays planner status and plan time in microseconds.

## Tech Stack

- React 19 + TypeScript 5.9, Vite 8 (beta), React Compiler enabled
- Plotly.js for visualization, Tailwind CSS 3 for styling
- C++17 compiled to WASM via Emscripten
