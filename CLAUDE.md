# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Commands
- No build, test, or lint commands are defined in-repo. If you add one, document it here.

## Editing
- Use the `Edit` tool for all code changes. Do not use shell patching or other patch commands.
- Use actual newlines in Jupyter notebook cells to avoid noisy git diffs (see `_PLANNER.ipynb`).

## Architecture
- `constant-jerk-planner.h` / `constant-jerk-planner.cpp`: C-compatible API for the constant-jerk planner. Implements a forward/backward reachability envelope over a velocity grid, intersects feasible sets, then greedily selects per-junction `(v,a)` states.
- `miniplanner.h` / `miniplanner.cpp`: Minimal Marlin-style 1D trapezoidal planner used for reference and comparison (reverse/forward pass over entry speeds).
- `marlin files/`: Snapshot of Marlin planner headers/sources and trajectory helpers (`trajectory_*`) used as a reference context.
- `_PLANNER.ipynb` and `planner_plot.py`: Exploratory notebook and plotting helper (Plotly) for visualizing motion profiles.
