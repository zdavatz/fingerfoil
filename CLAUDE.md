# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Fingerfoil is a Blender-based procedural generator for 3D-printable hydrofoil parts. It uses Python with the Blender API (`bpy`, `bmesh`) to generate board, mast, fuselage, front wing, and stabilizer as STL files. No external Python packages are required.

## Build & Run Commands

```bash
# Generate all parts (Linux)
blender -b -P fingerfoil.py -- [options]

# macOS
/Applications/Blender.app/Contents/MacOS/Blender -b -P fingerfoil.py -- [options]

# Build single piece standalone
blender -b -P board.py -- -size 0.5
blender -b -P wings.py -- -size 0.5

# Full size with combined assembly
blender -b -P fingerfoil.py -- -size 1.0 -combined
```

Options: `-size FLOAT` (default 0.50), `-board PATH`, `-mast PATH`, `-fuse PATH`, `-combined`.

Output goes to `~/Desktop/fingerfoil/`.

## Architecture

- **`fingerfoil.py`** — Orchestrator. Calls all 5 builders sequentially, optionally assembles combined STL.
- **`common.py`** — Shared core: CLI parsing, constants/dimensions (scaled by `PIECE_SCALE`), cubic B-spline math (`SplineDistribution`), `naca_4digit()` airfoil generator, `loft_bspline_wing()` for wing geometry, and utility functions (import/export, boolean ops for screw holes/slots).
- **`board.py`** — Board with Hydroskate planform, elliptical rails, hemispherical caps.
- **`mast.py`** — NACA 0013 profile lofted vertically, superellipse mastfoot fairing.
- **`fuselage.py`** — Elliptical body with NACA wing fairings blended in, mast pocket, 7 screw holes.
- **`wings.py`** — Front wing (NACA 2412) and stabilizer (NACA 0024) using `loft_bspline_wing()`.

Each piece module follows the pattern: `make_placeholder_*()` generates procedural geometry, `build_*()` orchestrates (tries custom mesh import, falls back to procedural), and has `if __name__ == "__main__"` for standalone execution.

## Key Conventions

- **Coordinate system**: X = lengthwise, Y = spanwise, Z = vertical.
- **All dimensions** in `common.py` are multiplied by `PS = PIECE_SCALE` (from `-size` arg, default 0.50).
- **Piece naming**: numbered prefix `1_board`, `2_mast`, `3_fuselage`, `4_frontwing`, `5_stabilizer`.
- **Wing geometry** is controlled by 4 `SplineDistribution` curves (chord, sweep, twist, thickness) evaluated along the span with cosine-spaced stations.
- **Mesh operations** use `bmesh` for vertex-level construction, Blender modifiers for booleans (screw holes, mast pocket), and Catmull-Clark subdivision for smoothing.
