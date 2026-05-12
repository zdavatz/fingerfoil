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

# Paper-aligned geometry (Yim & Gallaire)
blender -b -P fingerfoil.py -- -size 1.0 -combined -epfl
```

Options: `-size FLOAT` (default 0.50), `-board PATH`, `-mast PATH`, `-fuse PATH`, `-combined`, `-original` (default), `-epfl`.

Output goes to `~/Desktop/fingerfoil/`.

## Geometry modes

Two geometry presets, selectable at build time:

- **`-original`** (default) — Stock fingerfoil dimensions. Mast pivot at 44% of fuselage length, near-symmetric front/rear lever arms.
- **`-epfl`** — Paper-aligned with Yim & Gallaire, *A minimal model of pump foil dynamics* (`pdf/Pumpfoil_Model.pdf`). Mast pivot at 21.3% of fuselage, longer fuse (175mm), longer mast (190mm), wider stab span (95mm). Scaled 1:5.25 from the paper's Table I to match real-pumpfoil proportions. See GitHub issue #1 for the derivation.

Modes are set by branching in `common.py` on the `GEOMETRY` variable. The fractions `FUSE_FW_X_FRAC`, `FUSE_STAB_X_FRAC`, `FUSE_MAST_X_FRAC` and the dimensions `MAST_HEIGHT`, `FUSE_LENGTH`, `STAB_SPAN` depend on the mode; everything else (wing chords, NACA codes, etc.) is shared.

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
