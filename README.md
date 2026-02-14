# Fingerfoil

Blender script to generate 3D-printable hydrofoil parts (board, mast, fuselage, front wing, stabilizer) as STL files.

All geometry is generated procedurally — no input files required. Optionally, custom meshes can be provided for the board, mast, and fuselage.

## Requirements

- [Blender](https://www.blender.org/) (tested with 4.x)

## File Structure

| File | Description |
|------|-------------|
| `fingerfoil.py` | Main entry point — runs all builders, optional combined assembly |
| `common.py` | Shared constants, CLI parsing, B-spline math, NACA profiles, wing lofting, utilities |
| `board.py` | Board geometry — skateboard-style rounded nose/tail, elliptical rail cross-sections |
| `mast.py` | Mast body with NACA profile, mastfoot fairing, screw holes |
| `fuselage.py` | Fuselage with wing junction fairings, mast pocket, screw holes |
| `wings.py` | Front wing and stabilizer builders |

Each piece file has a `build_*()` function and `if __name__ == "__main__"` for standalone execution.

## Python venv setup

Blender ships its own Python, but if you need additional packages (e.g. for post-processing), set up a venv using Blender's Python:

```bash
# Find Blender's Python
BLENDER_PYTHON=$(/Applications/Blender.app/Contents/MacOS/Blender -b --python-expr "import sys; print(sys.executable)" 2>/dev/null | grep python)

# Create venv
$BLENDER_PYTHON -m venv ~/.blender_venv

# Activate
source ~/.blender_venv/bin/activate

# Install packages if needed
pip install numpy
```

On Linux:

```bash
BLENDER_PYTHON=$(blender -b --python-expr "import sys; print(sys.executable)" 2>/dev/null | grep python)
$BLENDER_PYTHON -m venv ~/.blender_venv
source ~/.blender_venv/bin/activate
```

Note: Fingerfoil only uses Blender's built-in modules (`bpy`, `bmesh`, `math`, `os`, `sys`) — no extra packages are needed.

## Usage

```bash
/Applications/Blender.app/Contents/MacOS/Blender -b -P fingerfoil.py -- [options]
```

On Linux:

```bash
blender -b -P fingerfoil.py -- [options]
```

Build a single piece standalone:

```bash
blender -b -P board.py -- -size 0.5
blender -b -P wings.py -- -size 0.5
```

## Options

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `-size` | float | `0.50` | Scale factor for all pieces. `1.0` = base size, `0.5` = half. |
| `-board` | path | — | Custom board mesh (`.stl`). Generated if omitted. |
| `-mast` | path | — | Custom mast mesh (`.stl` or `.step`). Generated if omitted. |
| `-fuse` | path | — | Custom fuselage mesh (`.stp` or `.step`). Generated if omitted. |
| `-combined` | flag | off | Also export all 5 pieces assembled as `0_combined.stl`. |

## Examples

Generate all parts at half size (default):

```bash
blender -b -P fingerfoil.py
```

Generate at full base size with combined assembly:

```bash
blender -b -P fingerfoil.py -- -size 1.0 -combined
```

Use a custom board mesh at 50% scale:

```bash
blender -b -P fingerfoil.py -- -size 0.5 -board ~/Desktop/my_board.stl
```

Use all custom input files:

```bash
blender -b -P fingerfoil.py -- -size 0.5 \
  -board ~/Desktop/Pumpskate_Zeno_2025.stl \
  -mast ~/Desktop/85cm_mast.STEP \
  -fuse ~/Desktop/61cm_onix_indiana.stp
```

## Output

STL files are written to `~/Desktop/fingerfoil/`:

| File | Description |
|------|-------------|
| `1_board.stl` | Board with rounded rails, skateboard-style nose/tail |
| `2_mast.stl` | Mast with NACA 0013 profile and screw holes |
| `3_fuselage.stl` | Fuselage with wing fairings, mast pocket, and screw holes |
| `4_frontwing.stl` | Front wing, NACA 2412, with 3 screw holes |
| `5_stabilizer.stl` | Stabilizer, NACA 0024, with 2 screw holes |
| `0_combined.stl` | All 5 pieces assembled (only with `-combined` flag) |

## Dimensions at `-size 1.0`

| Part | Dimensions |
|------|-----------|
| Board | 190 × 66 × 10.2mm |
| Mast | 170mm tall, 24mm chord, NACA 0013 |
| Fuselage | 123mm long, 9.5mm wide |
| Front wing | 200mm span, 25.2mm root chord, NACA 2412 |
| Stabilizer | 45mm span, 12.6mm root chord, NACA 0024 |
| Screws | M1.6 |

At `-size 0.5` all dimensions are halved (e.g. front wing = 100mm span, board = 95 × 33 × 5.1mm).

## Board Details

- Planform outline from Hydroskate reference (57% nose, 48% tail width)
- Elliptical rail cross-sections (semicircle of full half-thickness)
- Flat deck with subtle 3% crown, flat bottom
- Skateboard-style blunt rounded nose and tail (12-ring hemispherical caps)
- Cosine-spaced lengthwise stations (120×64) for smooth subdivision
- Thickness tapers at ends (minimum 40%), width stays wide

## Assembly

- **Mast → Board**: mast foot sits under board, secured by screws
- **Mast → Fuselage**: NACA pocket from top (50% depth), 2× vertical screws from bottom
- **Front wing → Fuselage**: smooth fairing on fuselage top, 3× vertical screws through fuselage
- **Stabilizer → Fuselage**: smooth fairing on fuselage top, 2× vertical screws through fuselage

## Wing Features

- Cubic B-spline spanwise interpolation (C2 continuous)
- SplineDistribution class with control points for chord, sweep, twist, thickness
- Hemispherical tip caps (8-ring cos-shrink to center point)
- Finite trailing edge thickness (0.2% chord)
- Cosine section spacing (dense at tips)
- Catmull-Clark subdivision
- 2° washout on front wing
