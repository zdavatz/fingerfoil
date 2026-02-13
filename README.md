# Fingerfoil

Blender script to generate 3D-printable hydrofoil parts (board, mast, fuselage, front wing, stabilizer) as STL files.

All geometry is generated procedurally — no input files required. Optionally, custom meshes can be provided for the board, mast, and fuselage.

## Requirements

- [Blender](https://www.blender.org/) (tested with 4.x)

## Usage

```bash
/Applications/Blender.app/Contents/MacOS/Blender -b -P fingerfoil.py -- [options]
```

On Linux:

```bash
blender -b -P fingerfoil.py -- [options]
```

## Options

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `-size` | float | `0.50` | Scale factor for all pieces. `1.0` = base size, `0.5` = half, `2.0` = double. |
| `-board` | path | — | Custom board mesh (`.stl`). Generated if omitted. |
| `-mast` | path | — | Custom mast mesh (`.stl` or `.step`). Generated if omitted. |
| `-fuse` | path | — | Custom fuselage mesh (`.stp` or `.step`). Generated if omitted. |

## Examples

Generate all parts at half size (default):

```bash
blender -b -P fingerfoil.py
```

Generate at full base size:

```bash
blender -b -P fingerfoil.py -- -size 1.0
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

Five STL files are written to `~/Desktop/pumpfoil_v6/`:

| File | Description |
|------|-------------|
| `1_board.stl` | Board with mast slot and magnet recess |
| `2_mast.stl` | Mast with NACA 0013 profile and screw holes |
| `3_fuselage.stl` | Fuselage with wing pockets, mast pocket, and screw holes |
| `4_frontwing.stl` | Front wing, NACA 2412, with 3 screw holes |
| `5_stabilizer.stl` | Stabilizer, NACA 0024, with 2 screw holes |

## Dimensions at `-size 1.0`

| Part | Dimensions |
|------|-----------|
| Board | 190 × 66 × 15mm |
| Mast | 170mm tall, 24mm chord |
| Fuselage | 123mm long, 9.5mm wide |
| Front wing | 200mm span, 25.2mm root chord |
| Stabilizer | 90mm span, 12.6mm root chord |
| Screws | M1.6 |

At `-size 0.5` all dimensions are halved (e.g. front wing = 100mm span, screws = M0.8).

## Assembly

- **Mast → Fuselage**: NACA pocket from top (50% depth), 2× vertical screws from bottom
- **Front wing → Fuselage**: smooth fairing on fuselage top, 3× vertical screws through fuselage
- **Stabilizer → Fuselage**: smooth fairing on fuselage top, 2× vertical screws through fuselage
- **Board → Mast**: rounded slot + magnet recess

## Wing Features

- Cubic B-spline spanwise interpolation (C2 continuous)
- Chord, sweep, twist, thickness via spline control points
- Hemispherical tip caps
- Finite trailing edge thickness (0.2% chord)
- Cosine section spacing (dense at tips)
- Catmull-Clark subdivision
- 2° washout on front wing
