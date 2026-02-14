import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
#!/usr/bin/env python3
# ============================================================
# FINGERFOIL — B-spline lofted hydrofoil parts
# ============================================================
# Usage:
#   Blender -b -P fingerfoil.py -- [options]
#
# Options:
#   -size  FLOAT   Scale all pieces (default: 0.50)
#   -board PATH    Board mesh (.stl)
#   -mast  PATH    Mast mesh (.stl or .step)
#   -fuse  PATH    Fuselage mesh (.stp or .step)
#
# Example:
#   /Applications/Blender.app/Contents/MacOS/Blender -b -P fingerfoil.py -- -size 1.0
#   /Applications/Blender.app/Contents/MacOS/Blender -b -P fingerfoil.py -- -size 0.5 -board ~/my_board.stl

from common import *
from board import build_board
from mast import build_mast
from fuselage import build_fuselage
from wings import build_frontwing, build_stabilizer

print("\n" + "="*60)
print("  FINGERFOIL — B-spline wings")
print("="*60 + "\n")

build_board()
build_mast()
build_fuselage()
build_frontwing()
build_stabilizer()

print(f"\n{'='*60}")
print(f"  Done! → {export_dir}")
print(f"{'='*60}")
print(f"""
  1_board.stl        {BOARD_LENGTH:.0f} x {BOARD_WIDTH:.0f} x {BOARD_THICK:.1f}mm {'('+BOARD_FILE+')' if BOARD_FILE else '(generated)'}
  2_mast.stl         {MAST_HEIGHT:.0f}mm tall, {MAST_CHORD:.0f}mm chord, {MAST_CHORD*0.13:.1f}mm thick, NACA {MAST_NACA} {'('+MAST_FILE+')' if MAST_FILE else '(generated)'}
  3_fuselage.stl     {FUSE_LENGTH:.1f}mm long, {4.76*2*PS:.1f}mm wide {'('+FUSE_FILE+')' if FUSE_FILE else '(generated)'}
  4_frontwing.stl    {FW_SPAN:.0f}mm span, {FW_ROOT_CHORD:.1f}mm root chord, {FW_ROOT_CHORD*0.12:.1f}mm thick, NACA {FW_NACA}
  5_stabilizer.stl   {STAB_SPAN:.0f}mm span, {STAB_ROOT_CHORD:.1f}mm root chord, {STAB_ROOT_CHORD*0.24:.1f}mm thick, NACA {STAB_NACA}

  Wing smoothness:
    • Cubic B-spline spanwise interpolation (C2 continuous)
    • Chord, sweep, twist, thickness all via spline control points
    • Hemispherical tip caps (8-ring cos-shrink to center point)
    • Finite trailing edge thickness (0.2% chord)
    • Cosine section spacing (dense at tips)
    • 1-level Catmull-Clark subdivision on top
    • {FW_TWIST}° washout on front wing
""")
