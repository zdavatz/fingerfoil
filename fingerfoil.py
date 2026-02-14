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
#   -size      FLOAT   Scale all pieces (default: 0.50)
#   -board     PATH    Board mesh (.stl)
#   -mast      PATH    Mast mesh (.stl or .step)
#   -fuse      PATH    Fuselage mesh (.stp or .step)
#   -combined          Also export all pieces assembled as 0_combined.stl
#
# Example:
#   blender -b -P fingerfoil.py -- -size 0.5 -combined

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

# ── Assembled combined export ──
if COMBINED:
    print("\n[6/6] Assembling combined model...")
    clear_scene()

    stl_files = ["1_board.stl", "2_mast.stl", "3_fuselage.stl",
                 "4_frontwing.stl", "5_stabilizer.stl"]

    # Board is at origin, other parts need positioning relative to it
    # Mast: hangs below board at 80% from nose
    mast_board_x = BOARD_LENGTH * 0.30  # 80% from nose = +30% from center

    # Fuselage: hangs below mast, at bottom of mast
    fuse_fh = 6.0 * PS  # fuselage height

    # Front wing: on fuselage at 11% of fuse length
    fw_fy = FUSE_LENGTH * 0.11

    # Stabilizer: on fuselage at 88% of fuse length
    stab_sy = FUSE_LENGTH * 0.88

    positions = {
        "1_board.stl":      (0, 0, 0),
        "2_mast.stl":       (mast_board_x, 0, -BOARD_THICK/2 - MAST_HEIGHT),
        "3_fuselage.stl":   (mast_board_x - FUSE_LENGTH * 0.44, 0, -BOARD_THICK/2 - MAST_HEIGHT),
        "4_frontwing.stl":  (mast_board_x - FUSE_LENGTH * 0.44 + fw_fy, 0, -BOARD_THICK/2 - MAST_HEIGHT + fuse_fh),
        "5_stabilizer.stl": (mast_board_x - FUSE_LENGTH * 0.44 + stab_sy, 0, -BOARD_THICK/2 - MAST_HEIGHT + fuse_fh),
    }

    for stl_name in stl_files:
        stl_path = os.path.join(export_dir, stl_name)
        if not os.path.exists(stl_path):
            print(f"  Warning: {stl_name} not found, skipping")
            continue
        try:
            bpy.ops.wm.stl_import(filepath=stl_path)
        except:
            bpy.ops.import_mesh.stl(filepath=stl_path)
        obj = bpy.context.active_object
        if obj:
            obj.name = stl_name.replace('.stl', '')
            px, py, pz = positions.get(stl_name, (0, 0, 0))
            obj.location = (px, py, pz)
            bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)

    # Select all and export
    bpy.ops.object.select_all(action='SELECT')
    combined_path = os.path.join(export_dir, "0_combined.stl")
    bpy.ops.wm.stl_export(filepath=combined_path)
    print(f"  -> 0_combined.stl")

print(f"\n{'='*60}")
print(f"  Done! → {export_dir}")
print(f"{'='*60}")
print(f"""
  1_board.stl        {BOARD_LENGTH:.0f} x {BOARD_WIDTH:.0f} x {BOARD_THICK:.1f}mm {'('+BOARD_FILE+')' if BOARD_FILE else '(generated)'}
  2_mast.stl         {MAST_HEIGHT:.0f}mm tall, {MAST_CHORD:.0f}mm chord, {MAST_CHORD*0.13:.1f}mm thick, NACA {MAST_NACA} {'('+MAST_FILE+')' if MAST_FILE else '(generated)'}
  3_fuselage.stl     {FUSE_LENGTH:.1f}mm long, {4.76*2*PS:.1f}mm wide {'('+FUSE_FILE+')' if FUSE_FILE else '(generated)'}
  4_frontwing.stl    {FW_SPAN:.0f}mm span, {FW_ROOT_CHORD:.1f}mm root chord, {FW_ROOT_CHORD*0.12:.1f}mm thick, NACA {FW_NACA}
  5_stabilizer.stl   {STAB_SPAN:.0f}mm span, {STAB_ROOT_CHORD:.1f}mm root chord, {STAB_ROOT_CHORD*0.24:.1f}mm thick, NACA {STAB_NACA}
{"  0_combined.stl    All pieces assembled" if COMBINED else ""}
  Wing smoothness:
    • Cubic B-spline spanwise interpolation (C2 continuous)
    • Chord, sweep, twist, thickness all via spline control points
    • Hemispherical tip caps (8-ring cos-shrink to center point)
    • Finite trailing edge thickness (0.2% chord)
    • Cosine section spacing (dense at tips)
    • 1-level Catmull-Clark subdivision on top
    • {FW_TWIST}° washout on front wing
""")
