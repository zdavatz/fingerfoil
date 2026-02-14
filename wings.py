import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import *

# ============================================================
# FRONT WING (piece 4)
# ============================================================

def make_front_wing(name):
    obj = loft_bspline_wing(
        name, FW_ROOT_CHORD, FW_TIP_CHORD, FW_SPAN,
        FW_SWEEP, FW_DIHEDRAL, FW_TWIST, FW_NACA,
        n_span_half=40)
    apply_all_modifiers(obj)

    fw_thick = FW_ROOT_CHORD * 0.12
    add_screw_holes(obj,
        [(-2.0*PS, 0, 0), (3.0*PS, 0, 0), (8.0*PS, 0, 0)],
        SCREW_DIAM, fw_thick + 6*PS, 'z')
    return obj


def build_frontwing():
    print("[4/5] Front wing (B-spline)...")
    clear_scene()
    fw = make_front_wing("FrontWing")
    export_stl(fw, "4_frontwing.stl")
    return fw


# ============================================================
# STABILIZER (piece 5)
# ============================================================

def make_stabilizer(name):
    obj = loft_bspline_wing(
        name, STAB_ROOT_CHORD, STAB_TIP_CHORD, STAB_SPAN,
        STAB_SWEEP, 0, 0, STAB_NACA,
        n_span_half=28)
    apply_all_modifiers(obj)

    stab_thick = STAB_ROOT_CHORD * 0.24
    st_shift = STAB_ROOT_CHORD * 0.25
    st_half_sp = STAB_ROOT_CHORD * 0.15
    add_screw_holes(obj,
        [(-st_half_sp + st_shift, 0, 0), (st_half_sp + st_shift, 0, 0)],
        SCREW_DIAM, stab_thick + 6*PS, 'z')
    return obj


def build_stabilizer():
    print("[5/5] Stabilizer (B-spline)...")
    clear_scene()
    stab = make_stabilizer("Stabilizer")
    export_stl(stab, "5_stabilizer.stl")
    return stab


if __name__ == "__main__":
    build_frontwing()
    build_stabilizer()
