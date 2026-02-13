import bpy
import bmesh
import math
import os

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
# All input files are optional — generated geometry is used when omitted.
#
# Example:
#   /Applications/Blender.app/Contents/MacOS/Blender -b -P fingerfoil.py -- -size 1.0
#   /Applications/Blender.app/Contents/MacOS/Blender -b -P fingerfoil.py -- -size 0.5 -board ~/my_board.stl

export_dir = os.path.expanduser("~/Desktop/fingerfoil")
os.makedirs(export_dir, exist_ok=True)

SCALE = 0.20

# ── Command-line options ──
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
#   /Applications/Blender.app/Contents/MacOS/Blender -b -P fingerfoil.py -- -size 0.5 -board ~/Desktop/board.stl -mast ~/Desktop/mast.STEP

import sys
PIECE_SCALE = 0.50
BOARD_FILE = None
MAST_FILE  = None
FUSE_FILE  = None

argv = sys.argv
if '--' in argv:
    custom_args = argv[argv.index('--') + 1:]
    i = 0
    while i < len(custom_args):
        arg = custom_args[i]
        if arg == '-size' and i + 1 < len(custom_args):
            PIECE_SCALE = float(custom_args[i + 1]); i += 2
        elif arg == '-board' and i + 1 < len(custom_args):
            BOARD_FILE = os.path.expanduser(custom_args[i + 1]); i += 2
        elif arg == '-mast' and i + 1 < len(custom_args):
            MAST_FILE = os.path.expanduser(custom_args[i + 1]); i += 2
        elif arg == '-fuse' and i + 1 < len(custom_args):
            FUSE_FILE = os.path.expanduser(custom_args[i + 1]); i += 2
        else:
            i += 1

print(f"  PIECE_SCALE = {PIECE_SCALE}")
if BOARD_FILE: print(f"  Board file:  {BOARD_FILE}")
if MAST_FILE:  print(f"  Mast file:   {MAST_FILE}")
if FUSE_FILE:  print(f"  Fuse file:   {FUSE_FILE}")

# Base dimensions (mm) — these get multiplied by PIECE_SCALE
_P = PIECE_SCALE
BOARD_LENGTH = 190.0*_P;  BOARD_WIDTH = 66.0*_P;  BOARD_THICK = 10.2*_P
MAST_HEIGHT  = 170.0*_P;  MAST_CHORD  = 24.0*_P;  MAST_NACA = '0013'
FUSE_LENGTH  = 123.0*_P

MAST_PLATE_THICK = 2.0*_P;  MAST_PLATE_LENGTH = 18.0*_P;  MAST_PLATE_HEIGHT = 10.0*_P
FW_TAB_THICKNESS = 2.5*_P;  FW_TAB_LENGTH = 14.0*_P
STAB_TAB_THICKNESS = 2.0*_P; STAB_TAB_LENGTH = 10.0*_P

FW_SPAN = 200.0*_P;  FW_ROOT_CHORD = 25.2*_P;  FW_TIP_CHORD = 12.6*_P
FW_SWEEP = 10.0*_P;  FW_DIHEDRAL = 3.0*_P;     FW_NACA = '2412'
FW_TWIST = -2.0   # degrees — not scaled (it's an angle)

STAB_SPAN = 45.0*_P;  STAB_ROOT_CHORD = 12.6*_P;  STAB_TIP_CHORD = 7.0*_P
STAB_SWEEP = 2.5*_P;  STAB_NACA = '0024'

SLOT_CLEARANCE = 0.3*_P;  SCREW_DIAM = 1.6*_P;  FOIL_PTS = 80


# ============================================================
# CUBIC B-SPLINE INTERPOLATION
# ============================================================

def cubic_bspline_prep(knots, values):
    """Compute natural cubic spline coefficients.

    Given n data points (knots[i], values[i]), returns coefficients
    (a, b, c, d) for each segment such that:
        S_i(x) = a[i] + b[i](x-x_i) + c[i](x-x_i)^2 + d[i](x-x_i)^3

    Natural spline: S''(x_0) = S''(x_n) = 0
    This gives C2 continuity across all knots.
    """
    n = len(knots) - 1
    if n < 1:
        return [(values[0], 0.0, 0.0, 0.0)]

    a = list(values)
    h = [knots[i+1] - knots[i] for i in range(n)]

    # Solve tridiagonal system for c coefficients
    # Natural spline: c[0] = c[n] = 0
    alpha = [0.0] * (n + 1)
    for i in range(1, n):
        if h[i-1] == 0 or h[i] == 0:
            alpha[i] = 0
        else:
            alpha[i] = (3.0/h[i] * (a[i+1] - a[i])
                        - 3.0/h[i-1] * (a[i] - a[i-1]))

    # Forward sweep
    l = [1.0] + [0.0] * n
    mu = [0.0] * (n + 1)
    z = [0.0] * (n + 1)

    for i in range(1, n):
        l[i] = 2.0 * (knots[i+1] - knots[i-1]) - h[i-1] * mu[i-1]
        if abs(l[i]) < 1e-12:
            l[i] = 1e-12
        mu[i] = h[i] / l[i]
        z[i] = (alpha[i] - h[i-1] * z[i-1]) / l[i]

    l[n] = 1.0
    z[n] = 0.0

    # Back substitution
    c = [0.0] * (n + 1)
    b = [0.0] * n
    d = [0.0] * n

    for j in range(n - 1, -1, -1):
        c[j] = z[j] - mu[j] * c[j+1]
        if abs(h[j]) < 1e-12:
            b[j] = 0.0
            d[j] = 0.0
        else:
            b[j] = (a[j+1] - a[j]) / h[j] - h[j] * (c[j+1] + 2.0*c[j]) / 3.0
            d[j] = (c[j+1] - c[j]) / (3.0 * h[j])

    return [(a[i], b[i], c[i], d[i]) for i in range(n)]


def cubic_bspline_eval(knots, coeffs, x):
    """Evaluate the cubic spline at point x."""
    n = len(coeffs)
    # Clamp to range
    x = max(knots[0], min(knots[-1], x))

    # Find segment
    seg = 0
    for i in range(n):
        if x >= knots[i]:
            seg = i
    if seg >= n:
        seg = n - 1

    dx = x - knots[seg]
    a, b, c, d = coeffs[seg]
    return a + b*dx + c*dx*dx + d*dx*dx*dx


class SplineDistribution:
    """Defines a spanwise parameter distribution using cubic splines.

    Control points are given at specific eta values (0=root, 1=tip).
    Evaluation at any eta returns a smoothly interpolated value.
    """
    def __init__(self, control_points):
        """control_points: list of (eta, value) tuples, sorted by eta."""
        self.knots = [cp[0] for cp in control_points]
        self.values = [cp[1] for cp in control_points]
        self.coeffs = cubic_bspline_prep(self.knots, self.values)

    def __call__(self, eta):
        return cubic_bspline_eval(self.knots, self.coeffs, eta)


# ============================================================
# AIRFOIL PROFILES
# ============================================================

def naca_4digit(code='0012', n=80, te_thickness=0.002):
    """NACA 4-digit airfoil with optional finite trailing edge thickness.

    te_thickness: fraction of chord for TE gap (0 = sharp, 0.002 = realistic)
    Returns closed loop of (x, y) with x in [0,1].
    """
    m = int(code[0]) / 100.0
    p = int(code[1]) / 10.0
    t = int(code[2:]) / 100.0

    def yt(x):
        # Modified coefficient for finite TE
        a4 = -0.1015 + te_thickness / (5.0 * t) if t > 0 else -0.1015
        return 5.0 * t * (
            0.2969 * math.sqrt(max(x, 0))
            - 0.1260 * x - 0.3516 * x**2
            + 0.2843 * x**3 + a4 * x**4
        )

    def camber(x):
        if p == 0 or m == 0:
            return 0.0, 0.0
        if x < p:
            yc = m / p**2 * (2*p*x - x**2)
            dyc = 2*m / p**2 * (p - x)
        else:
            yc = m / (1-p)**2 * ((1-2*p) + 2*p*x - x**2)
            dyc = 2*m / (1-p)**2 * (p - x)
        return yc, dyc

    upper, lower = [], []
    for i in range(n + 1):
        beta = math.pi * i / n
        x = 0.5 * (1 - math.cos(beta))
        y = yt(x)
        yc, dyc = camber(x)
        th = math.atan(dyc) if (p > 0 and m > 0) else 0.0
        upper.append((x - y*math.sin(th), yc + y*math.cos(th)))
        lower.append((x + y*math.sin(th), yc - y*math.cos(th)))

    return list(reversed(upper)) + lower[1:]


# ============================================================
# B-SPLINE WING LOFT
# ============================================================

def loft_bspline_wing(name, root_chord, tip_chord, span,
                      sweep, dihedral_deg, twist_deg,
                      naca_code, n_span_half=40):
    """Loft a wing using cubic B-spline spanwise interpolation.

    Every spanwise parameter (chord, sweep offset, dihedral offset,
    twist angle, thickness) is defined by control points and evaluated
    through a cubic spline, giving C2 continuous surface variation.

    Tips are closed with hemispherical caps (8 concentric rings
    shrinking via cos() to a center point), NOT by collapsing the
    airfoil to a spike.

    Orientation: span=Y, chord=X (LE toward -X), thickness=Z.
    """
    profile = naca_4digit(naca_code, FOIL_PTS, te_thickness=0.002)
    n_prof = len(profile)
    half_span = span / 2.0
    dihedral = math.radians(dihedral_deg)
    twist_tip = math.radians(twist_deg)

    # ── Spanwise distributions via cubic splines ──
    # Each defined by control points at (eta, value) where eta ∈ [0, 1]
    # eta=0 is root, eta=1 is tip

    # Chord distribution: smooth taper, narrows toward tip
    # The hemispherical cap closes the last section smoothly
    chord_spline = SplineDistribution([
        (0.00, root_chord),
        (0.10, root_chord * 0.98),
        (0.30, root_chord * 0.90 + tip_chord * 0.10),
        (0.60, root_chord * 0.50 + tip_chord * 0.50),
        (0.80, tip_chord * 0.95),
        (0.90, tip_chord * 0.70),
        (0.95, tip_chord * 0.45),
        (1.00, tip_chord * 0.25),         # small but not zero — cap finishes it
    ])

    # Sweep distribution: smooth increase to tip
    sweep_spline = SplineDistribution([
        (0.00, 0.0),
        (0.30, sweep * 0.08),
        (0.60, sweep * 0.35),
        (0.80, sweep * 0.65),
        (1.00, sweep),
    ])

    # Twist distribution: linear washout is fine, but spline makes it smooth
    twist_spline = SplineDistribution([
        (0.00, 0.0),
        (0.30, twist_tip * 0.05),
        (0.60, twist_tip * 0.25),
        (0.80, twist_tip * 0.55),
        (1.00, twist_tip),
    ])

    # Thickness multiplier: full to 60% span, then tapers toward tip
    thick_spline = SplineDistribution([
        (0.00, 1.0),
        (0.50, 1.0),
        (0.70, 0.95),
        (0.80, 0.80),
        (0.90, 0.55),
        (0.95, 0.35),
        (1.00, 0.20),                # thin at tip — cap rounds it off
    ])

    # ── NO tip collapse — we keep real airfoil sections to the very tip ──
    # The tips are closed with smooth rounded caps added after the loft.

    # ── Build sections ──
    # Cosine spacing from 0 to 1 (denser at tips)
    etas_half = []
    for j in range(n_span_half + 1):
        eta_cos = 1.0 - math.cos(math.pi / 2 * j / n_span_half)
        etas_half.append(min(eta_cos, 1.0))

    # Full span: negative side (mirrored) + positive side
    etas_full = []
    for eta in reversed(etas_half[1:]):
        etas_full.append(-eta)
    etas_full.append(0.0)
    for eta in etas_half[1:]:
        etas_full.append(eta)

    verts = []
    for eta_signed in etas_full:
        eta = abs(eta_signed)
        y = eta_signed * half_span

        # Evaluate splines — no collapse, real chord everywhere
        c = chord_spline(eta)
        sw = sweep_spline(eta)
        tw = twist_spline(eta)
        th_mult = thick_spline(eta)
        z_off = abs(y) * math.tan(dihedral)

        c = max(c, 0.5)  # safety floor

        cos_tw = math.cos(tw)
        sin_tw = math.sin(tw)
        for px, py in profile:
            x_local = px * c - c * 0.30
            z_local = py * c * th_mult
            x_rot = x_local * cos_tw + z_local * sin_tw
            z_rot = -x_local * sin_tw + z_local * cos_tw
            verts.append((sw + x_rot, y, z_rot + z_off))

    n_sections = len(etas_full)
    faces = []
    for j in range(n_sections - 1):
        for i in range(n_prof):
            i_next = (i + 1) % n_prof
            v0 = j*n_prof + i
            v1 = j*n_prof + i_next
            v2 = (j+1)*n_prof + i_next
            v3 = (j+1)*n_prof + i
            faces.append((v0, v1, v2, v3))

    # ── Rounded tip caps ──
    # For each tip (first and last section), add a hemisphere-like cap.
    # We shrink the profile inward over several rings, ending at a center point.
    cap_rings = 8
    for tip_side in [0, n_sections - 1]:
        base_idx = tip_side * n_prof
        y_tip = verts[base_idx][1]
        sign = -1.0 if tip_side == 0 else 1.0

        # Compute center of the tip section
        cx = sum(verts[base_idx + i][0] for i in range(n_prof)) / n_prof
        cz = sum(verts[base_idx + i][2] for i in range(n_prof)) / n_prof

        prev_ring_start = base_idx
        for ring in range(1, cap_rings + 1):
            t = ring / cap_rings  # 0→1

            # Elliptical profile: radius shrinks, Y extends outward
            # like the top of a sphere
            r_factor = math.cos(t * math.pi / 2)  # 1→0
            y_offset = math.sin(t * math.pi / 2)  # 0→1

            # Maximum outward extent based on the tip section's thickness
            max_extent = 0
            for i in range(n_prof):
                dx = verts[base_idx + i][0] - cx
                dz = verts[base_idx + i][2] - cz
                max_extent = max(max_extent, math.sqrt(dx*dx + dz*dz))

            y_new = y_tip + sign * y_offset * max_extent * 0.5

            if ring < cap_rings:
                # Add a ring of vertices, shrunk toward center
                ring_start = len(verts)
                for i in range(n_prof):
                    ox = verts[base_idx + i][0]
                    oz = verts[base_idx + i][2]
                    nx = cx + (ox - cx) * r_factor
                    nz = cz + (oz - cz) * r_factor
                    verts.append((nx, y_new, nz))

                # Connect to previous ring
                for i in range(n_prof):
                    i_next = (i + 1) % n_prof
                    if ring == 1:
                        pv0 = prev_ring_start + i
                        pv1 = prev_ring_start + i_next
                    else:
                        pv0 = prev_ring_start + i
                        pv1 = prev_ring_start + i_next
                    cv0 = ring_start + i
                    cv1 = ring_start + i_next

                    if tip_side == 0:
                        faces.append((pv1, pv0, cv0, cv1))
                    else:
                        faces.append((pv0, pv1, cv1, cv0))

                prev_ring_start = ring_start
            else:
                # Final ring: fan to center point
                center_idx = len(verts)
                verts.append((cx, y_new, cz))
                for i in range(n_prof):
                    i_next = (i + 1) % n_prof
                    pv0 = prev_ring_start + i
                    pv1 = prev_ring_start + i_next
                    if tip_side == 0:
                        faces.append((pv1, pv0, center_idx))
                    else:
                        faces.append((pv0, pv1, center_idx))

    mesh = bpy.data.meshes.new(name)
    mesh.from_pydata(verts, [], faces)
    mesh.update()

    obj = bpy.data.objects.new(name, mesh)
    bpy.context.collection.objects.link(obj)
    bpy.context.view_layer.objects.active = obj
    obj.select_set(True)

    # Clean mesh
    bpy.ops.object.mode_set(mode='EDIT')
    bpy.ops.mesh.select_all(action='SELECT')
    bpy.ops.mesh.remove_doubles(threshold=0.03)
    bpy.ops.mesh.normals_make_consistent(inside=False)
    bpy.ops.object.mode_set(mode='OBJECT')

    # One level of Catmull-Clark to further smooth any remaining faceting
    sub = obj.modifiers.new("Subdiv", type='SUBSURF')
    sub.levels = 1
    sub.render_levels = 1

    bpy.ops.object.shade_smooth()
    return obj


# ============================================================
# UTILITIES (same as v5)
# ============================================================

def import_and_scale_stl(filepath, scale, name):
    if not filepath or not os.path.exists(filepath):
        return None
    print(f"  Importing: {filepath}")
    try: bpy.ops.wm.stl_import(filepath=filepath)
    except:
        try: bpy.ops.import_mesh.stl(filepath=filepath)
        except Exception as e:
            print(f"  STL import failed: {e}"); return None
    obj = bpy.context.active_object
    if not obj:
        objs = [o for o in bpy.data.objects if o.type == 'MESH']
        obj = objs[-1] if objs else None
    if not obj: return None
    obj.name = name
    bpy.context.view_layer.objects.active = obj
    obj.select_set(True)
    bpy.ops.object.origin_set(type='ORIGIN_GEOMETRY', center='BOUNDS')
    obj.scale = (scale, scale, scale)
    bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
    obj.location = (0,0,0)
    bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
    bpy.ops.object.shade_smooth()
    return obj

def import_and_scale_mesh(filepath, scale, name):
    """Import STL or STEP/STP file based on extension."""
    if not filepath or not os.path.exists(filepath):
        return None
    ext = os.path.splitext(filepath)[1].lower()
    if ext == '.stl':
        return import_and_scale_stl(filepath, scale, name)
    print(f"  Importing: {filepath}")
    imported = False
    for op_name, op_func in [
        ("wm.step_import",    lambda p: bpy.ops.wm.step_import(filepath=p)),
        ("import_scene.step", lambda p: bpy.ops.import_scene.step(filepath=p)),
        ("import_mesh.step",  lambda p: bpy.ops.import_mesh.step(filepath=p)),
    ]:
        if imported: break
        try: op_func(filepath); imported = True; print(f"  via {op_name}")
        except: pass
    if not imported: return None
    objs = [o for o in bpy.data.objects if o.type == 'MESH']
    if not objs: return None
    bpy.ops.object.select_all(action='DESELECT')
    for o in objs: o.select_set(True)
    bpy.context.view_layer.objects.active = objs[0]
    if len(objs) > 1: bpy.ops.object.join()
    obj = bpy.context.active_object
    obj.name = name
    bpy.ops.object.origin_set(type='ORIGIN_GEOMETRY', center='BOUNDS')
    obj.scale = (scale, scale, scale)
    bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
    obj.location = (0,0,0)
    bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
    bpy.ops.object.shade_smooth()
    return obj


# ============================================================
# UTILITY FUNCTIONS
# ============================================================

def clear_scene():
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete()
    for m in bpy.data.meshes:     bpy.data.meshes.remove(m)
    for m in bpy.data.materials:  bpy.data.materials.remove(m)

def apply_all_modifiers(obj):
    bpy.context.view_layer.objects.active = obj
    obj.select_set(True)
    for mod in list(obj.modifiers):
        try: bpy.ops.object.modifier_apply(modifier=mod.name)
        except Exception as e: print(f"    Warn: {mod.name}: {e}")

def export_stl(obj, filename):
    bpy.ops.object.select_all(action='DESELECT')
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    bpy.ops.wm.stl_export(filepath=os.path.join(export_dir, filename))
    print(f"  -> {filename}")

def cut_rounded_slot(obj, width, length, depth, location=(0,0,0)):
    bpy.ops.mesh.primitive_cube_add(size=1)
    cutter = bpy.context.active_object
    cutter.scale = (width/2, length/2, depth/2)
    cutter.location = location
    bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
    bev = cutter.modifiers.new("Bevel", type='BEVEL')
    bev.width = min(width, length, depth) * 0.15
    bev.segments = 4
    bev.limit_method = 'ANGLE'
    bev.angle_limit = math.radians(60)
    bpy.context.view_layer.objects.active = cutter
    bpy.ops.object.modifier_apply(modifier=bev.name)
    bpy.context.view_layer.objects.active = obj
    obj.select_set(True)
    bm = obj.modifiers.new("Slot", type='BOOLEAN')
    bm.object = cutter; bm.operation = 'DIFFERENCE'
    bpy.ops.object.modifier_apply(modifier=bm.name)
    bpy.data.objects.remove(cutter, do_unlink=True)

def add_screw_holes(obj, positions, diameter, depth, axis='x'):
    r = diameter / 2
    for pos in positions:
        rot = (0,0,0)
        if axis == 'x':   rot = (0, math.radians(90), 0)
        elif axis == 'y': rot = (math.radians(90), 0, 0)
        bpy.ops.mesh.primitive_cylinder_add(
            radius=r, depth=depth, vertices=24, location=pos, rotation=rot)
        cyl = bpy.context.active_object
        bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
        bpy.context.view_layer.objects.active = obj
        obj.select_set(True)
        bm = obj.modifiers.new("Screw", type='BOOLEAN')
        bm.object = cyl; bm.operation = 'DIFFERENCE'
        bpy.ops.object.modifier_apply(modifier=bm.name)
        bpy.data.objects.remove(cyl, do_unlink=True)


# ============================================================
# PLACEHOLDER BUILDERS
# ============================================================

def make_placeholder_board(name):
    print("  Placeholder board...")
    bm = bmesh.new()
    n_x = 80   # lengthwise stations
    n_y = 32   # widthwise stations per cross-section
    n_z = 8    # stations around the rounded cross-section profile
    half_w = BOARD_WIDTH / 2
    half_t = BOARD_THICK / 2

    # Planform width profile from Hydroskate reference (normalized)
    profile_stations = [0.0, 0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40,
                        0.45, 0.50, 0.55, 0.60, 0.65, 0.70, 0.75, 0.80, 0.85,
                        0.90, 0.95, 1.00]
    profile_widths =   [0.57, 0.74, 0.83, 0.88, 0.91, 0.93, 0.96, 0.98, 0.99,
                        1.00, 1.00, 1.00, 1.00, 1.00, 0.99, 0.96, 0.94, 0.88,
                        0.82, 0.71, 0.48]

    def interp_profile(t_val):
        if t_val <= 0: return profile_widths[0]
        if t_val >= 1: return profile_widths[-1]
        for k in range(len(profile_stations) - 1):
            if profile_stations[k] <= t_val <= profile_stations[k+1]:
                frac = (t_val - profile_stations[k]) / (profile_stations[k+1] - profile_stations[k])
                return profile_widths[k] + frac * (profile_widths[k+1] - profile_widths[k])
        return profile_widths[-1]

    # Build cross-sections along length
    # Each cross-section is a closed loop: top surface → round edge → bottom → round edge
    # The loop has n_circ points
    n_circ = 2 * n_y + 2 * n_z  # top + right_round + bottom + left_round

    # Nose/tail rounding: reduce thickness near the ends
    def end_thickness_factor(t_val):
        """Reduce thickness near nose and tail for rounded ends."""
        nose_zone = 0.08
        tail_zone = 0.10
        if t_val < nose_zone:
            s = t_val / nose_zone
            return math.sqrt(s)  # smooth ramp from 0 to 1
        elif t_val > (1.0 - tail_zone):
            s = (1.0 - t_val) / tail_zone
            return math.sqrt(s)
        return 1.0

    rings = []
    for i in range(n_x + 1):
        t = i / n_x
        x = (t - 0.5) * BOARD_LENGTH
        pw = half_w * interp_profile(t)
        th_factor = end_thickness_factor(t)
        ht = half_t * th_factor  # local half-thickness

        ring = []
        # Build closed cross-section loop:
        # 1) Top surface: left edge → right edge (n_y+1 points)
        for j in range(n_y + 1):
            s = j / n_y  # 0=left, 1=right
            y = (s - 0.5) * 2 * pw
            yn = abs(s - 0.5) * 2
            # Gentle deck crown
            z = ht * (1.0 - 0.08 * (1 - yn * yn))
            ring.append((x, y, z))

        # 2) Right edge round: top → bottom (n_z-1 interior points)
        for k in range(1, n_z):
            angle = k / n_z * math.pi  # 0=top, π=bottom
            y = pw + ht * 0.3 * math.cos(math.pi - angle)  # slight inward tuck
            z = ht * math.cos(angle)
            ring.append((x, y, z))

        # 3) Bottom surface: right edge → left edge (n_y+1 points, reversed)
        for j in range(n_y, -1, -1):
            s = j / n_y
            y = (s - 0.5) * 2 * pw
            yn = abs(s - 0.5) * 2
            # Flat bottom with slight V
            z = -ht * (1.0 - 0.05 * yn)
            ring.append((x, y, z))

        # 4) Left edge round: bottom → top (n_z-1 interior points)
        for k in range(1, n_z):
            angle = k / n_z * math.pi
            y = -pw - ht * 0.3 * math.cos(math.pi - angle)
            z = -ht * math.cos(angle)
            ring.append((x, y, z))

        for v in ring:
            bm.verts.new(v)
        rings.append(len(ring))

    n_ring = rings[0]  # all rings same size
    bm.verts.ensure_lookup_table()

    # Connect adjacent rings with quads
    for i in range(n_x):
        for j in range(n_ring):
            jn = (j + 1) % n_ring
            v0 = i * n_ring + j
            v1 = i * n_ring + jn
            v2 = (i+1) * n_ring + jn
            v3 = (i+1) * n_ring + j
            bm.faces.new((bm.verts[v0], bm.verts[v1], bm.verts[v2], bm.verts[v3]))

    # Cap nose (first ring)
    center_nose = bm.verts.new(( -0.5 * BOARD_LENGTH, 0, 0))
    bm.verts.ensure_lookup_table()
    for j in range(n_ring):
        jn = (j + 1) % n_ring
        bm.faces.new((center_nose, bm.verts[jn], bm.verts[j]))

    # Cap tail (last ring)
    last_start = n_x * n_ring
    center_tail = bm.verts.new(( 0.5 * BOARD_LENGTH, 0, 0))
    bm.verts.ensure_lookup_table()
    for j in range(n_ring):
        jn = (j + 1) % n_ring
        bm.faces.new((center_tail, bm.verts[last_start + j], bm.verts[last_start + jn]))

    mesh = bpy.data.meshes.new(name)
    bm.to_mesh(mesh); bm.free()
    obj = bpy.data.objects.new(name, mesh)
    bpy.context.collection.objects.link(obj)
    bpy.context.view_layer.objects.active = obj; obj.select_set(True)
    bpy.ops.object.mode_set(mode='EDIT')
    bpy.ops.mesh.select_all(action='SELECT')
    bpy.ops.mesh.normals_make_consistent(inside=False)
    bpy.ops.object.mode_set(mode='OBJECT')
    sub = obj.modifiers.new("Sub", type='SUBSURF'); sub.levels = 2
    bpy.ops.object.modifier_apply(modifier=sub.name)
    bpy.ops.object.shade_smooth()
    return obj

def make_placeholder_mast(name):
    print("  Placeholder mast...")
    profile = naca_4digit(MAST_NACA, FOIL_PTS)
    n_prof = len(profile)

    # Mastfoot flares from NACA to a rounded-square mounting surface
    # Size: just enough to hold 4 screw holes outside the mast profile
    foot_size = MAST_CHORD * 1.2  # only 20% wider than chord (was 2x)

    # Build mast body + steep fairing as one continuous loft
    n_h = 30
    n_fair = 6
    fair_start = 0.97  # steep: fairing in top 3% of mast

    all_verts = []
    all_faces = []
    total_rings = n_h + n_fair

    for j in range(total_rings):
        if j < n_h:
            t = j / (n_h - 1) * fair_start
            z = t * MAST_HEIGHT
            c = MAST_CHORD * (1 - 0.1 * t)
            blend = 0.0
        else:
            fi = j - n_h
            ft = fi / (n_fair - 1)
            t = fair_start + ft * (1.0 - fair_start)
            z = t * MAST_HEIGHT
            c = MAST_CHORD * (1 - 0.1 * t)
            blend = ft  # linear for steep transition

        for pi in range(n_prof):
            px, py = profile[pi]
            mx = (px - 0.5) * c
            my = py * c

            # Foot outline: superellipse (rounded square)
            angle = 2 * math.pi * pi / n_prof
            ca = math.cos(angle)
            sa = math.sin(angle)
            n_exp = 4.0
            r_sq = foot_size / 2
            denom = (abs(ca)**n_exp + abs(sa)**n_exp) ** (1.0 / n_exp)
            ex = r_sq * ca / denom if denom > 0.001 else 0
            ey = r_sq * sa / denom if denom > 0.001 else 0

            x_f = mx + blend * (ex - mx)
            y_f = my + blend * (ey - my)
            all_verts.append((x_f, y_f, z))

    # Connect rings
    for j in range(total_rings - 1):
        for i in range(n_prof):
            i_next = (i + 1) % n_prof
            v0 = j * n_prof + i
            v1 = j * n_prof + i_next
            v2 = (j + 1) * n_prof + i_next
            v3 = (j + 1) * n_prof + i
            all_faces.append((v0, v1, v2, v3))

    # Caps
    all_faces.append(tuple(reversed(range(n_prof))))
    last_start = (total_rings - 1) * n_prof
    all_faces.append(tuple(range(last_start, last_start + n_prof)))

    mesh = bpy.data.meshes.new(name)
    mesh.from_pydata(all_verts, [], all_faces); mesh.update()
    obj = bpy.data.objects.new(name, mesh)
    bpy.context.collection.objects.link(obj)
    bpy.context.view_layer.objects.active = obj; obj.select_set(True)
    bpy.ops.object.mode_set(mode='EDIT')
    bpy.ops.mesh.select_all(action='SELECT')
    bpy.ops.mesh.normals_make_consistent(inside=False)
    bpy.ops.object.mode_set(mode='OBJECT')

    # 4 screw holes through the mastfoot top
    screw_inset = foot_size * 0.30
    for sx, sy in [(-1,-1), (-1,1), (1,-1), (1,1)]:
        bpy.ops.mesh.primitive_cylinder_add(
            radius=SCREW_DIAM / 2, depth=MAST_HEIGHT * 0.10,
            vertices=24,
            location=(sx * screw_inset, sy * screw_inset, MAST_HEIGHT))
        cyl = bpy.context.active_object
        bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
        bpy.context.view_layer.objects.active = obj
        obj.select_set(True)
        mod = obj.modifiers.new("ScrewHole", type='BOOLEAN')
        mod.object = cyl; mod.operation = 'DIFFERENCE'
        bpy.ops.object.modifier_apply(modifier=mod.name)
        bpy.data.objects.remove(cyl, do_unlink=True)

    bpy.ops.object.shade_smooth()
    return obj



def make_placeholder_fuselage(name):
    print("  Placeholder fuselage...")

    fw_r = 4.76 * _P; fh = 6.0 * _P  # fw_r = half-width, fh = body height
    n_circ = 32

    # Wing junction parameters
    fy = FUSE_LENGTH * 0.11   # front wing Y
    sy = FUSE_LENGTH * 0.88   # stabilizer Y

    def naca_lower_z_at(code, chord, fuse_y, y_center):
        """Get wing lower-surface Z at a Y position along the fuselage.
        Returns (z_lower, half_thickness) or (None, None) if outside chord."""
        m_v = int(code[0])/100; p_c = int(code[1])/10; t_v = int(code[2:])/100
        px = (fuse_y - y_center) / chord + 0.30
        if px < 0.001 or px > 0.999:
            return None, None
        yt = 5*t_v*(0.2969*math.sqrt(px)-0.126*px-0.3516*px**2+0.2843*px**3-0.1015*px**4)
        if p_c > 0 and m_v > 0:
            if px < p_c: yc = m_v/p_c**2*(2*p_c*px - px**2)
            else:         yc = m_v/(1-p_c)**2*((1-2*p_c)+2*p_c*px - px**2)
        else:
            yc = 0
        z_lower = fh + (yc - yt) * chord   # wing lower surface Z in fuse coords
        half_t = yt * chord                  # half-thickness at this chord station
        return z_lower, half_t

    fw_blend_y = 5.0 * _P   # Y distance over which blend fades (along fuselage)
    st_blend_y = 4.0 * _P

    # Dense Y stations near junctions
    y_stations = set()
    for i in range(81):
        y_stations.add(i / 80 * FUSE_LENGTH)
    for yc in [fy, sy]:
        for dy in [-8, -6, -5, -4, -3, -2.5, -2, -1.5, -1, -0.5, 0,
                    0.5, 1, 1.5, 2, 2.5, 3, 4, 5, 6, 8]:
            yv = yc + dy * _P
            if 0 <= yv <= FUSE_LENGTH:
                y_stations.add(yv)
    y_stations = sorted(y_stations)
    n_len = len(y_stations)

    bm = bmesh.new()

    for i, y in enumerate(y_stations):
        t = y / FUSE_LENGTH
        if t < 0.15:
            s = t / 0.15  # 0→1 over nose region
            rf = 0.35 + 0.65 * math.sin(s * math.pi / 2)  # starts at 35% radius, not zero
        elif t < 0.55: rf = 1.0
        else:          rf = 1.0 - (t-0.55)/0.45*0.65
        rx = max(fw_r*rf, 0.3); rz = max(fh/2*rf, 0.3)

        # Blend factors (0 = ellipse, 1 = flat top matching wing)
        fw_d = abs(y - fy)
        st_d = abs(y - sy)
        fw_t = max(0, 1.0 - fw_d / fw_blend_y) if fw_d < fw_blend_y else 0
        st_t = max(0, 1.0 - st_d / st_blend_y) if st_d < st_blend_y else 0
        # Smooth with cos² for C1 continuity
        fw_t = (math.sin(fw_t * math.pi / 2)) ** 2
        st_t = (math.sin(st_t * math.pi / 2)) ** 2

        # Get wing Z at this Y station
        fw_z, fw_ht = naca_lower_z_at(FW_NACA, FW_ROOT_CHORD, y, fy)
        st_z, st_ht = naca_lower_z_at(STAB_NACA, STAB_ROOT_CHORD, y, sy)

        # Pick the active wing (the one with higher blend factor)
        blend = 0; wing_z_lower = None
        if fw_t > st_t and fw_z is not None:
            blend = fw_t; wing_z_lower = fw_z
        elif st_t > 0 and st_z is not None:
            blend = st_t; wing_z_lower = st_z

        for j in range(n_circ):
            th = 2 * math.pi * j / n_circ
            x_ell = rx * math.cos(th)
            z_ell = rz * math.sin(th) + fh/2

            if blend > 0 and wing_z_lower is not None and math.sin(th) > 0:
                # Upper half of cross-section: flatten toward wing lower surface
                # The target: at blend=1, the top of the ellipse (Z=fh) should
                # match the wing lower surface Z. The sides stay elliptical.
                #
                # sin(th) ranges from 0 (side) to 1 (top)
                # Only flatten the portion above the wing surface
                sin_th = math.sin(th)

                # Target Z: the wing lower surface (flat across X at this Y)
                target_z = wing_z_lower

                # How much of this vertex should be affected:
                # fully at the top (sin_th=1), fading to zero at the sides (sin_th=0)
                vert_blend = blend * sin_th

                # Clamp: only move vertices that are above the target
                if z_ell > target_z:
                    z_ell = z_ell - vert_blend * (z_ell - target_z)

            bm.verts.new((x_ell, y, z_ell))

    bm.verts.ensure_lookup_table()

    for i in range(n_len - 1):
        for j in range(n_circ):
            jn = (j+1) % n_circ
            bm.faces.new((bm.verts[i*n_circ+j], bm.verts[i*n_circ+jn],
                           bm.verts[(i+1)*n_circ+jn], bm.verts[(i+1)*n_circ+j]))

    # Nose cap
    nv = bm.verts.new((0, -0.5*_P, fh/2)); bm.verts.ensure_lookup_table()
    for j in range(n_circ):
        bm.faces.new((nv, bm.verts[(j+1)%n_circ], bm.verts[j]))
    # Tail cap
    tb = (n_len - 1) * n_circ
    tv = bm.verts.new((0, FUSE_LENGTH+0.5*_P, fh/2)); bm.verts.ensure_lookup_table()
    for j in range(n_circ):
        bm.faces.new((tv, bm.verts[tb+j], bm.verts[tb+(j+1)%n_circ]))

    mesh = bpy.data.meshes.new(name)
    bm.to_mesh(mesh); bm.free()
    obj = bpy.data.objects.new(name, mesh)
    bpy.context.collection.objects.link(obj)
    bpy.context.view_layer.objects.active = obj; obj.select_set(True)
    bpy.ops.object.mode_set(mode='EDIT')
    bpy.ops.mesh.select_all(action='SELECT')
    bpy.ops.mesh.normals_make_consistent(inside=False)
    bpy.ops.object.mode_set(mode='OBJECT')

    # ── NACA-shaped mast pocket (50% depth from top) ──
    py_pos = FUSE_LENGTH * 0.44
    profile = naca_4digit(MAST_NACA, 48)
    cl = SLOT_CLEARANCE
    mast_c = MAST_CHORD + cl * 2

    cutter_verts = []
    cutter_faces = []
    n_prof = len(profile)
    z_bot = fh * 0.5
    z_top = fh + 2.0 * _P

    for zi, z in enumerate([z_bot, z_top]):
        for px, pz in profile:
            fuse_y = (px - 0.5) * mast_c + py_pos
            fuse_x = pz * mast_c
            cutter_verts.append((fuse_x, fuse_y, z))

    for i in range(n_prof):
        i_next = (i + 1) % n_prof
        cutter_faces.append((i, i_next, n_prof + i_next, n_prof + i))
    cutter_faces.append(tuple(range(n_prof - 1, -1, -1)))
    cutter_faces.append(tuple(range(n_prof, 2 * n_prof)))

    cutter_mesh = bpy.data.meshes.new("MastCutter")
    cutter_mesh.from_pydata(cutter_verts, [], cutter_faces)
    cutter_mesh.update()
    cutter_obj = bpy.data.objects.new("MastCutter", cutter_mesh)
    bpy.context.collection.objects.link(cutter_obj)
    bpy.context.view_layer.objects.active = cutter_obj; cutter_obj.select_set(True)
    bpy.ops.object.mode_set(mode='EDIT')
    bpy.ops.mesh.select_all(action='SELECT')
    bpy.ops.mesh.normals_make_consistent(inside=False)
    bpy.ops.object.mode_set(mode='OBJECT')
    bpy.context.view_layer.objects.active = obj; obj.select_set(True)
    bool_mod = obj.modifiers.new("MastHole", type='BOOLEAN')
    bool_mod.object = cutter_obj; bool_mod.operation = 'DIFFERENCE'
    bpy.ops.object.modifier_apply(modifier=bool_mod.name)
    bpy.data.objects.remove(cutter_obj, do_unlink=True)

    # Subdivision for smoothness
    sub = obj.modifiers.new("S", type='SUBSURF'); sub.levels = 1
    bpy.ops.object.modifier_apply(modifier=sub.name)

    # ── Screw holes (through full fuselage) ──

    # Front wing screws: 3 vertical through full fuselage
    add_screw_holes(obj,
        [(0, fy - 2.0*_P, fh/2),
         (0, fy + 3.0*_P, fh/2),
         (0, fy + 8.0*_P, fh/2)],
        SCREW_DIAM, fh + 4*_P, 'z')

    # Stab screws: 2 vertical through full fuselage, 30% apart, 25% shifted
    st_shift = STAB_ROOT_CHORD * 0.25
    st_half_sp = STAB_ROOT_CHORD * 0.15
    add_screw_holes(obj,
        [(0, sy - st_half_sp + st_shift, fh/2),
         (0, sy + st_half_sp + st_shift, fh/2)],
        SCREW_DIAM, fh + 4*_P, 'z')

    # Mast screws: 2 vertical from bottom into mast pocket floor
    mast_screw_z = fh * 0.25
    mast_screw_depth = fh * 0.5 + 2*_P
    add_screw_holes(obj,
        [(0, py_pos - 7.0*_P, mast_screw_z),
         (0, py_pos - 1.0*_P, mast_screw_z)],
        SCREW_DIAM, mast_screw_depth, 'z')

    bpy.ops.object.shade_smooth()
    return obj


# ============================================================
# WING BUILDERS
# ============================================================

def make_front_wing(name):
    obj = loft_bspline_wing(
        name, FW_ROOT_CHORD, FW_TIP_CHORD, FW_SPAN,
        FW_SWEEP, FW_DIHEDRAL, FW_TWIST, FW_NACA,
        n_span_half=40)
    apply_all_modifiers(obj)

    # 3 vertical screw holes at root, spaced along X (chord = behind each other)
    # Shifted toward trailing edge and further apart
    fw_thick = FW_ROOT_CHORD * 0.12
    add_screw_holes(obj,
        [(-2.0*_P, 0, 0), (3.0*_P, 0, 0), (8.0*_P, 0, 0)],
        SCREW_DIAM, fw_thick + 6*_P, 'z')
    return obj

def make_stabilizer(name):
    obj = loft_bspline_wing(
        name, STAB_ROOT_CHORD, STAB_TIP_CHORD, STAB_SPAN,
        STAB_SWEEP, 0, 0, STAB_NACA,
        n_span_half=28)
    apply_all_modifiers(obj)

    # 2 vertical screw holes at root, 30% chord apart, shifted 25% toward TE
    stab_thick = STAB_ROOT_CHORD * 0.24
    st_shift = STAB_ROOT_CHORD * 0.25
    st_half_sp = STAB_ROOT_CHORD * 0.15  # half of 30% spacing
    add_screw_holes(obj,
        [(-st_half_sp + st_shift, 0, 0), (st_half_sp + st_shift, 0, 0)],
        SCREW_DIAM, stab_thick + 6*_P, 'z')
    return obj


# ============================================================
# BUILD
# ============================================================

print("\n" + "="*60)
print("  PUMPFOIL v6 — B-spline wings")
print("="*60 + "\n")

# 1. Board
print("[1/5] Board...")
clear_scene()
board = import_and_scale_mesh(BOARD_FILE, SCALE, "Board") if BOARD_FILE else None
if not board: board = make_placeholder_board("Board")

# Mast foot: flat plate sits on board bottom, secured by 4 screws
mast_board_x = BOARD_LENGTH * 0.30  # 80% from nose
# 4 screw holes matching the mast foot
foot_size = MAST_CHORD * 1.2
screw_inset = foot_size * 0.30
add_screw_holes(board,
    [(mast_board_x - screw_inset, -screw_inset, 0),
     (mast_board_x - screw_inset,  screw_inset, 0),
     (mast_board_x + screw_inset, -screw_inset, 0),
     (mast_board_x + screw_inset,  screw_inset, 0)],
    SCREW_DIAM, BOARD_THICK + 4*_P, 'z')

export_stl(board, "1_board.stl")

# 2. Mast
print("[2/5] Mast...")
clear_scene()
mast = import_and_scale_mesh(MAST_FILE, SCALE, "Mast") if MAST_FILE else None
if not mast:
    mast = make_placeholder_mast("Mast")
    screw_z = 1.5 * _P
    add_screw_holes(mast,
        [(-7.0*_P, 0, screw_z),
         (-1.0*_P, 0, screw_z)],
        SCREW_DIAM, 5.0*_P, 'z')
export_stl(mast, "2_mast.stl")

# 3. Fuselage
print("[3/5] Fuselage...")
clear_scene()
fuse = import_and_scale_mesh(FUSE_FILE, SCALE, "Fuselage") if FUSE_FILE else None
if not fuse: fuse = make_placeholder_fuselage("Fuselage")
export_stl(fuse, "3_fuselage.stl")

# 4. Front wing
print("[4/5] Front wing (B-spline)...")
clear_scene()
fw = make_front_wing("FrontWing")
export_stl(fw, "4_frontwing.stl")

# 5. Stabilizer
print("[5/5] Stabilizer (B-spline)...")
clear_scene()
stab = make_stabilizer("Stabilizer")
export_stl(stab, "5_stabilizer.stl")

print(f"\n{'='*60}")
print(f"  Done! → {export_dir}")
print(f"{'='*60}")
print(f"""
  1_board.stl        {BOARD_LENGTH:.0f} x {BOARD_WIDTH:.0f} x {BOARD_THICK:.1f}mm {'('+BOARD_FILE+')' if BOARD_FILE else '(generated)'}
  2_mast.stl         {MAST_HEIGHT:.0f}mm tall, {MAST_CHORD:.0f}mm chord, NACA {MAST_NACA} {'('+MAST_FILE+')' if MAST_FILE else '(generated)'}
  3_fuselage.stl     {FUSE_LENGTH:.1f}mm long, {4.76*2*_P:.1f}mm wide {'('+FUSE_FILE+')' if FUSE_FILE else '(generated)'}
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
