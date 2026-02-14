import bpy
import bmesh
import math
import os
import sys

# ============================================================
# FINGERFOIL — Shared constants and utilities
# ============================================================

export_dir = os.path.expanduser("~/Desktop/fingerfoil")
os.makedirs(export_dir, exist_ok=True)

SCALE = 0.20

# ── Command-line options ──
PIECE_SCALE = 0.50
BOARD_FILE = None
MAST_FILE  = None
FUSE_FILE  = None
COMBINED   = False

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
        elif arg == '-combined':
            COMBINED = True; i += 1
        else:
            i += 1

print(f"  PIECE_SCALE = {PIECE_SCALE}")
if BOARD_FILE: print(f"  Board file:  {BOARD_FILE}")
if MAST_FILE:  print(f"  Mast file:   {MAST_FILE}")
if FUSE_FILE:  print(f"  Fuse file:   {FUSE_FILE}")

# ── Base dimensions (mm) ──
PS = PIECE_SCALE
BOARD_LENGTH = 190.0*PS;  BOARD_WIDTH = 66.0*PS;  BOARD_THICK = 10.2*PS
MAST_HEIGHT  = 170.0*PS;  MAST_CHORD  = 24.0*PS;  MAST_NACA = '0013'
FUSE_LENGTH  = 123.0*PS

MAST_PLATE_THICK = 2.0*PS;  MAST_PLATE_LENGTH = 18.0*PS;  MAST_PLATE_HEIGHT = 10.0*PS
FW_TAB_THICKNESS = 2.5*PS;  FW_TAB_LENGTH = 14.0*PS
STAB_TAB_THICKNESS = 2.0*PS; STAB_TAB_LENGTH = 10.0*PS

FW_SPAN = 200.0*PS;  FW_ROOT_CHORD = 25.2*PS;  FW_TIP_CHORD = 12.6*PS
FW_SWEEP = 10.0*PS;  FW_DIHEDRAL = 3.0;        FW_NACA = '2412'
FW_TWIST = -2.0

STAB_SPAN = 45.0*PS;  STAB_ROOT_CHORD = 12.6*PS;  STAB_TIP_CHORD = 7.0*PS
STAB_SWEEP = 2.5*PS;  STAB_NACA = '0024'

SLOT_CLEARANCE = 0.3*PS;  SCREW_DIAM = 1.6*PS;  FOIL_PTS = 80



# ============================================================
# CUBIC B-SPLINE INTERPOLATION
# ============================================================

def cubic_bspline_prep(knots, values):
    """Compute natural cubic spline coefficients."""
    n = len(knots) - 1
    if n < 1:
        return [(values[0], 0.0, 0.0, 0.0)]

    a = list(values)
    h = [knots[i+1] - knots[i] for i in range(n)]

    alpha = [0.0] * (n + 1)
    for i in range(1, n):
        if h[i-1] == 0 or h[i] == 0:
            alpha[i] = 0
        else:
            alpha[i] = (3.0/h[i] * (a[i+1] - a[i])
                        - 3.0/h[i-1] * (a[i] - a[i-1]))

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
    x = max(knots[0], min(knots[-1], x))

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
    """Defines a spanwise parameter distribution using cubic splines."""
    def __init__(self, control_points):
        self.knots = [cp[0] for cp in control_points]
        self.values = [cp[1] for cp in control_points]
        self.coeffs = cubic_bspline_prep(self.knots, self.values)

    def __call__(self, eta):
        return cubic_bspline_eval(self.knots, self.coeffs, eta)


# ============================================================
# AIRFOIL PROFILES
# ============================================================

def naca_4digit(code='0012', n=80, te_thickness=0.002):
    """NACA 4-digit airfoil with optional finite trailing edge thickness."""
    m = int(code[0]) / 100.0
    p = int(code[1]) / 10.0
    t = int(code[2:]) / 100.0

    def yt(x):
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
    Orientation: span=Y, chord=X, thickness=Z.
    """
    profile = naca_4digit(naca_code, FOIL_PTS, te_thickness=0.002)
    n_prof = len(profile)
    half_span = span / 2.0
    dihedral = math.radians(dihedral_deg)
    twist_tip = math.radians(twist_deg)

    chord_spline = SplineDistribution([
        (0.00, root_chord),
        (0.10, root_chord * 0.98),
        (0.30, root_chord * 0.90 + tip_chord * 0.10),
        (0.60, root_chord * 0.50 + tip_chord * 0.50),
        (0.80, tip_chord * 0.95),
        (0.90, tip_chord * 0.70),
        (0.95, tip_chord * 0.45),
        (1.00, tip_chord * 0.25),
    ])

    sweep_spline = SplineDistribution([
        (0.00, 0.0),
        (0.30, sweep * 0.08),
        (0.60, sweep * 0.35),
        (0.80, sweep * 0.65),
        (1.00, sweep),
    ])

    twist_spline = SplineDistribution([
        (0.00, 0.0),
        (0.30, twist_tip * 0.05),
        (0.60, twist_tip * 0.25),
        (0.80, twist_tip * 0.55),
        (1.00, twist_tip),
    ])

    thick_spline = SplineDistribution([
        (0.00, 1.0),
        (0.50, 1.0),
        (0.70, 0.95),
        (0.80, 0.80),
        (0.90, 0.55),
        (0.95, 0.35),
        (1.00, 0.20),
    ])

    etas_half = []
    for j in range(n_span_half + 1):
        eta_cos = 1.0 - math.cos(math.pi / 2 * j / n_span_half)
        etas_half.append(min(eta_cos, 1.0))

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

        c = chord_spline(eta)
        sw = sweep_spline(eta)
        tw = twist_spline(eta)
        th_mult = thick_spline(eta)
        z_off = abs(y) * math.tan(dihedral)

        c = max(c, 0.5)

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

    # Rounded tip caps
    cap_rings = 8
    for tip_side in [0, n_sections - 1]:
        base_idx = tip_side * n_prof
        y_tip = verts[base_idx][1]
        sign = -1.0 if tip_side == 0 else 1.0

        cx = sum(verts[base_idx + i][0] for i in range(n_prof)) / n_prof
        cz = sum(verts[base_idx + i][2] for i in range(n_prof)) / n_prof

        prev_ring_start = base_idx
        for ring in range(1, cap_rings + 1):
            t = ring / cap_rings
            r_factor = math.cos(t * math.pi / 2)
            y_offset = math.sin(t * math.pi / 2)

            max_extent = 0
            for i in range(n_prof):
                dx = verts[base_idx + i][0] - cx
                dz = verts[base_idx + i][2] - cz
                max_extent = max(max_extent, math.sqrt(dx*dx + dz*dz))

            y_new = y_tip + sign * y_offset * max_extent * 0.5

            if ring < cap_rings:
                ring_start = len(verts)
                for i in range(n_prof):
                    ox = verts[base_idx + i][0]
                    oz = verts[base_idx + i][2]
                    nx = cx + (ox - cx) * r_factor
                    nz = cz + (oz - cz) * r_factor
                    verts.append((nx, y_new, nz))

                for i in range(n_prof):
                    i_next = (i + 1) % n_prof
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

    bpy.ops.object.mode_set(mode='EDIT')
    bpy.ops.mesh.select_all(action='SELECT')
    bpy.ops.mesh.remove_doubles(threshold=0.03)
    bpy.ops.mesh.normals_make_consistent(inside=False)
    bpy.ops.object.mode_set(mode='OBJECT')

    sub = obj.modifiers.new("Subdiv", type='SUBSURF')
    sub.levels = 1
    sub.render_levels = 1

    bpy.ops.object.shade_smooth()
    return obj


# ============================================================
# IMPORT UTILITIES
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
