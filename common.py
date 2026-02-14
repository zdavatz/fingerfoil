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

# ── Base dimensions (mm) ──
PS = PIECE_SCALE
BOARD_LENGTH = 190.0*PS;  BOARD_WIDTH = 66.0*PS;  BOARD_THICK = 10.2*PS
MAST_HEIGHT  = 170.0*PS;  MAST_CHORD  = 24.0*PS;  MAST_NACA = '0013'
FUSE_LENGTH  = 123.0*PS

MAST_PLATE_THICK = 2.0*PS;  MAST_PLATE_LENGTH = 18.0*PS;  MAST_PLATE_HEIGHT = 10.0*PS
FW_TAB_THICKNESS = 2.5*PS;  FW_TAB_LENGTH = 14.0*PS
STAB_TAB_THICKNESS = 2.0*PS; STAB_TAB_LENGTH = 10.0*PS

FW_SPAN = 200.0*PS;  FW_ROOT_CHORD = 25.2*PS;  FW_TIP_CHORD = 12.6*PS
FW_SWEEP = 10.0*PS;  FW_DIHEDRAL = 3.0*PS;     FW_NACA = '2412'
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
    h = [knots[i+1] - knots[i] for i in range(n)]
    alpha = [0.0] * (n + 1)
    for i in range(1, n):
        alpha[i] = (3/h[i]*(values[i+1]-values[i]) - 3/h[i-1]*(values[i]-values[i-1]))
    l = [1.0] + [0.0]*n
    mu = [0.0]*(n+1)
    z = [0.0]*(n+1)
    for i in range(1, n):
        l[i] = 2*(knots[i+1]-knots[i-1]) - h[i-1]*mu[i-1]
        mu[i] = h[i]/l[i]
        z[i] = (alpha[i]-h[i-1]*z[i-1])/l[i]
    l[n] = 1.0
    c = [0.0]*(n+1)
    b = [0.0]*n
    d = [0.0]*n
    for j in range(n-1, -1, -1):
        c[j] = z[j] - mu[j]*c[j+1]
        b[j] = (values[j+1]-values[j])/h[j] - h[j]*(c[j+1]+2*c[j])/3
        d[j] = (c[j+1]-c[j])/(3*h[j])
    return (list(values[:n+1]), b, c[:n+1], d)

def cubic_bspline_eval(knots, coeffs, x):
    """Evaluate cubic spline at x."""
    a, b, c, d = coeffs
    n = len(knots) - 1
    if x <= knots[0]:
        dx = x - knots[0]
        return a[0] + b[0]*dx
    if x >= knots[n]:
        dx = x - knots[n-1]
        return a[n-1] + b[n-1]*dx + c[n-1]*dx*dx + d[n-1]*dx*dx*dx
    lo, hi = 0, n-1
    while lo < hi:
        mid = (lo+hi)//2
        if knots[mid+1] < x: lo = mid+1
        else: hi = mid
    dx = x - knots[lo]
    return a[lo] + b[lo]*dx + c[lo]*dx*dx + d[lo]*dx*dx*dx


# ============================================================
# NACA AIRFOIL
# ============================================================

def naca_4digit(code='0012', n=80, te_thickness=0.002):
    m = int(code[0]) / 100.0
    p = int(code[1]) / 10.0
    t_max = int(code[2:4]) / 100.0
    pts = []
    for i in range(n):
        beta = math.pi * i / (n - 1)
        x = 0.5 * (1 - math.cos(beta))
        yt = 5 * t_max * (0.2969*math.sqrt(x) - 0.1260*x - 0.3516*x**2
                          + 0.2843*x**3 - 0.1015*x**4)
        if te_thickness > 0:
            yt = max(yt, te_thickness * (1 - x))
        if p > 0 and m > 0:
            yc = m/(p*p)*(2*p*x - x*x) if x < p else m/((1-p)**2)*((1-2*p)+2*p*x-x*x)
            dyc = 2*m/(p*p)*(p-x) if x < p else 2*m/((1-p)**2)*(p-x)
            theta = math.atan(dyc)
            xu = x - yt*math.sin(theta); yu = yc + yt*math.cos(theta)
            xl = x + yt*math.sin(theta); yl = yc - yt*math.cos(theta)
        else:
            xu, yu = x, yt
            xl, yl = x, -yt
        pts.append((xu, yu))
    for i in range(n-2, 0, -1):
        beta = math.pi * i / (n - 1)
        x = 0.5 * (1 - math.cos(beta))
        yt = 5 * t_max * (0.2969*math.sqrt(x) - 0.1260*x - 0.3516*x**2
                          + 0.2843*x**3 - 0.1015*x**4)
        if te_thickness > 0:
            yt = max(yt, te_thickness * (1 - x))
        if p > 0 and m > 0:
            yc = m/(p*p)*(2*p*x - x*x) if x < p else m/((1-p)**2)*((1-2*p)+2*p*x-x*x)
            dyc = 2*m/(p*p)*(p-x) if x < p else 2*m/((1-p)**2)*(p-x)
            theta = math.atan(dyc)
            xl = x + yt*math.sin(theta); yl = yc - yt*math.cos(theta)
        else:
            xl, yl = x, -yt
        pts.append((xl, yl))
    return pts


# ============================================================
# B-SPLINE WING LOFTING
# ============================================================

def loft_bspline_wing(name, root_chord, tip_chord, span,
                      sweep, dihedral, twist_deg, naca_code,
                      n_span_half=40):
    profile = naca_4digit(naca_code, FOIL_PTS)
    n_prof = len(profile)
    half_span = span / 2

    eta_knots = [0.0, 0.3, 0.7, 1.0]
    chord_vals = [root_chord,
                  root_chord - 0.25*(root_chord-tip_chord),
                  tip_chord + 0.15*(root_chord-tip_chord),
                  tip_chord]
    sweep_vals = [0, sweep*0.3, sweep*0.75, sweep]
    dihed_vals = [0, dihedral*0.2, dihedral*0.65, dihedral]
    twist_vals = [0, twist_deg*0.15, twist_deg*0.6, twist_deg]

    chord_coeffs = cubic_bspline_prep(eta_knots, chord_vals)
    sweep_coeffs = cubic_bspline_prep(eta_knots, sweep_vals)
    dihed_coeffs = cubic_bspline_prep(eta_knots, dihed_vals)
    twist_coeffs = cubic_bspline_prep(eta_knots, twist_vals)

    sections_eta = []
    for i in range(n_span_half + 1):
        beta = math.pi * 0.5 * i / n_span_half
        sections_eta.append(1 - math.cos(beta))

    all_verts = []
    all_faces = []
    total_sections = len(sections_eta) * 2 - 1

    for side in [1, -1]:
        etas = sections_eta if side == 1 else list(reversed(sections_eta[1:]))
        for eta in etas:
            c = cubic_bspline_eval(eta_knots, chord_coeffs, eta)
            sw = cubic_bspline_eval(eta_knots, sweep_coeffs, eta)
            dh = cubic_bspline_eval(eta_knots, dihed_coeffs, eta)
            tw = cubic_bspline_eval(eta_knots, twist_coeffs, eta)
            tw_rad = math.radians(tw)
            y_pos = side * eta * half_span

            for px, py in profile:
                xr = (px - 0.25) * c
                zr = py * c
                x_tw = xr * math.cos(tw_rad) - zr * math.sin(tw_rad)
                z_tw = xr * math.sin(tw_rad) + zr * math.cos(tw_rad)
                x_final = x_tw + 0.25*c + sw
                y_final = y_pos
                z_final = z_tw + dh
                all_verts.append((x_final, y_final, z_final))

    n_sections = total_sections
    for s in range(n_sections - 1):
        for i in range(n_prof):
            i_next = (i + 1) % n_prof
            v0 = s * n_prof + i
            v1 = s * n_prof + i_next
            v2 = (s + 1) * n_prof + i_next
            v3 = (s + 1) * n_prof + i
            all_faces.append((v0, v1, v2, v3))

    # Tip caps
    n_tip_rings = 8
    for tip_side, base_section in [(1, n_span_half), (-1, total_sections - 1)]:
        base_start = base_section * n_prof
        tip_y = tip_side * half_span
        center_x = sum(all_verts[base_start + i][0] for i in range(n_prof)) / n_prof
        center_z = sum(all_verts[base_start + i][2] for i in range(n_prof)) / n_prof
        prev_ring_start = base_start

        for ring in range(1, n_tip_rings + 1):
            t = ring / n_tip_rings
            shrink = math.cos(t * math.pi / 2)
            y_off = tip_side * half_span * 0.03 * math.sin(t * math.pi / 2)

            if ring < n_tip_rings:
                ring_start = len(all_verts)
                for i in range(n_prof):
                    bx, by, bz = all_verts[base_start + i]
                    nx = center_x + (bx - center_x) * shrink
                    nz = center_z + (bz - center_z) * shrink
                    all_verts.append((nx, tip_y + y_off, nz))
                for i in range(n_prof):
                    i_next = (i + 1) % n_prof
                    if tip_side == 1:
                        all_faces.append((prev_ring_start + i, prev_ring_start + i_next,
                                          ring_start + i_next, ring_start + i))
                    else:
                        all_faces.append((prev_ring_start + i_next, prev_ring_start + i,
                                          ring_start + i, ring_start + i_next))
                prev_ring_start = ring_start
            else:
                tip_center = len(all_verts)
                all_verts.append((center_x, tip_y + y_off, center_z))
                for i in range(n_prof):
                    i_next = (i + 1) % n_prof
                    if tip_side == 1:
                        all_faces.append((prev_ring_start + i, prev_ring_start + i_next, tip_center))
                    else:
                        all_faces.append((prev_ring_start + i_next, prev_ring_start + i, tip_center))

    mesh = bpy.data.meshes.new(name)
    mesh.from_pydata(all_verts, [], all_faces)
    mesh.update()
    obj = bpy.data.objects.new(name, mesh)
    bpy.context.collection.objects.link(obj)
    bpy.context.view_layer.objects.active = obj
    obj.select_set(True)

    bpy.ops.object.mode_set(mode='EDIT')
    bpy.ops.mesh.select_all(action='SELECT')
    bpy.ops.mesh.normals_make_consistent(inside=False)
    bpy.ops.object.mode_set(mode='OBJECT')

    sub = obj.modifiers.new("Subdiv", type='SUBSURF')
    sub.levels = 1
    bpy.ops.object.modifier_apply(modifier=sub.name)
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
