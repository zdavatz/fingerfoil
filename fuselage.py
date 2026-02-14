import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import *

# ============================================================
# FUSELAGE (piece 3)
# ============================================================

def make_placeholder_fuselage(name):
    print("  Placeholder fuselage...")

    fw_r = 4.76 * PS; fh = 6.0 * PS
    n_circ = 32

    fy = FUSE_LENGTH * 0.11
    sy = FUSE_LENGTH * 0.88

    def naca_lower_z_at(code, chord, fuse_y, y_center):
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
        z_lower = fh + (yc - yt) * chord
        half_t = yt * chord
        return z_lower, half_t

    fw_blend_y = 5.0 * PS
    st_blend_y = 4.0 * PS

    y_stations = set()
    for i in range(81):
        y_stations.add(i / 80 * FUSE_LENGTH)
    for yc in [fy, sy]:
        for dy in [-8, -6, -5, -4, -3, -2.5, -2, -1.5, -1, -0.5, 0,
                    0.5, 1, 1.5, 2, 2.5, 3, 4, 5, 6, 8]:
            yv = yc + dy * PS
            if 0 <= yv <= FUSE_LENGTH:
                y_stations.add(yv)
    y_stations = sorted(y_stations)
    n_len = len(y_stations)

    bm = bmesh.new()

    for i, y in enumerate(y_stations):
        t = y / FUSE_LENGTH
        if t < 0.15:
            s = t / 0.15
            rf = 0.35 + 0.65 * math.sin(s * math.pi / 2)
        elif t < 0.55: rf = 1.0
        else:          rf = 1.0 - (t-0.55)/0.45*0.65
        rx = max(fw_r*rf, 0.3); rz = max(fh/2*rf, 0.3)

        fw_d = abs(y - fy)
        st_d = abs(y - sy)
        fw_t = max(0, 1.0 - fw_d / fw_blend_y) if fw_d < fw_blend_y else 0
        st_t = max(0, 1.0 - st_d / st_blend_y) if st_d < st_blend_y else 0
        fw_t = (math.sin(fw_t * math.pi / 2)) ** 2
        st_t = (math.sin(st_t * math.pi / 2)) ** 2

        fw_z, fw_ht = naca_lower_z_at(FW_NACA, FW_ROOT_CHORD, y, fy)
        st_z, st_ht = naca_lower_z_at(STAB_NACA, STAB_ROOT_CHORD, y, sy)

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
                sin_th = math.sin(th)
                target_z = wing_z_lower
                vert_blend = blend * sin_th
                if z_ell > target_z:
                    z_ell = z_ell - vert_blend * (z_ell - target_z)

            bm.verts.new((x_ell, y, z_ell))

    bm.verts.ensure_lookup_table()

    for i in range(n_len - 1):
        for j in range(n_circ):
            jn = (j+1) % n_circ
            bm.faces.new((bm.verts[i*n_circ+j], bm.verts[i*n_circ+jn],
                           bm.verts[(i+1)*n_circ+jn], bm.verts[(i+1)*n_circ+j]))

    nv = bm.verts.new((0, -0.5*PS, fh/2)); bm.verts.ensure_lookup_table()
    for j in range(n_circ):
        bm.faces.new((nv, bm.verts[(j+1)%n_circ], bm.verts[j]))
    tb = (n_len - 1) * n_circ
    tv = bm.verts.new((0, FUSE_LENGTH+0.5*PS, fh/2)); bm.verts.ensure_lookup_table()
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

    # NACA-shaped mast pocket
    py_pos = FUSE_LENGTH * 0.44
    profile = naca_4digit(MAST_NACA, 48)
    cl = SLOT_CLEARANCE
    mast_c = MAST_CHORD + cl * 2

    cutter_verts = []
    cutter_faces = []
    n_prof = len(profile)
    z_bot = fh * 0.5
    z_top = fh + 2.0 * PS

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

    sub = obj.modifiers.new("S", type='SUBSURF'); sub.levels = 1
    bpy.ops.object.modifier_apply(modifier=sub.name)

    # Screw holes
    add_screw_holes(obj,
        [(0, fy - 2.0*PS, fh/2),
         (0, fy + 3.0*PS, fh/2),
         (0, fy + 8.0*PS, fh/2)],
        SCREW_DIAM, fh + 4*PS, 'z')

    st_shift = STAB_ROOT_CHORD * 0.25
    st_half_sp = STAB_ROOT_CHORD * 0.15
    add_screw_holes(obj,
        [(0, sy - st_half_sp + st_shift, fh/2),
         (0, sy + st_half_sp + st_shift, fh/2)],
        SCREW_DIAM, fh + 4*PS, 'z')

    mast_screw_z = fh * 0.25
    mast_screw_depth = fh * 0.5 + 2*PS
    add_screw_holes(obj,
        [(0, py_pos - 7.0*PS, mast_screw_z),
         (0, py_pos - 1.0*PS, mast_screw_z)],
        SCREW_DIAM, mast_screw_depth, 'z')

    bpy.ops.object.shade_smooth()
    return obj


def build_fuselage():
    print("[3/5] Fuselage...")
    clear_scene()
    fuse = import_and_scale_mesh(FUSE_FILE, SCALE, "Fuselage") if FUSE_FILE else None
    if not fuse: fuse = make_placeholder_fuselage("Fuselage")
    export_stl(fuse, "3_fuselage.stl")
    return fuse


if __name__ == "__main__":
    build_fuselage()
