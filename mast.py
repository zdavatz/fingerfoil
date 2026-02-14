import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import *

# ============================================================
# MAST (piece 2)
# ============================================================

def make_placeholder_mast(name):
    print("  Placeholder mast...")
    profile = naca_4digit(MAST_NACA, FOIL_PTS)
    n_prof = len(profile)

    # Mastfoot flares from NACA to a rounded-square mounting surface
    foot_size = MAST_CHORD * 1.2

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
            blend = ft

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


def build_mast():
    print("[2/5] Mast...")
    clear_scene()
    mast = import_and_scale_mesh(MAST_FILE, SCALE, "Mast") if MAST_FILE else None
    if not mast:
        mast = make_placeholder_mast("Mast")
        screw_z = 1.5 * PS
        add_screw_holes(mast,
            [(-7.0*PS, 0, screw_z),
             (-1.0*PS, 0, screw_z)],
            SCREW_DIAM, 5.0*PS, 'z')
    export_stl(mast, "2_mast.stl")
    return mast


if __name__ == "__main__":
    build_mast()
