from common import *

# ============================================================
# BOARD (piece 1)
# ============================================================

def make_placeholder_board(name):
    print("  Placeholder board...")
    bm = bmesh.new()
    n_x = 80
    n_y = 32
    n_z = 8
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

    n_circ = 2 * n_y + 2 * n_z

    def end_thickness_factor(t_val):
        nose_zone = 0.08
        tail_zone = 0.10
        if t_val < nose_zone:
            return math.sqrt(t_val / nose_zone)
        elif t_val > (1.0 - tail_zone):
            return math.sqrt((1.0 - t_val) / tail_zone)
        return 1.0

    rings = []
    for i in range(n_x + 1):
        t = i / n_x
        x = (t - 0.5) * BOARD_LENGTH
        pw = half_w * interp_profile(t)
        th_factor = end_thickness_factor(t)
        ht = half_t * th_factor

        ring = []
        for j in range(n_y + 1):
            s = j / n_y
            y = (s - 0.5) * 2 * pw
            yn = abs(s - 0.5) * 2
            z = ht * (1.0 - 0.08 * (1 - yn * yn))
            ring.append((x, y, z))

        for k in range(1, n_z):
            angle = k / n_z * math.pi
            y = pw + ht * 0.3 * math.cos(math.pi - angle)
            z = ht * math.cos(angle)
            ring.append((x, y, z))

        for j in range(n_y, -1, -1):
            s = j / n_y
            y = (s - 0.5) * 2 * pw
            yn = abs(s - 0.5) * 2
            z = -ht * (1.0 - 0.05 * yn)
            ring.append((x, y, z))

        for k in range(1, n_z):
            angle = k / n_z * math.pi
            y = -pw - ht * 0.3 * math.cos(math.pi - angle)
            z = -ht * math.cos(angle)
            ring.append((x, y, z))

        for v in ring:
            bm.verts.new(v)
        rings.append(len(ring))

    n_ring = rings[0]
    bm.verts.ensure_lookup_table()

    for i in range(n_x):
        for j in range(n_ring):
            jn = (j + 1) % n_ring
            v0 = i * n_ring + j
            v1 = i * n_ring + jn
            v2 = (i+1) * n_ring + jn
            v3 = (i+1) * n_ring + j
            bm.faces.new((bm.verts[v0], bm.verts[v1], bm.verts[v2], bm.verts[v3]))

    center_nose = bm.verts.new((-0.5 * BOARD_LENGTH, 0, 0))
    bm.verts.ensure_lookup_table()
    for j in range(n_ring):
        jn = (j + 1) % n_ring
        bm.faces.new((center_nose, bm.verts[jn], bm.verts[j]))

    last_start = n_x * n_ring
    center_tail = bm.verts.new((0.5 * BOARD_LENGTH, 0, 0))
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


def build_board():
    print("[1/5] Board...")
    clear_scene()
    board = import_and_scale_mesh(BOARD_FILE, SCALE, "Board") if BOARD_FILE else None
    if not board: board = make_placeholder_board("Board")

    # Mast foot screw holes
    mast_board_x = BOARD_LENGTH * 0.30  # 80% from nose
    foot_size = MAST_CHORD * 1.2
    screw_inset = foot_size * 0.30
    add_screw_holes(board,
        [(mast_board_x - screw_inset, -screw_inset, 0),
         (mast_board_x - screw_inset,  screw_inset, 0),
         (mast_board_x + screw_inset, -screw_inset, 0),
         (mast_board_x + screw_inset,  screw_inset, 0)],
        SCREW_DIAM, BOARD_THICK + 4*_P, 'z')

    export_stl(board, "1_board.stl")
    return board


if __name__ == "__main__":
    build_board()
