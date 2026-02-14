import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import *

# ============================================================
# BOARD (piece 1)
# ============================================================

def make_placeholder_board(name):
    print("  Placeholder board...")
    bm = bmesh.new()
    n_x = 120
    n_circ = 64
    half_w = BOARD_WIDTH / 2
    half_t = BOARD_THICK / 2

    # Planform width profile from Hydroskate reference
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

    # Skateboard-style ends: width stays wide, only thickness tapers
    nose_zone = 0.10
    tail_zone = 0.10

    def end_thickness_factor(t_val):
        """Taper thickness at ends — minimum 30% for blunt rounded tips."""
        if t_val < nose_zone:
            s = t_val / nose_zone
            return 0.40 + 0.60 * (math.sin(s * math.pi / 2))
        elif t_val > (1.0 - tail_zone):
            s = (1.0 - t_val) / tail_zone
            return 0.40 + 0.60 * (math.sin(s * math.pi / 2))
        return 1.0

    # Cosine spacing along length
    t_values = []
    for i in range(n_x + 1):
        beta = math.pi * i / n_x
        t_values.append(0.5 * (1 - math.cos(beta)))

    # Cross-section: flat deck, round rails (semicircle), flat bottom
    # Width follows the Hydroskate planform — stays WIDE at nose/tail
    # Only thickness reduces → blunt wide rounded ends like a skateboard

    for t in t_values:
        x = (t - 0.5) * BOARD_LENGTH

        pw = max(half_w * interp_profile(t), 0.1)  # planform width, no narrowing
        ht = max(half_t * end_thickness_factor(t), 0.1)  # only thickness tapers

        rail_r = ht  # rail = full semicircle of the local thickness
        flat_hw = max(pw - rail_r, 0.0)

        # Build cross-section: n_circ points going clockwise
        # Split into segments:
        #   top-right flat, right rail (semicircle), bottom-right flat,
        #   bottom-left flat, left rail (semicircle), top-left flat
        n_flat = n_circ // 4      # points per flat segment (top or bottom, one side)
        n_rail = n_circ // 4      # points per rail semicircle

        ring = []

        # 1) Top surface right half: center → right edge (y=0 to y=flat_hw)
        for j in range(n_flat):
            s = j / n_flat
            y = s * flat_hw
            # Gentle deck crown
            z = ht - 0.03 * ht * (1 - s * s)
            ring.append((y, z))

        # 2) Right rail: semicircle from top to bottom
        for j in range(n_rail):
            angle = j / n_rail * math.pi  # 0=top, π=bottom
            y = flat_hw + rail_r * math.sin(angle)
            z = rail_r * math.cos(angle)
            ring.append((y, z))

        # 3) Bottom surface right half → left half (y=flat_hw → y=-flat_hw)
        for j in range(n_flat):
            s = j / n_flat
            y = flat_hw * (1 - 2 * s)
            z = -ht + 0.02 * ht * abs(y / max(flat_hw, 0.001))
            ring.append((y, z))

        # 4) Left rail: semicircle from bottom to top
        for j in range(n_rail):
            angle = j / n_rail * math.pi  # 0=bottom, π=top
            y = -flat_hw - rail_r * math.sin(angle)
            z = -rail_r * math.cos(angle)
            ring.append((y, z))

        for y, z in ring:
            bm.verts.new((x, y, z))

    bm.verts.ensure_lookup_table()
    n_stations = len(t_values)

    for i in range(n_stations - 1):
        for j in range(n_circ):
            jn = (j + 1) % n_circ
            v0 = i * n_circ + j
            v1 = i * n_circ + jn
            v2 = (i + 1) * n_circ + jn
            v3 = (i + 1) * n_circ + j
            bm.faces.new((bm.verts[v0], bm.verts[v1], bm.verts[v2], bm.verts[v3]))

    # Rounded nose cap: hemisphere proportional to nose WIDTH
    n_cap_rings = 12
    first_ring = [(bm.verts[j].co.y, bm.verts[j].co.z) for j in range(n_circ)]
    nose_x0 = bm.verts[0].co.x
    # Cap depth = half the nose width for a proper rounded end
    nose_hw = max(abs(bm.verts[j].co.y) for j in range(n_circ))
    nose_cap_depth = nose_hw * 0.35

    prev_start = 0
    for cr in range(1, n_cap_rings + 1):
        s = cr / n_cap_rings
        shrink = math.cos(s * math.pi / 2)  # 1→0
        x_off = -nose_cap_depth * math.sin(s * math.pi / 2)

        if cr < n_cap_rings:
            ring_start = len(bm.verts)
            for j in range(n_circ):
                by, bz = first_ring[j]
                bm.verts.new((nose_x0 + x_off, by * shrink, bz * shrink))
            bm.verts.ensure_lookup_table()
            for j in range(n_circ):
                jn = (j + 1) % n_circ
                bm.faces.new((bm.verts[prev_start + j], bm.verts[prev_start + jn],
                               bm.verts[ring_start + jn], bm.verts[ring_start + j]))
            prev_start = ring_start
        else:
            tip = bm.verts.new((nose_x0 + x_off, 0, 0))
            bm.verts.ensure_lookup_table()
            for j in range(n_circ):
                jn = (j + 1) % n_circ
                bm.faces.new((bm.verts[prev_start + j], bm.verts[prev_start + jn], tip))

    # Rounded tail cap
    last_start = (n_stations - 1) * n_circ
    last_ring = [(bm.verts[last_start + j].co.y, bm.verts[last_start + j].co.z) for j in range(n_circ)]
    tail_x0 = bm.verts[last_start].co.x
    tail_hw = max(abs(bm.verts[last_start + j].co.y) for j in range(n_circ))
    tail_cap_depth = tail_hw * 0.35

    prev_start = last_start
    for cr in range(1, n_cap_rings + 1):
        s = cr / n_cap_rings
        shrink = math.cos(s * math.pi / 2)
        x_off = tail_cap_depth * math.sin(s * math.pi / 2)

        if cr < n_cap_rings:
            ring_start = len(bm.verts)
            for j in range(n_circ):
                by, bz = last_ring[j]
                bm.verts.new((tail_x0 + x_off, by * shrink, bz * shrink))
            bm.verts.ensure_lookup_table()
            for j in range(n_circ):
                jn = (j + 1) % n_circ
                bm.faces.new((bm.verts[prev_start + jn], bm.verts[prev_start + j],
                               bm.verts[ring_start + j], bm.verts[ring_start + jn]))
            prev_start = ring_start
        else:
            tip = bm.verts.new((tail_x0 + x_off, 0, 0))
            bm.verts.ensure_lookup_table()
            for j in range(n_circ):
                jn = (j + 1) % n_circ
                bm.faces.new((bm.verts[prev_start + jn], bm.verts[prev_start + j], tip))

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
        SCREW_DIAM, BOARD_THICK + 4*PS, 'z')

    export_stl(board, "1_board.stl")
    print(f"  Board: {BOARD_LENGTH:.1f} x {BOARD_WIDTH:.1f} x {BOARD_THICK:.1f}mm")
    return board


if __name__ == "__main__":
    build_board()
