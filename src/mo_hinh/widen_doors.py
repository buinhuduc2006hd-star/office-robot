"""
Script chạy với Blender headless để mở rộng cửa phòng thêm 0.2m.
Cách dùng:
    blender --background --python widen_doors.py

Nguyên lý:
- Tải office.dae vào Blender
- Với mỗi mesh object, tìm các vertex gần cạnh P1/P2 của từng cửa (trong world space)
- Dịch chuyển các vertex đó ra ngoài thêm 0.1m mỗi phía (tổng 0.2m rộng hơn)
- Export lại thành office.dae
"""

import bpy
import bmesh
import math
import os

# ── CẤU HÌNH ────────────────────────────────────────────────────────────────
DAE_PATH = "/home/duc/robot_ws/src/mo_hinh/models/cpr_office/meshes/office.dae"
BACKUP_PATH = DAE_PATH + ".bak"

# Danh sách cửa (từ rooms.yaml) — p1, p2 là 2 mép cửa trong world coords
DOORS = [
    {
        "name": "Phong01_main_door",
        "p1": (8.10, 1.90),
        "p2": (9.15, 2.00),
    },
    {
        "name": "Phong02_main_door",
        "p1": (3.0, 1.8),
        "p2": (4.0, 1.8),
    },
]

WIDEN_TOTAL = 0.2       # tổng mở rộng (m) — mỗi mép dịch 0.1m ra ngoài
SEARCH_RADIUS = 0.18    # bán kính tìm vertex gần cạnh cửa (m)
# ────────────────────────────────────────────────────────────────────────────


def door_unit_vec(p1, p2):
    """Trả về vector đơn vị dọc theo chiều cửa (p1→p2)."""
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    length = math.sqrt(dx * dx + dy * dy)
    if length < 1e-6:
        return (1.0, 0.0)
    return (dx / length, dy / length)


def process_door(bm, door, widen_total, radius, obj_matrix):
    """
    Tìm và dịch các vertex ở cạnh P1/P2 của cửa trong world space.
    obj_matrix: ma trận transform của object (local→world).
    """
    half = widen_total / 2.0
    p1 = door["p1"]
    p2 = door["p2"]
    ux, uy = door_unit_vec(p1, p2)

    moved_p1 = 0
    moved_p2 = 0

    for v in bm.verts:
        # Chuyển vertex từ local sang world
        world_co = obj_matrix @ v.co
        wx, wy = world_co.x, world_co.y

        # Kiểm tra gần P1 không
        dist1 = math.sqrt((wx - p1[0]) ** 2 + (wy - p1[1]) ** 2)
        if dist1 < radius:
            # Dịch ra ngoài theo hướng ngược chiều cửa (-ux, -uy)
            # Tính displacement trong world space rồi transform ngược về local
            world_co.x -= ux * half
            world_co.y -= uy * half
            v.co = obj_matrix.inverted() @ world_co
            moved_p1 += 1
            continue

        # Kiểm tra gần P2 không
        dist2 = math.sqrt((wx - p2[0]) ** 2 + (wy - p2[1]) ** 2)
        if dist2 < radius:
            # Dịch ra ngoài theo hướng cùng chiều cửa (+ux, +uy)
            world_co.x += ux * half
            world_co.y += uy * half
            v.co = obj_matrix.inverted() @ world_co
            moved_p2 += 1

    return moved_p1, moved_p2


def main():
    print("=" * 60)
    print("  WIDEN DOORS — Office DAE mesh editor")
    print("=" * 60)

    # --- Backup file gốc ---
    if not os.path.exists(BACKUP_PATH):
        import shutil
        shutil.copy2(DAE_PATH, BACKUP_PATH)
        print(f"✅ Đã backup file gốc → {BACKUP_PATH}")
    else:
        print(f"ℹ️  Backup đã tồn tại: {BACKUP_PATH}")

    # --- Xóa scene mặc định ---
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete()
    for block in bpy.data.meshes:
        bpy.data.meshes.remove(block)

    # --- Import DAE ---
    print(f"\n📂 Đang import: {DAE_PATH}")
    bpy.ops.wm.collada_import(filepath=DAE_PATH)
    print(f"   Số objects sau import: {len(bpy.data.objects)}")

    # --- Xử lý từng cửa ---
    total_moved = {}
    for door in DOORS:
        total_moved[door["name"]] = {"p1": 0, "p2": 0}

    for obj in bpy.data.objects:
        if obj.type != 'MESH':
            continue

        mat = obj.matrix_world.copy()

        bm = bmesh.new()
        bm.from_mesh(obj.data)
        bm.verts.ensure_lookup_table()

        for door in DOORS:
            mp1, mp2 = process_door(bm, door, WIDEN_TOTAL, SEARCH_RADIUS, mat)
            if mp1 > 0 or mp2 > 0:
                print(f"   [{obj.name}] {door['name']}: P1={mp1} verts, P2={mp2} verts di chuyển")
                total_moved[door["name"]]["p1"] += mp1
                total_moved[door["name"]]["p2"] += mp2

        bm.to_mesh(obj.data)
        bm.free()
        obj.data.update()

    # --- Báo cáo kết quả ---
    print("\n📊 Kết quả:")
    for door in DOORS:
        r = total_moved[door["name"]]
        p1_ok = "✅" if r["p1"] > 0 else "⚠️  KHÔNG TÌM THẤY"
        p2_ok = "✅" if r["p2"] > 0 else "⚠️  KHÔNG TÌM THẤY"
        print(f"   {door['name']}:")
        print(f"     P1 {p1_ok} ({r['p1']} vertices)")
        print(f"     P2 {p2_ok} ({r['p2']} vertices)")

    # --- Export lại DAE ---
    print(f"\n💾 Đang export lại: {DAE_PATH}")
    bpy.ops.wm.collada_export(
        filepath=DAE_PATH,
        check_existing=False,
        apply_modifiers=True,
        selected=False,
        include_children=True,
        triangulate=True,
        use_object_instantiation=True,
        use_blender_profile=True,
        export_object_transformation_type_selection='matrix',
    )
    print("✅ Hoàn tất! Restart Gazebo để thấy thay đổi.")
    print("=" * 60)


main()
