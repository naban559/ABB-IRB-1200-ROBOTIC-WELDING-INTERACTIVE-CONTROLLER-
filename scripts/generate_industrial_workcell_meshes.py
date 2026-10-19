#!/usr/bin/env python3
"""
Generates high-precision binary STL meshes for the ABB IRB 1200 Welding Workcell:
1. Industrial Welding Table (precision ground top, tubular frame, leveling pads, lower shelf)
2. Workpiece Assembly (Base plate, vertical fillet plate, transverse L-gusset plate, chamfered weld groove, toggle clamps)
3. MIG/GMAW Welding Torch (mount adapter, barrel, shock sensor, swan-neck, nozzle, contact wire)
"""

import os
import math
import struct


def write_stl(filename, facets):
    """Write binary STL file."""
    os.makedirs(os.path.dirname(filename), exist_ok=True)
    with open(filename, 'wb') as f:
        header = b'ABB IRB 1200 Precision Industrial Welding Workcell STL' + b' ' * 80
        f.write(header[:80])
        f.write(struct.pack('<I', len(facets)))
        for normal, v1, v2, v3 in facets:
            f.write(struct.pack('<3f3f3f3fH',
                normal[0], normal[1], normal[2],
                v1[0], v1[1], v1[2],
                v2[0], v2[1], v2[2],
                v3[0], v3[1], v3[2],
                0))


def calc_normal(v1, v2, v3):
    ax, ay, az = v2[0] - v1[0], v2[1] - v1[1], v2[2] - v1[2]
    bx, by, bz = v3[0] - v1[0], v3[1] - v1[1], v3[2] - v1[2]
    nx = ay * bz - az * by
    ny = az * bx - ax * bz
    nz = ax * by - ay * bx
    l = math.sqrt(nx*nx + ny*ny + nz*nz)
    if l > 1e-8:
        return (nx/l, ny/l, nz/l)
    return (0.0, 0.0, 1.0)


def add_box(facets, cx, cy, cz, sx, sy, sz):
    hx, hy, hz = sx/2.0, sy/2.0, sz/2.0
    corners = [
        (cx - hx, cy - hy, cz - hz),
        (cx + hx, cy - hy, cz - hz),
        (cx + hx, cy + hy, cz - hz),
        (cx - hx, cy + hy, cz - hz),
        (cx - hx, cy - hy, cz + hz),
        (cx + hx, cy - hy, cz + hz),
        (cx + hx, cy + hy, cz + hz),
        (cx - hx, cy + hy, cz + hz),
    ]
    faces = [
        (0, 1, 2, 3), # bottom (-z)
        (4, 7, 6, 5), # top (+z)
        (0, 4, 5, 1), # front (-y)
        (2, 6, 7, 3), # back (+y)
        (0, 3, 7, 4), # left (-x)
        (1, 5, 6, 2), # right (+x)
    ]
    for i1, i2, i3, i4 in faces:
        p1, p2, p3, p4 = corners[i1], corners[i2], corners[i3], corners[i4]
        n1 = calc_normal(p1, p2, p3)
        facets.append((n1, p1, p2, p3))
        n2 = calc_normal(p1, p3, p4)
        facets.append((n2, p1, p3, p4))


def add_cylinder(facets, cx, cy, z_bottom, z_top, r, segments=20):
    angle_step = 2 * math.pi / segments
    for i in range(segments):
        a1 = i * angle_step
        a2 = (i + 1) * angle_step
        x1, y1 = cx + r * math.cos(a1), cy + r * math.sin(a1)
        x2, y2 = cx + r * math.cos(a2), cy + r * math.sin(a2)
        p1 = (x1, y1, z_bottom)
        p2 = (x2, y2, z_bottom)
        p3 = (x2, y2, z_top)
        p4 = (x1, y1, z_top)
        facets.append((calc_normal(p1, p2, p3), p1, p2, p3))
        facets.append((calc_normal(p1, p3, p4), p1, p3, p4))
        # Caps
        pb0 = (cx, cy, z_bottom)
        facets.append(((0,0,-1), pb0, (x2, y2, z_bottom), (x1, y1, z_bottom)))
        pt0 = (cx, cy, z_top)
        facets.append(((0,0,1), pt0, (x1, y1, z_top), (x2, y2, z_top)))


def add_cone(facets, p_base, p_tip, r_base, r_tip, segments=16):
    dx, dy, dz = p_tip[0] - p_base[0], p_tip[1] - p_base[1], p_tip[2] - p_base[2]
    L = math.sqrt(dx*dx + dy*dy + dz*dz)
    if L < 1e-6: return
    ux, uy, uz = dx/L, dy/L, dz/L
    if abs(uz) < 0.99:
        vx, vy, vz = -uy, ux, 0
    else:
        vx, vy, vz = 0, -uz, uy
    vl = math.sqrt(vx*vx + vy*vy + vz*vz)
    vx, vy, vz = vx/vl, vy/vl, vz/vl
    wx, wy, wz = uy*vz - uz*vy, uz*vx - ux*vz, ux*vy - uy*vx

    angle_step = 2 * math.pi / segments
    for i in range(segments):
        a1 = i * angle_step
        a2 = (i + 1) * angle_step
        c1, s1 = math.cos(a1), math.sin(a1)
        c2, s2 = math.cos(a2), math.sin(a2)

        b1 = (p_base[0] + r_base*(c1*vx + s1*wx), p_base[1] + r_base*(c1*vy + s1*wy), p_base[2] + r_base*(c1*vz + s1*wz))
        b2 = (p_base[0] + r_base*(c2*vx + s2*wx), p_base[1] + r_base*(c2*vy + s2*wy), p_base[2] + r_base*(c2*vz + s2*wz))
        t1 = (p_tip[0] + r_tip*(c1*vx + s1*wx), p_tip[1] + r_tip*(c1*vy + s1*wy), p_tip[2] + r_tip*(c1*vz + s1*wz))
        t2 = (p_tip[0] + r_tip*(c2*vx + s2*wx), p_tip[1] + r_tip*(c2*vy + s2*wy), p_tip[2] + r_tip*(c2*vz + s2*wz))

        facets.append((calc_normal(b1, b2, t2), b1, b2, t2))
        facets.append((calc_normal(b1, t2, t1), b1, t2, t1))
        facets.append(((-ux, -uy, -uz), p_base, b2, b1))
        facets.append(((ux, uy, uz), p_tip, t1, t2))


def generate_table_mesh():
    """Industrial Welding Table mesh (0.60m x 0.80m x 0.50m total)."""
    facets = []
    # Solid heavy ground plate: 0.60m in X, 0.80m in Y, 0.04m thick, centered at Z=0.0
    # Top face at Z=+0.02, bottom face at Z=-0.02
    add_box(facets, 0, 0, 0.0, 0.60, 0.80, 0.04)

    # Perimeter channel frame
    add_box(facets, 0,  0.37, -0.045, 0.56, 0.05, 0.05)
    add_box(facets, 0, -0.37, -0.045, 0.56, 0.05, 0.05)
    add_box(facets,  0.27, 0, -0.045, 0.05, 0.74, 0.05)
    add_box(facets, -0.27, 0, -0.045, 0.05, 0.74, 0.05)

    # 4 Heavy tubular steel legs: 60mm x 60mm
    leg_x, leg_y = 0.25, 0.35
    leg_len = 0.41
    z_leg_center = -0.045 - leg_len/2.0
    for lx in [-leg_x, leg_x]:
        for ly in [-leg_y, leg_y]:
            add_box(facets, lx, ly, z_leg_center, 0.06, 0.06, leg_len)
            # Leveling foot pad
            add_box(facets, lx, ly, -0.045 - leg_len - 0.01, 0.10, 0.10, 0.02)

    # Lower stiffener shelf
    add_box(facets, 0, 0, -0.32, 0.50, 0.70, 0.025)

    return facets


def generate_workpiece_mesh():
    """
    Workpiece Assembly with authentic physical fillet and L-joints:
    - Base plate: 0.28m x 0.24m x 0.016m (Z: 0 to 0.016m)
    - Vertical plate along Y: from Y=-0.10 to Y=+0.10, X in [-0.012, 0.0]
      -> Fillet seam root is EXACTLY at X=0.0, Y in [-0.09, +0.09], Z=0.016m!
    - Transverse plate along X: from X=-0.07 to X=0.03, Y in [-0.072, -0.060]
      -> Intersection forms the L-shaped corner seam!
    - Clamping fixtures at the four corners.
    """
    facets = []
    # 1. Base Plate (Horizontal): 0.28m x 0.24m x 0.016m
    # Centered at (0, 0, 0.008)
    add_box(facets, 0.0, 0.0, 0.008, 0.28, 0.24, 0.016)

    # 2. Main Vertical Fillet Plate (along Y-axis):
    # Length: 0.20m (Y: -0.10 to +0.10)
    # Thickness: 0.012m (X: -0.012 to 0.0 -> center at X = -0.006)
    # Height: 0.070m (Z: 0.016 to 0.086 -> center at Z = 0.051)
    add_box(facets, -0.006, 0.0, 0.051, 0.012, 0.20, 0.070)

    # 3. Transverse Plate for L-Shaped Seam (along X-axis):
    # Length: 0.10m (X: -0.07 to +0.03 -> center at X = -0.02)
    # Thickness: 0.012m (Y: -0.072 to -0.060 -> center at Y = -0.066)
    # Height: 0.070m (Z: 0.016 to 0.086 -> center at Z = 0.051)
    add_box(facets, -0.02, -0.066, 0.051, 0.10, 0.012, 0.070)

    # 4. Machined V-Groove along the main fillet root:
    # Runs along X=0.0, from Y=-0.095 to Y=+0.095, depth 2mm, width 4mm
    add_box(facets, 0.001, 0.0, 0.0165, 0.004, 0.19, 0.002)

    # 5. Machined V-Groove along the transverse L-seam root:
    # Runs along Y=-0.06, from X=-0.07 to X=0.03
    add_box(facets, -0.02, -0.059, 0.0165, 0.09, 0.004, 0.002)

    # 6. Industrial Toggle Clamps (4 corners):
    # Clamping base plate down to welding table
    clamp_coords = [(-0.11, 0.09), (0.11, 0.09), (-0.11, -0.09), (0.11, -0.09)]
    for cx, cy in clamp_coords:
        # Clamp base block
        add_box(facets, cx, cy, 0.025, 0.035, 0.030, 0.035)
        # Pressure spindle touching plate
        spindle_y = cy - 0.018 if cy > 0 else cy + 0.018
        add_box(facets, cx, spindle_y, 0.020, 0.018, 0.018, 0.025)
        # Handle lever
        add_box(facets, cx, cy, 0.050, 0.012, 0.012, 0.025)

    return facets


def generate_torch_mesh():
    """MIG/GMAW Industrial Welding Torch with accurate swan-neck and nozzle."""
    facets = []
    # Tool0 flange mount adapter: D=80mm, h=14mm
    add_cylinder(facets, 0, 0, 0.0, 0.014, 0.040, segments=24)
    # Insulator spacer
    add_cylinder(facets, 0, 0, 0.014, 0.032, 0.026, segments=20)
    # Main torch barrel
    add_cylinder(facets, 0, 0, 0.032, 0.095, 0.022, segments=20)
    # Gas/Water/Wire cable coupling block
    add_box(facets, 0.020, 0, 0.065, 0.032, 0.044, 0.048)
    # Swan neck curve: angled pipe from (0, 0, 0.095) to (0.045, 0, 0.170)
    add_cone(facets, (0, 0, 0.095), (0.045, 0, 0.170), 0.018, 0.014, segments=16)
    # Conical gas nozzle: from (0.045, 0, 0.170) to (0.060, 0, 0.220)
    add_cone(facets, (0.045, 0, 0.170), (0.060, 0, 0.220), 0.014, 0.009, segments=16)
    # Copper contact tip & welding wire: from (0.060, 0, 0.220) to (0.065, 0, 0.240)
    add_cone(facets, (0.060, 0, 0.220), (0.065, 0, 0.240), 0.005, 0.0015, segments=12)

    return facets


def main():
    pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    vis_dir = os.path.join(pkg_dir, 'meshes', 'visual')
    col_dir = os.path.join(pkg_dir, 'meshes', 'collision')

    print("Generating precision workcell meshes...")

    table_facets = generate_table_mesh()
    write_stl(os.path.join(vis_dir, 'welding_table.stl'), table_facets)
    write_stl(os.path.join(col_dir, 'welding_table.stl'), table_facets)
    print(f"  [OK] welding_table.stl ({len(table_facets)} triangles)")

    workpiece_facets = generate_workpiece_mesh()
    write_stl(os.path.join(vis_dir, 'workpiece_plates.stl'), workpiece_facets)
    write_stl(os.path.join(col_dir, 'workpiece_plates.stl'), workpiece_facets)
    print(f"  [OK] workpiece_plates.stl ({len(workpiece_facets)} triangles)")

    torch_facets = generate_torch_mesh()
    write_stl(os.path.join(vis_dir, 'welding_torch.stl'), torch_facets)
    write_stl(os.path.join(col_dir, 'welding_torch.stl'), torch_facets)
    print(f"  [OK] welding_torch.stl ({len(torch_facets)} triangles)")

    print("All meshes generated successfully.")


if __name__ == '__main__':
    main()

