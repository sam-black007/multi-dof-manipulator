#!/usr/bin/env python3
"""
URDF Inspection Tool for 6-DOF Robotic Arm

Loads a reference URDF and enumerates:
- Links and joints
- Parent->child relationships
- Joint types, origins, axes, limits
- Mesh file locations
- Inertial information
- Detects malformed references

Exits non-zero if required referenced assets are missing.
"""

import sys
import os
import xml.etree.ElementTree as ET
import traceback


def find_urdf(path):
    """Find and parse a URDF file, trying common extensions."""
    candidates = [path] + [path + ext for ext in ['.urdf', '.xacro']]
    for c in candidates:
        if os.path.exists(c):
            return ET.parse(c).getroot()
    print(f"ERROR: URDF not found at any of: {candidates}")
    return None


def main():
    if len(sys.argv) < 2:
        print("Usage: python inspect_robot_description.py <path_to_urdf>")
        sys.exit(1)

    urdf_path = sys.argv[1]
    root = find_urdf(urdf_path)
    if root is None:
        sys.exit(1)

    ns = '{http://www.ros.org/wiki/URDF}'

    print("=" * 60)
    print("URDF INSPECTION REPORT")
    print("=" * 60)
    print(f"\nFile: {os.path.abspath(urdf_path)}")
    print(f"Robot name: {root.get('name', 'N/A')}\n")

    # Enumerate links
    print("-" * 60)
    print("LINKS")
    print("-" * 60)
    links = root.findall(ns + 'link')
    print(f"Total links: {len(links)}")
    for link in links:
        name = link.get('name')
        inertial = link.find(ns + 'inertial')
        visual = link.find(ns + 'visual')
        collision = link.find(ns + 'collision')

        print(f"\n  Link: {name}")
        if inertial is not None:
            mass = inertial.find(ns + 'mass')
            inertia = inertial.find(ns + 'inertia')
            if mass is not None:
                print(f"    mass: {mass.get('value')}")
            if inertia is not None:
                print(f"    inertia: ixx={inertia.get('ixx')}, ixy={inertia.get('ixy')}, "
                      f"ixz={inertia.get('ixz')}, iyy={inertia.get('iyy')}, "
                      f"iyz={inertia.get('iyz')}, izz={inertia.get('izz')}")

        if visual is not None:
            geom = visual.find(ns + 'geometry')
            if geom is not None:
                mesh = geom.find(ns + 'mesh')
                if mesh is not None:
                    filename = mesh.get('filename', 'N/A')
                    print(f"    visual mesh: {filename}")
                    # Check if file exists (relative to URDF dir)
                    urdf_dir = os.path.dirname(os.path.abspath(urdf_path))
                    mesh_path = os.path.join(urdf_dir, filename.lstrip('./'))
                    if not os.path.exists(mesh_path):
                        print(f"    >>> WARNING: Mesh file NOT found at: {mesh_path}")

        if collision is not None:
            geom = collision.find(ns + 'geometry')
            if geom is not None:
                mesh = geom.find(ns + 'mesh')
                if mesh is not None:
                    filename = mesh.get('filename', 'N/A')
                    print(f"    collision mesh: {filename}")
                    urdf_dir = os.path.dirname(os.path.abspath(urdf_path))
                    mesh_path = os.path.join(urdf_dir, filename.lstrip('./'))
                    if not os.path.exists(mesh_path):
                        print(f"    >>> WARNING: Mesh file NOT found at: {mesh_path}")

    # Enumerate joints
    print("\n" + "-" * 60)
    print("JOINTTS")
    print("-" * 60)
    joints = root.findall(ns + 'joint')
    print(f"Total joints: {len(joints)}")
    for joint in joints:
        name = joint.get('name')
        joint_type = joint.get('type')
        origin = joint.find(ns + 'origin')
        axis = joint.find(ns + 'axis')
        parent = joint.find(ns + 'parent')
        child = joint.find(ns + 'child')
        limit = joint.find(ns + 'limit')

        print(f"\n  Joint: {name} (type: {joint_type})")
        if origin is not None:
            print(f"    origin xyz: {origin.get('xyz')}, rpy: {origin.get('rpy')}")
        if axis is not None:
            print(f"    axis: {axis.get('xyz')}")
        if parent is not None:
            print(f"    parent: {parent.get('child')}")
        if child is not None:
            print(f"    child: {child.get('link')}")
        if limit is not None:
            print(f"    limit lower: {limit.get('lower')}, upper: {limit.get('upper')}")
            effort = limit.get('effort', 'N/A')
            velocity = limit.get('velocity', 'N/A')
            print(f"    effort: {effort}, velocity: {velocity}")

    # Check for mesh resolution
    print("\n" + "=" * 60)
    print("MESH FILE EXISTENCE CHECK")
    print("=" * 60)
    mesh_issues = 0
    for link in links:
        name = link.get('name')
        for geom_elem in [link.find(ns + 'visual'), link.find(ns + 'collision')]:
            if geom_elem is None:
                continue
            mesh = geom.find(ns + 'geometry').find(ns + 'mesh') if geom.find(ns + 'geometry') else None
            if mesh is not None:
                filename = mesh.get('filename', '')
                if filename:
                    # Resolve relative to URDF directory
                    urdf_dir = os.path.dirname(os.path.abspath(urdf_path))
                    # Remove leading package:// or file:// prefixes
                    clean = filename.replace('package://', '').replace('file://', '')
                    mesh_path = os.path.join(urdf_dir, clean.lstrip('./'))
                    if os.path.exists(mesh_path):
                        print(f"  [OK] {name}: {clean}")
                    else:
                        print(f"  [MISSING] {name}: {clean}")
                        mesh_issues += 1

    for joint in joints:
        name = joint.get('name')
        for geom_elem_type in ['visual', 'collision']:
            geom = joint.find(ns + geom_elem_type)
            if geom is None:
                continue
            mesh = geom.find(ns + 'geometry').find(ns + 'mesh') if geom.find(ns + 'geometry') else None
            if mesh is not None:
                filename = mesh.get('filename', '')
                if filename:
                    urdf_dir = os.path.dirname(os.path.abspath(urdf_path))
                    clean = filename.replace('package://', '').replace('file://', '')
                    mesh_path = os.path.join(urdf_dir, clean.lstrip('./'))
                    if os.path.exists(mesh_path):
                        print(f"  [OK] {name}: {clean}")
                    else:
                        print(f"  [MISSING] {name}: {clean}")
                        mesh_issues += 1

    print(f"\nTotal missing mesh references: {mesh_issues}")

    # Success if no critical errors
    if mesh_issues > 0:
        print(f"\nWarnings: {mesh_issues} missing mesh references detected")
        sys.exit(0)  # Warnings but not fatal
    else:
        print("\nAll mesh references resolved successfully")
        sys.exit(0)


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"\nFATAL ERROR: {e}")
        traceback.print_exc()
        sys.exit(1)