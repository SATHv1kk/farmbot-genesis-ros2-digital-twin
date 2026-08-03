#!/usr/bin/env python3
# Copyright 2026 Sathvik
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Replace per-part collision meshes with one bounding-box primitive per link.

farmbot_genesis.urdf is built from ~1174 raw CAD parts consolidated into
5 links; each part's full-resolution visual mesh was also used verbatim
as its collision geometry, leaving hundreds of concave collision meshes
per moving link. Gazebo's ODE engine handles concave-trimesh collision
between dynamic bodies poorly (slow or unstable), so this script keeps
the detailed visuals but gives each link a single axis-aligned box
collision derived from the true world-space extent of its visual mesh
vertices (requires numpy; not a runtime/exec dependency of any node).

Run after regenerating/editing farmbot_genesis.urdf's visuals:
    python3 src/farmbot_description/scripts/simplify_collisions.py
"""

import glob
import os
import struct
import xml.etree.ElementTree as ET

import numpy as np

URDF_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
URDF_FILE = os.path.join(URDF_DIR, 'farmbot_genesis.urdf')
ASSETS_DIR = os.path.join(URDF_DIR, 'assets')
MESH_PREFIX = 'package://farmbot_description/assets/'


def read_stl_vertices(path: str) -> np.ndarray:
    """Return an (N, 3) float32 array of all triangle vertices in a mesh."""
    with open(path, 'rb') as f:
        data = f.read()

    n_tri = struct.unpack_from('<I', data, 80)[0]
    expected_len = 84 + n_tri * 50
    if len(data) < expected_len:
        raise ValueError(f'{path}: truncated binary STL')

    # Each 50-byte record: 12 bytes normal, 36 bytes (3 vertices), 2 bytes
    # attribute count. View as records and slice out the 9 vertex floats.
    dtype = np.dtype([
        ('normal', '<f4', 3),
        ('verts', '<f4', 9),
        ('attr', '<u2'),
    ])
    records = np.frombuffer(data, dtype=dtype, count=n_tri, offset=84)
    return records['verts'].reshape(-1, 3)


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Return the 3x3 rotation matrix for URDF-convention rpy."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def parse_origin(elem):
    """Return (xyz, rpy) tuples from an <origin> element (defaults 0)."""
    if elem is None:
        return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
    xyz = tuple(float(v) for v in elem.get('xyz', '0 0 0').split())
    rpy = tuple(float(v) for v in elem.get('rpy', '0 0 0').split())
    return xyz, rpy


def indent_xml(elem, level=0, spaces='  '):
    """Pretty-print an XML subtree in place (matches generate_urdf.py)."""
    indent = '\n' + spaces * level
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = indent + spaces
        for sub in elem:
            indent_xml(sub, level + 1, spaces)
        if not elem.tail or not elem.tail.strip():
            elem.tail = indent
    else:
        if level and (not elem.tail or not elem.tail.strip()):
            elem.tail = indent


def main() -> None:
    """Rewrite every link's collision geometry as one bounding box."""
    print('=' * 70)
    print('  FarmBot Genesis Collision Simplifier')
    print('=' * 70)

    mesh_files = sorted(glob.glob(os.path.join(ASSETS_DIR, '*.stl')))
    print(f'[INFO] Caching vertices for {len(mesh_files)} meshes ...')
    mesh_verts = {}
    for path in mesh_files:
        mesh_verts[os.path.basename(path)] = read_stl_vertices(path)

    print(f'[INFO] Parsing {URDF_FILE} ...')
    tree = ET.parse(URDF_FILE)
    root = tree.getroot()

    total_removed = 0
    total_added = 0
    for link in root.findall('link'):
        link_name = link.get('name')
        visuals = link.findall('visual')

        link_min = np.array([np.inf, np.inf, np.inf])
        link_max = np.array([-np.inf, -np.inf, -np.inf])
        found_mesh = False

        for vis in visuals:
            mesh_elem = vis.find('geometry/mesh')
            if mesh_elem is None:
                continue
            filename = mesh_elem.get('filename', '')
            if not filename.startswith(MESH_PREFIX):
                continue
            mesh_name = filename[len(MESH_PREFIX):]
            verts = mesh_verts.get(mesh_name)
            if verts is None:
                continue

            xyz, rpy = parse_origin(vis.find('origin'))
            rot = rpy_to_matrix(*rpy)
            world_verts = verts @ rot.T + np.array(xyz)

            link_min = np.minimum(link_min, world_verts.min(axis=0))
            link_max = np.maximum(link_max, world_verts.max(axis=0))
            found_mesh = True

        old_collisions = link.findall('collision')
        for col in old_collisions:
            link.remove(col)
        total_removed += len(old_collisions)

        if not found_mesh:
            continue

        center = (link_min + link_max) / 2.0
        size = np.maximum(link_max - link_min, 1e-4)

        collision = ET.SubElement(link, 'collision')
        collision.append(ET.Comment(
            ' Simplified box collision (extent of visual mesh vertices); '
            'see scripts/simplify_collisions.py '
        ))
        origin = ET.SubElement(collision, 'origin')
        origin.set('xyz', f'{center[0]:.8g} {center[1]:.8g} {center[2]:.8g}')
        origin.set('rpy', '0 0 0')
        geometry = ET.SubElement(collision, 'geometry')
        box = ET.SubElement(geometry, 'box')
        box.set('size', f'{size[0]:.8g} {size[1]:.8g} {size[2]:.8g}')
        total_added += 1

        indent_xml(link, level=1)

        print(
            f'[INFO] {link_name}: {len(old_collisions)} mesh collisions -> '
            f'1 box {size[0]:.3f} x {size[1]:.3f} x {size[2]:.3f} m'
        )

    tree.write(URDF_FILE, encoding='unicode', xml_declaration=True)
    print(
        f'[DONE] Removed {total_removed} mesh collisions, '
        f'added {total_added} box collisions.'
    )


if __name__ == '__main__':
    main()
