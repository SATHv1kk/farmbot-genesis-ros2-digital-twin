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
Generate a kinematic URDF from the detailed FarmBot Genesis URDF.

Reads the existing detailed URDF, categorizes all parts by axis,
and produces a simplified URDF with 3 prismatic joints forming
the kinematic tree: base → x_axis → y_axis → z_axis.
"""

from copy import deepcopy
import os
import shutil
import sys
import xml.etree.ElementTree as ET

URDF_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DETAILED_URDF = os.path.join(URDF_DIR, 'farmbot_genesis_detailed.urdf')
ORIGINAL_URDF = os.path.join(URDF_DIR, 'farmbot_genesis.urdf')
OUTPUT_URDF = os.path.join(URDF_DIR, 'farmbot_genesis.urdf')

PREFIX = 'supporting_infrastructure__infrastructure_default_farmbot_default'
PREFIX_UNDERSCORE = PREFIX + '_'

# ---------------------------------------------------------------------------
# Categorization
# ---------------------------------------------------------------------------

# Priority: Z_AXIS > Y_AXIS > X_AXIS > BASE > COMMON
# Check is ordered: specific multi-word before single-word

BASE_KEYWORDS = [
    'track_extrusion', 'electronics_box_gasket', 'electronics_box_latch',
    'electronics_box_lid', 'electronics_box', 'farmduino', 'raspberry',
    'power_supply_cable', 'power_supply', 'garden_faucet', 'garden_hose',
    'push_button_o_ring', 'push_button_nut', 'push_button_wiring',
    'push_button', 'led_pin_header', 'led_strip',
    'extension_cord', 'pressure_regulator', 'npt_to_barb',
    'inline_air_filter', 'outlet_box', 'jumper_link', 'microsd_to_sd',
    'microsd_card_case', 'microsd_card',
    'realtime_clock', 'power_cord', 'pi_adapter', 'pi_power',
    'farmduino_data', 'fuse___',
    'seed_tray', 'seed_trough_holder_mount_plate',
    'seed_trough_holder', 'seed_trough',
    'getting_started', 'main_carton', 'polystrap',
    'short_board', 'long_board', 'soil__',
    '25mm_wood_screw', '75mm_wood_screw', 'flathead_wood_screw',
    'plastic_bag', 'post__',
    '2mm_hex_key', '2_5mm_hex_key', '3mm_hex_driver',
    '8mm_thin_wrench', 't10_bit', 't25_bit',
    '1_x_2_2_54mm_shunt',
    'camera_calibration_backing', 'luer_lock_needle_cover',
    'luer_lock_needle', '3_4__rubber_gasket',
    'toolbay', '20mm_x_20mm_x_1000mm_extrusion',
    'utilities_post', 'o_ring',
    'pipe',
]

Y_AXIS_KEYWORDS = [
    'cross_slide_plate', 'y_axis_belt',
    '80mm_vertical_motor_housing', '40mm_vertical_cable_carrier_support',
    '40mm_cable_carrier_spacer_block',
    'motor_cable___y__', 'encoder_cable___y__',
    'water_tube__y___',
]

Z_AXIS_KEYWORDS = [
    'leadscrew_block', 'leadscrew',
    'z_axis_hardstop', 'z_axis_motor_mount',
    '5mm_to_8mm_shaft_coupler',
    'utm_magnet', 'utm_pcb',
    'solenoid_valve_cable', 'solenoid_mount', 'solenoid_valve',
    'vacuum_pump_cover', 'vacuum_pump_mount',
    'vacuum_pump__connector', 'vacuum_tube___z',
    '90_degree_barb',
    'm5_to_luer_lock_adapter', 'm5_barb',
    'seeder', 'seed_bin',
    'rotary_tool_bottom', 'rotary_tool_chuck_adapter',
    'rotary_tool_chuck', 'rotary_tool_m5_shaft_adapter',
    'rotary_tool_motor', 'rotary_tool_pcb',
    'rotary_tool_shaft_extension', 'rotary_tool_top',
    'rotary_tool_trimmer_mount', 'rotary_tool_trimmer_',
    'rotary_tool_trimmer',
    'watering_nozzle', 'soil_sensor',
    'camera_mount_half', 'camera__farmbot_default',
    'camera_cable__farmbot_default',
    'zz_encoder_cable', 'zz_motor_cable',
    'motor_cable___zy__', 'encoder_cable___zy__',
    'water_tube__z', 'utm_cable___z', 'utm_cable___y__',
    'vacuum_pump_cable___y__',
    'utm',  # broad match - catches utm, utm_magnet, utm_cable, etc
    'tool_magnet', 'm5_barb', 'm5_to_luer_lock_adapter',
]

X_AXIS_KEYWORDS = [
    'gantry_column', 'gantry_main_beam', 'gantry_joining_bar',
    'gantry_wheel_plate', 'left_gantry_corner_bracket',
    'right_gantry_corner_bracket',
    'x_axis_belt', 'x_axis_cc_mount',
    '75mm_horizontal_motor_housing',
    'horizontal_cable_carrier_support_extrusion',
    'cable_carrier_end_1', 'cable_carrier_end_2',
    'cable_carrier_end_', 'belt_clip', 'belt_sleeve',
    'motor_cable___x1', 'motor_cable___x2',
    'encoder_cable___x1', 'encoder_cable___x2',
    'water_tube__x___',
    'nema_17_stepper_motor',  # catches _motor (X1), _motor_2 (X2) only
    'rotary_encoder',  # catches _encoder (X1), _encoder_2 (X2), see filter
    'gt2_pulley___20_tooth',  # X pulleys - pre-filtered to avoid _2/_3
]

COMMON_KEYWORDS = [
    'v_wheel', 'bearing', 'cable_carrier_link', 'cable_carrier_tab',
    'm3_', 'm4_', 'm5_',
    'nut_bar', 'eccentric_spacer', 'supergland_half',
    'pogo_pin', 'zip_tie', 'encoder_base',
]


def extract_part_name(link_name: str) -> str:
    """Return the part name with the URDF prefix stripped."""
    if link_name.startswith(PREFIX_UNDERSCORE):
        return link_name[len(PREFIX_UNDERSCORE):]
    if link_name == PREFIX:
        return '__infrastructure_base__'
    return link_name


def categorize_part(part_name: str) -> str:
    """
    Categorize a part name using keyword matching.

    Priority order (first match wins):
      1. Y-specific cable patterns (___y__ / __y___)
      2. Z-specific cable/part patterns (___z / _zz_ / ___zy__)
      3. Numbered motor/encoder variants for Y (motor_3, encoder_3)
      4. Numbered motor/encoder variants for Z (motor_4, encoder_4)
      5. Numbered pulley variants (gt2_pulley..._2 for Y, _3 for Z)
      6. Z_AXIS main keywords (before COMMON to avoid catching tool parts)
      7. COMMON fasteners / generic hardware
      8. Y_AXIS main keywords
      9. X_AXIS main keywords (remaining motors, encoders, pulleys)
     10. BASE infrastructure keywords
     11. Default: X_AXIS
    """
    name = part_name.lower()

    # --- Step 1: Y-specific cable patterns (before Z keywords) ---
    if '___y__' in name or '__y___' in name:
        if 'utm' in name or 'vacuum_pump' in name:
            return 'Z_AXIS'
        return 'Y_AXIS'

    # --- Step 2: Z-specific patterns ---
    if ('___z_' in name or '__z_' in name or '_zz_' in name
            or '___zy__' in name):
        return 'Z_AXIS'

    # --- Step 3: Numbered motor/encoder -> Y axis ---
    #   nema_17_stepper_motor_3, rotary_encoder_3, encoder_base_3
    if 'nema_17_stepper_motor_3' in name or 'rotary_encoder_3' in name:
        return 'Y_AXIS'

    # --- Step 4: Numbered motor/encoder -> Z axis ---
    if 'nema_17_stepper_motor_4' in name or 'rotary_encoder_4' in name:
        return 'Z_AXIS'

    # --- Step 5: Numbered pulleys ---
    if 'gt2_pulley___20_tooth_2' in name:
        return 'Y_AXIS'
    if 'gt2_pulley___20_tooth_3' in name:
        return 'Z_AXIS'

    # --- Step 6: Z_AXIS main parts (before COMMON to avoid conflicts) ---
    for kw in Z_AXIS_KEYWORDS:
        if kw in name:
            return 'Z_AXIS'

    # --- Step 7: COMMON hardware (fasteners, bearings, wheels, etc.) ---
    for kw in COMMON_KEYWORDS:
        if kw in name:
            return 'COMMON'

    # --- Step 8: Y_AXIS main parts ---
    for kw in Y_AXIS_KEYWORDS:
        if kw in name:
            return 'Y_AXIS'

    # --- Step 9: X_AXIS main parts ---
    for kw in X_AXIS_KEYWORDS:
        if kw in name:
            return 'X_AXIS'

    # --- Step 10: BASE infrastructure ---
    for kw in BASE_KEYWORDS:
        if kw in name:
            return 'BASE'

    # --- Step 11: Fallback ---
    if any(k in name for k in ['screw', 'washer', 'setscrew']):
        return 'COMMON'
    if any(k in name for k in ['cable_carrier', 'v_wheel']):
        return 'COMMON'
    if 'nut' in name and 'nema' not in name:
        return 'COMMON'

    return 'X_AXIS'


# ---------------------------------------------------------------------------
# Math helpers
# ---------------------------------------------------------------------------

def parse_origin(elem) -> tuple:
    """Return (xyz_tuple, rpy_tuple) parsed from an <origin> element."""
    xyz_str = elem.get('xyz', '0 0 0')
    rpy_str = elem.get('rpy', '0 0 0')
    xyz = tuple(float(v) for v in xyz_str.split())
    rpy = tuple(float(v) for v in rpy_str.split())
    return xyz, rpy


def sub_xyz(a, b):
    """Return component-wise subtraction a - b."""
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def add_xyz(a, b):
    """Return component-wise addition a + b."""
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def is_near_zero_rpy(rpy):
    """Return True if all roll/pitch/yaw values are near zero."""
    return all(abs(v) < 1e-6 for v in rpy)


def fmt_xyz(xyz):
    """Return space-separated string from an (x, y, z) tuple."""
    return f'{xyz[0]:.8g} {xyz[1]:.8g} {xyz[2]:.8g}'


def fmt_rpy(rpy):
    """Return space-separated string from a (roll, pitch, yaw) tuple."""
    return f'{rpy[0]:.8g} {rpy[1]:.8g} {rpy[2]:.8g}'


# ---------------------------------------------------------------------------
# XML helpers
# ---------------------------------------------------------------------------

def indent_xml(elem, level=0, spaces='  '):
    """Pretty-print XML element tree (in-place)."""
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


def new_element(tag, attrib=None, text=None, **extra):
    """Create an XML element with optional attributes and text."""
    el = ET.Element(tag, attrib=(attrib or {}))
    if text is not None:
        el.text = text
    for k, v in extra.items():
        el.set(k, str(v))
    return el


def new_origin(xyz=(0, 0, 0), rpy=(0, 0, 0)):
    """Create an <origin> element with given translation and rotation."""
    return new_element('origin', xyz=fmt_xyz(xyz), rpy=fmt_rpy(rpy))


def new_limit(effort, velocity, lower=None, upper=None):
    """Create a <limit> element for a joint."""
    attrs = {'effort': str(effort), 'velocity': str(velocity)}
    if lower is not None:
        attrs['lower'] = str(lower)
    if upper is not None:
        attrs['upper'] = str(upper)
    return new_element('limit', **attrs)


def new_inertial(mass, ixx, iyy, izz, xyz=(0, 0, 0), rpy=(0, 0, 0)):
    """Create an <inertial> element with mass and inertia values."""
    iv = new_element('inertial')
    iv.append(new_origin(xyz, rpy))
    iv.append(new_element('mass', value=str(mass)))
    iv.append(
        new_element('inertia', ixx=str(ixx), ixy='0', ixz='0',
                    iyy=str(iyy), iyz='0', izz=str(izz))
    )
    return iv


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    """Read the detailed URDF and generate a simplified 3-axis URDF."""
    print('=' * 70)
    print('  FarmBot Genesis URDF Generator')
    print('=' * 70)

    # 1. Determine source file
    if os.path.exists(ORIGINAL_URDF) and not os.path.exists(DETAILED_URDF):
        print(f'[INFO] First run: backing up original to {DETAILED_URDF}')
        shutil.copy2(ORIGINAL_URDF, DETAILED_URDF)

    if os.path.exists(DETAILED_URDF):
        source_urdf = DETAILED_URDF
        print(f'[INFO] Reading from detailed backup: {DETAILED_URDF}')
    elif os.path.exists(ORIGINAL_URDF):
        source_urdf = ORIGINAL_URDF
        print(f'[INFO] Reading from original: {ORIGINAL_URDF}')
    else:
        print('[ERROR] No source URDF found!')
        sys.exit(1)

    # 2. Parse the source URDF
    print(f'[INFO] Parsing {source_urdf} ...')
    tree = ET.parse(source_urdf)
    root = tree.getroot()

    # Gather link data:
    #   link_name -> (inertial, visual_elements, collision_elements)
    link_map = {}
    for link_elem in root.findall('link'):
        name = link_elem.get('name')
        visuals = []
        collisions = []
        inertial_elem = link_elem.find('inertial')
        for v in link_elem.findall('visual'):
            visuals.append(v)
        for c in link_elem.findall('collision'):
            collisions.append(c)
        link_map[name] = {
            'inertial': (
                deepcopy(inertial_elem) if inertial_elem is not None
                else None
            ),
            'visuals': [deepcopy(v) for v in visuals],
            'collisions': [deepcopy(c) for c in collisions],
        }

    # Gather joint data: child_link -> (parent_link, origin_xyz, origin_rpy)
    joint_map = {}
    for joint_elem in root.findall('joint'):
        child = joint_elem.find('child').get('link')
        parent = joint_elem.find('parent').get('link')
        origin_elem = joint_elem.find('origin')
        if origin_elem is not None:
            xyz, rpy = parse_origin(origin_elem)
        else:
            xyz, rpy = (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
        joint_map[child] = (parent, xyz, rpy)

    print(f'[INFO] Found {len(link_map)} links, {len(joint_map)} joints')

    # 3. Categorize links
    categories = {
        'BASE': [], 'X_AXIS': [], 'Y_AXIS': [], 'Z_AXIS': [], 'COMMON': [],
    }

    # Reference origins for axis links
    # These are the joint origins of representative parts that sit at the
    # "base" of each axis in the world frame relative to
    # supporting_infrastructure.
    y_ref_xyz = None   # cross_slide_plate joint origin
    z_ref_xyz = None   # z_axis_motor_mount joint origin

    for link_name in sorted(link_map.keys()):
        if link_name == 'base_link':
            categories['BASE'].append(link_name)
            continue
        if link_name == PREFIX:
            categories['BASE'].append(link_name)
            continue

        part_name = extract_part_name(link_name)
        cat = categorize_part(part_name)

        # Track reference origins
        if cat in ('Y_AXIS', 'Z_AXIS') and link_name in joint_map:
            # Check for specific reference parts
            if 'cross_slide_plate' in part_name and y_ref_xyz is None:
                y_ref_xyz = joint_map[link_name][1]
            if 'z_axis_motor_mount' in part_name and z_ref_xyz is None:
                z_ref_xyz = joint_map[link_name][1]

        categories[cat].append(link_name)

    # Compute z_axis_joint origin in y_axis_link frame
    if y_ref_xyz is None:
        y_ref_xyz = (-0.39125, -0.2225, 0.516325)
    if z_ref_xyz is None:
        z_ref_xyz = (-0.33625, -0.2335, 0.968825)

    z_joint_origin_xyz = sub_xyz(z_ref_xyz, y_ref_xyz)

    print('[INFO] Reference origins:')
    print(f'       y_axis_link (global): {fmt_xyz(y_ref_xyz)}')
    print(f'       z_axis_link (global): {fmt_xyz(z_ref_xyz)}')
    print(
        f'       z_axis_joint (in y_axis_link): '
        f'{fmt_xyz(z_joint_origin_xyz)}'
    )

    counts = {k: len(v) for k, v in categories.items()}
    print(f'[INFO] Link counts: {counts}')

    # 4. Build new URDF
    new_root = ET.Element('robot', name='farmbot_genesis')
    new_root.text = '\n  '

    # --- base_link (dummy) ---
    # Deliberately massless: KDL does not support inertia on the root
    # link, and the fixed child links carry the real mass.
    link_base = ET.SubElement(new_root, 'link', name='base_link')
    link_base.append(ET.Comment(' Dummy base link (massless root) '))
    link_base.tail = '\n\n  '

    # --- supporting_infrastructure ---
    link_support = ET.SubElement(
        new_root, 'link', name='supporting_infrastructure')
    link_support.append(ET.Comment(
        ' Stationary infrastructure + base parts + common hardware '))
    link_support.tail = '\n\n  '

    sup_mass = 30.0
    sup_inertial = new_inertial(sup_mass, 5.0, 5.0, 5.0)
    link_support.append(sup_inertial)

    # --- x_axis_link ---
    x_mass = 15.0
    link_x = ET.SubElement(new_root, 'link', name='x_axis_link')
    link_x.append(ET.Comment(' Gantry / X-axis moving assembly '))
    link_x.tail = '\n\n  '
    # Approximated as a slender beam along Y (the gantry main beam);
    # values keep the inertia tensor physically consistent
    # (each principal moment <= sum of the other two).
    x_inertial = new_inertial(x_mass, 2.0, 0.1, 2.0)
    link_x.append(x_inertial)

    # --- y_axis_link ---
    y_mass = 5.0
    link_y = ET.SubElement(new_root, 'link', name='y_axis_link')
    link_y.append(ET.Comment(' Cross-slide / Y-axis moving assembly '))
    link_y.tail = '\n\n  '
    # Compact cross-slide, approximated as a ~0.3 m box.
    y_inertial = new_inertial(y_mass, 0.06, 0.05, 0.05)
    link_y.append(y_inertial)

    # --- z_axis_link ---
    z_mass = 3.0
    link_z = ET.SubElement(new_root, 'link', name='z_axis_link')
    link_z.append(ET.Comment(' Toolhead / Z-axis moving assembly '))
    link_z.tail = '\n\n  '
    # Approximated as a slender vertical column (~1 m along Z).
    z_inertial = new_inertial(z_mass, 0.25, 0.25, 0.01)
    link_z.append(z_inertial)

    # Map category to target link element
    cat_to_link = {
        'BASE': link_support,
        'COMMON': link_support,
        'X_AXIS': link_x,
        'Y_AXIS': link_y,
        'Z_AXIS': link_z,
    }

    # Which links need offset subtraction?
    #   X_AXIS: offset = joint_xyz          (same frame as base)
    #   Y_AXIS: offset = joint_xyz - y_ref
    #   Z_AXIS: offset = joint_xyz - z_ref
    cat_offsets = {
        'BASE': None,
        'COMMON': None,
        'X_AXIS': (0, 0, 0),
        'Y_AXIS': y_ref_xyz,
        'Z_AXIS': z_ref_xyz,
    }

    # Build the combined links
    for cat in ['BASE', 'COMMON', 'X_AXIS', 'Y_AXIS', 'Z_AXIS']:
        target_link = cat_to_link[cat]
        offset_ref = cat_offsets.get(cat)

        for link_name in categories[cat]:
            if link_name in ('base_link', PREFIX):
                continue

            info = link_map[link_name]
            parent, j_xyz, j_rpy = joint_map.get(
                link_name, (None, (0, 0, 0), (0, 0, 0))
            )

            # Compute local xyz in the axis link frame
            if offset_ref is not None:
                local_xyz = sub_xyz(j_xyz, offset_ref)
            else:
                local_xyz = j_xyz

            for vis in info['visuals']:
                clone = deepcopy(vis)
                orig_elem = clone.find('origin')
                if orig_elem is not None:
                    v_xyz, v_rpy = parse_origin(orig_elem)
                    combined_xyz = add_xyz(local_xyz, v_xyz)
                    combined_rpy = (
                        j_rpy if not is_near_zero_rpy(j_rpy) else v_rpy
                    )
                    orig_elem.set('xyz', fmt_xyz(combined_xyz))
                    orig_elem.set('rpy', fmt_rpy(combined_rpy))
                else:
                    orig_elem = new_origin(local_xyz, j_rpy)
                    clone.insert(0, orig_elem)

                target_link.append(clone)

            for col in info['collisions']:
                clone = deepcopy(col)
                orig_elem = clone.find('origin')
                if orig_elem is not None:
                    v_xyz, v_rpy = parse_origin(orig_elem)
                    combined_xyz = add_xyz(local_xyz, v_xyz)
                    combined_rpy = (
                        j_rpy if not is_near_zero_rpy(j_rpy) else v_rpy
                    )
                    orig_elem.set('xyz', fmt_xyz(combined_xyz))
                    orig_elem.set('rpy', fmt_rpy(combined_rpy))
                else:
                    orig_elem = new_origin(local_xyz, j_rpy)
                    clone.insert(0, orig_elem)
                target_link.append(clone)

    # --- Joints ---

    # base_link → supporting_infrastructure (fixed)
    j_base_support = ET.SubElement(
        new_root, 'joint',
        name='base_to_supporting_infrastructure',
        type='fixed',
    )
    j_base_support.tail = '\n\n  '
    j_base_support.append(ET.Comment(' Base to stationary infrastructure '))
    j_base_support.append(new_origin((0, 0, 0), (0, 0, 0)))
    parent_elem = ET.SubElement(j_base_support, 'parent', link='base_link')
    parent_elem.tail = '\n    '
    child_elem = ET.SubElement(
        j_base_support, 'child', link='supporting_infrastructure')
    child_elem.tail = '\n  '

    # base_link → x_axis_joint (prismatic)
    j_x = ET.SubElement(
        new_root, 'joint',
        name='x_axis_joint',
        type='prismatic',
    )
    j_x.tail = '\n\n  '
    j_x.append(ET.Comment(' Gantry X-axis (along tracks) '))
    j_x.append(new_origin((0, 0, 0), (0, 0, 0)))
    parent_elem = ET.SubElement(j_x, 'parent', link='base_link')
    parent_elem.tail = '\n    '
    child_elem = ET.SubElement(j_x, 'child', link='x_axis_link')
    child_elem.tail = '\n    '
    j_x.append(new_element('axis', xyz='1 0 0'))
    j_x.append(new_limit(effort=1000, velocity=0.1, lower=0, upper=2.7))
    j_x.append(ET.Comment(' 0 to 2.7 m (TODO: verify axis_length) '))

    # x_axis_link → y_axis_joint (prismatic)
    j_y = ET.SubElement(
        new_root, 'joint',
        name='y_axis_joint',
        type='prismatic',
    )
    j_y.tail = '\n\n  '
    j_y.append(ET.Comment(' Cross-slide Y-axis (along gantry beam) '))
    j_y.append(new_origin(y_ref_xyz, (0, 0, 0)))
    parent_elem = ET.SubElement(j_y, 'parent', link='x_axis_link')
    parent_elem.tail = '\n    '
    child_elem = ET.SubElement(j_y, 'child', link='y_axis_link')
    child_elem.tail = '\n    '
    j_y.append(new_element('axis', xyz='0 1 0'))
    j_y.append(new_limit(effort=1000, velocity=0.1, lower=0, upper=1.3))
    j_y.append(ET.Comment(' 0 to 1.3 m (TODO: verify axis_length) '))

    # y_axis_link → z_axis_joint (prismatic)
    j_z = ET.SubElement(
        new_root, 'joint',
        name='z_axis_joint',
        type='prismatic',
    )
    j_z.tail = '\n\n  '
    j_z.append(ET.Comment(' Toolhead Z-axis (vertical) '))
    j_z.append(new_origin(z_joint_origin_xyz, (0, 0, 0)))
    parent_elem = ET.SubElement(j_z, 'parent', link='y_axis_link')
    parent_elem.tail = '\n    '
    child_elem = ET.SubElement(j_z, 'child', link='z_axis_link')
    child_elem.tail = '\n    '
    j_z.append(new_element('axis', xyz='0 0 1'))
    j_z.append(new_limit(effort=500, velocity=0.02, lower=-0.4, upper=0))
    j_z.append(ET.Comment(' -0.4 to 0 m (downward, leadscrew ~5x slower) '))

    # z_axis_link → tool_link (fixed, end-effector frame)
    link_tool = ET.SubElement(new_root, 'link', name='tool_link')
    link_tool.tail = '\n\n  '
    link_tool.append(ET.Comment(
        ' End-effector reference frame (URDF requires inertial) '))
    tool_inertial = new_inertial(0.1, 0.001, 0.001, 0.001)
    link_tool.append(tool_inertial)

    j_tool = ET.SubElement(
        new_root, 'joint',
        name='joint_tool',
        type='fixed',
    )
    j_tool.tail = '\n\n  '
    j_tool.append(ET.Comment(' UTM tool mount (end-effector) '))
    j_tool.append(new_origin((0, 0, 0), (0, 0, 0)))
    parent_elem = ET.SubElement(j_tool, 'parent', link='z_axis_link')
    parent_elem.tail = '\n    '
    child_elem = ET.SubElement(j_tool, 'child', link='tool_link')
    child_elem.tail = '\n  '

    # 5. Write output
    indent_xml(new_root)
    tree = ET.ElementTree(new_root)
    tree.write(
        OUTPUT_URDF,
        encoding='unicode',
        xml_declaration=True,
    )

    # Fix mesh paths: package://assets/ -> .../farmbot_description/assets/
    with open(OUTPUT_URDF, 'r') as f:
        content = f.read()
    content = content.replace(
        'package://assets/', 'package://farmbot_description/assets/')
    with open(OUTPUT_URDF, 'w') as f:
        f.write(content)

    print(f'[INFO] Wrote kinematic URDF to {OUTPUT_URDF}')
    print(f'[INFO] Original preserved at {DETAILED_URDF}')
    print('[DONE]')


if __name__ == '__main__':
    main()
