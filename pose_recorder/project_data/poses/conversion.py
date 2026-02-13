#!/usr/bin/env python3
import os
import glob
import yaml

def convert_file(path):
    with open(path, 'r') as f:
        data = yaml.safe_load(f)

    # already in new format?
    hdr = data.get('header', {})
    if any(k.startswith('robot_') for k in hdr.keys()):
        return

    # pick up the old header fields
    new = {'header': {}}
    for key in ('pose_name', 'description', 'recorded', 'last_edited'):
        if key in hdr:
            new['header'][key] = hdr[key]

    # find all robot‐sections (top‐level keys except header, relative_*, transformation_matrix)
    robot_keys = [
        k for k in data.keys()
        if k not in ('header', 'transformation_matrix') and not k.startswith('relative')
    ]
    robot_keys.sort()

    # add robot_N entries to header
    for i, robot in enumerate(robot_keys):
        new['header'][f'robot_{i+1}'] = robot

    # convert each robot section
    for i, robot in enumerate(robot_keys):
        old_sec = data[robot]
        # joint angles in degrees
        jp_deg = old_sec.get('joint_pose', {}).get('joint_angles_degrees', {})

        # cartesian pose: strip "<robot>/" prefix from frame names
        cp = old_sec.get('cartesian_pose', {})
        def strip_prefix(s):
            return s.split('/', 1)[1] if '/' in s else s

        base = strip_prefix(cp.get('base_frame_name', ''))
        ee   = strip_prefix(cp.get('ee_frame_name',   ''))
        trans = cp.get('ee_translation_m', {})
        rot   = cp.get('ee_rotation_euler_degrees', {})

        new[f'robot_{i+1}'] = {
            'joint_pose': {
                'joint_angles_degrees': jp_deg
            },
            'cartesian_pose': {
                'base_frame_name': base,
                'ee_frame_name':   ee,
                'ee_translation_m':             trans,
                'ee_rotation_euler_degrees':    rot
            }
        }

    # overwrite the file in-place
    with open(path, 'w') as f:
        yaml.safe_dump(new, f, sort_keys=False, indent=2)

    print(f"Converted {os.path.basename(path)}")

def main():
    script_dir = os.path.dirname(os.path.realpath(__file__))
    for pattern in ('*.yaml', '*.yml'):
        for fn in glob.glob(os.path.join(script_dir, pattern)):
            convert_file(fn)

if __name__ == '__main__':
    main()
