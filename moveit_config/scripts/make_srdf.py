#!/usr/bin/env python3
"""Make the config/baxter*.srdf files from config/baxter.srdf.in and the original configuration.

The pairs of robot parts that never need a collision check come from the MoveIt
configuration that Rethink Robotics and MoveIt shipped for Baxter (ROS 1,
moveit_robots/baxter/baxter_moveit_config 1.1.1), kept unchanged in
reference/moveit_robots_baxter/: the pairs of baxter_base.srdf.xacro and, for both
arms, of default_gripper.srdf.xacro, and for an arm with Rethink's electric gripper those
of rethink_electric_gripper.srdf.xacro (as the original's baxter.srdf.xacro puts them
together). Pairs naming parts that the robot model does not have are left out.
To them come the pairs of baxter.srdf.in (reason "User"), which say why they are needed.

One file per choice of grippers, each for its robot model:

  baxter.srdf                 no electric gripper      robot model urdf/baxter.urdf
  baxter_left_gripper.srdf    on the left arm          xacro/baxter_grippers.urdf.xacro
  baxter_right_gripper.srdf   on the right arm           with left_electric_gripper and
  baxter_both_grippers.srdf   on both arms               right_electric_gripper

Run it after changing baxter.srdf.in, in the MoveIt image (it needs the robot model and
xacro), from the repository's root:
    docker compose run --rm -v ./moveit_config:/cfg moveit python3 /cfg/scripts/make_srdf.py
"""

import pathlib
import re
import subprocess
import sys
import xml.etree.ElementTree as ElementTree

HERE = pathlib.Path(__file__).resolve().parent.parent
ORIGINAL = HERE / 'reference' / 'moveit_robots_baxter' / 'baxter_moveit_config' / 'config'
PAIR = re.compile(r'<disable_collisions\s+link1="([^"]+)"\s+link2="([^"]+)"\s+reason="([^"]+)"\s*/>')
# Where the hand-written pairs start in baxter.srdf.in: everything from there on is
# replaced by the full list.
OWN_PAIRS = '  <!-- Pairs switched off by hand'
EDIT_NOTE = ('     This file is the hand-written part. config/baxter.srdf is made from it by\n'
             "     scripts/make_srdf.py, which adds the original configuration's pairs of parts that are\n"
             '     not checked. Edit this file, then run the script. -->')
MADE_NOTE = ('     GENERATED from config/baxter.srdf.in by scripts/make_srdf.py: do not edit this\n'
             '     file; change baxter.srdf.in and run the script. -->')
# file name: (electric gripper on the left arm, on the right arm)
VARIANTS = {
    'baxter.srdf': (False, False),
    'baxter_left_gripper.srdf': (True, False),
    'baxter_right_gripper.srdf': (False, True),
    'baxter_both_grippers.srdf': (True, True),
}


def original_pairs(left_gripper, right_gripper):
    """{(link1, link2): reason} of the original configuration, for one choice of grippers."""
    pairs = {}

    def add(text):
        for link1, link2, reason in PAIR.findall(text):
            pairs.setdefault(tuple(sorted((link1, link2))), reason)

    add((ORIGINAL / 'baxter_base.srdf.xacro').read_text(encoding='utf-8'))
    default = (ORIGINAL / 'default_gripper.srdf.xacro').read_text(encoding='utf-8')
    electric = (ORIGINAL / 'rethink_electric_gripper.srdf.xacro').read_text(encoding='utf-8')
    for side, mounted in (('left', left_gripper), ('right', right_gripper)):
        add(default.replace('${side}', side))
        if mounted:
            add(electric.replace('${side}', side).replace('${gripper_side}', side[0]))
    return pairs


def robot_model(left_gripper, right_gripper):
    """The robot model (URDF text) for one choice of grippers."""
    from ament_index_python.packages import get_package_share_directory
    share = pathlib.Path(get_package_share_directory('baxter_description'))
    if not (left_gripper or right_gripper):
        return (share / 'urdf' / 'baxter.urdf').read_text(encoding='utf-8')
    return subprocess.run(
        ['xacro', str(share / 'xacro' / 'baxter_grippers.urdf.xacro'),
         f'left_electric_gripper:={str(left_gripper).lower()}',
         f'right_electric_gripper:={str(right_gripper).lower()}'],
        check=True, capture_output=True, text=True).stdout


def write(name, left_gripper, right_gripper, source):
    model = ElementTree.fromstring(robot_model(left_gripper, right_gripper))
    links = {link.get('name') for link in model.findall('link')}

    everything = original_pairs(left_gripper, right_gripper)
    pairs = {pair: reason for pair, reason in everything.items() if set(pair) <= links}
    left_out = len(everything) - len(pairs)
    own = {tuple(sorted((a, b))): reason for a, b, reason in PAIR.findall(source)}
    added = {pair: reason for pair, reason in own.items() if pair not in pairs}
    pairs.update(added)

    grippers = {(False, False): 'no electric gripper', (True, False): 'the electric gripper on the left arm',
                (False, True): 'the electric gripper on the right arm',
                (True, True): 'the electric gripper on both arms'}[(left_gripper, right_gripper)]
    head = source[:source.index(OWN_PAIRS)].rstrip() + '\n'
    head = head.replace(EDIT_NOTE, MADE_NOTE)
    lines = [head,
             f'  <!-- For Baxter with {grippers}. Pairs of parts that are never checked against each\n'
             f'       other: {len(pairs) - len(added)} from the original configuration (reference/moveit_robots_baxter;\n'
             f'       {left_out} left out because they name parts this robot model does not have) and\n'
             f'       {len(added)} from baxter.srdf.in, where each says why. -->']
    for (link1, link2), reason in sorted(pairs.items()):
        lines.append(f'  <disable_collisions link1="{link1}" link2="{link2}" reason="{reason}"/>')
    lines.append('</robot>\n')
    (HERE / 'config' / name).write_text('\n'.join(lines), encoding='utf-8', newline='\n')
    print(f'config/{name} written: {len(pairs)} pairs ({len(pairs) - len(added)} original, '
          f'{len(added)} from baxter.srdf.in, {left_out} original ones left out)')


def main():
    source = (HERE / 'config' / 'baxter.srdf.in').read_text(encoding='utf-8')
    if OWN_PAIRS not in source or EDIT_NOTE not in source:
        sys.exit('baxter.srdf.in has changed where this script expects it; adapt make_srdf.py')
    for name, (left_gripper, right_gripper) in VARIANTS.items():
        write(name, left_gripper, right_gripper, source)


if __name__ == '__main__':
    main()
