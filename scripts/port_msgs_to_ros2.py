#!/usr/bin/env python3
"""Generate ROS 2 message packages from the ROS 1 message packages of the robots.

Reads   msgs/ros1/<package>/   (copied unchanged from their makers: Baxter's from
                                Rethink's baxter_common v1.2.0, the version of the lab's
                                robots; the Mobility Base's from Dataspeed's SDK)
Writes  msgs/ros2/<package>/   (ROS 2 interface package + ros1_bridge mapping rules)

ROS 2 is stricter than ROS 1 in four ways that matter for Baxter's messages:
  1. Field names must be lowercase snake_case: isPanning -> is_panning
  2. ROS 1 built-in time types are messages in ROS 2: time -> builtin_interfaces/Time
  3. The Header shorthand must be spelled out: Header -> std_msgs/Header
  4. String constant values must be quoted: CMD_GRIP = grip -> CMD_GRIP="grip"

Only (1) changes what ros1_bridge sees. For every message or service with a renamed
field, a rule in mapping_rules.yaml tells the bridge which ROS 1 field belongs to
which ROS 2 field. Once a rule exists, the bridge copies ONLY the fields it lists,
so the rules list every field, including unchanged ones.

Run from the repository root:   python3 scripts/port_msgs_to_ros2.py
The output is generated: change this script and re-run it rather than editing
msgs/ros2 by hand.
"""

import re
import shutil
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SRC = ROOT / "msgs" / "ros1"
DST = ROOT / "msgs" / "ros2"
# package: (version of msgs/ros1/<package>, where it is from, its author)
BAXTER = ("1.2.0", "the Baxter SDK (baxter_common v1.2.0)", "Rethink Robotics Inc.")
PACKAGES = {
    "baxter_core_msgs": BAXTER,
    "baxter_maintenance_msgs": BAXTER,
    "mobility_base_core_msgs": ("1.0.6", "Dataspeed's Mobility Base SDK (mobility_base_ros 1.0.6)",
                                "Dataspeed Inc."),
}

TYPE_MAP = {
    "time": "builtin_interfaces/Time",
    "duration": "builtin_interfaces/Duration",
    "Header": "std_msgs/Header",
}
ROS2_FIELD_NAME = re.compile(r"^[a-z][a-z0-9]*(_[a-z0-9]+)*$")
CONSTANT = re.compile(r"^\s*(\S+)\s+([A-Za-z_][A-Za-z0-9_]*)\s*=\s*(.*?)\s*$")
FIELD = re.compile(r"^\s*(\S+)\s+(\S+)\s*$")


def snake_case(name):
    return re.sub(r"(?<=[a-z0-9])([A-Z])", r"_\1", name).lower()


def split_array(type_):
    """'float64[]' -> ('float64', '[]'), 'bool' -> ('bool', '')."""
    i = type_.find("[")
    return (type_, "") if i < 0 else (type_[:i], type_[i:])


def convert_file(path):
    """Convert one .msg/.srv file.

    Returns (new_text, sections, notes): sections is a list (one per message part:
    message, or request + response) of [(ros1_field, ros2_field), ...];
    notes lists the changes made, for the file header and the summary.
    """
    out, sections, notes = [], [[]], []
    for raw in path.read_text(encoding="utf-8").splitlines():
        if raw.strip() == "---":
            out.append("---")
            sections.append([])
            continue

        # String constants: in ROS 1 everything after '=' is the value (a '#' is
        # part of it, not a comment), so handle them before splitting comments.
        m = CONSTANT.match(raw)
        if m and m.group(1) == "string":
            value = m.group(3).replace("\\", "\\\\").replace('"', '\\"')
            out.append(f'string {m.group(2)}="{value}"')
            notes.append(f"quoted string constant {m.group(2)}")
            continue

        code, hash_, comment = raw.partition("#")
        comment = hash_ + comment
        if not code.strip():
            out.append(raw.rstrip())
            continue

        m = CONSTANT.match(code)
        if m:  # numeric constant: copied as is, '=' spacing normalised
            line = f"{m.group(1)} {m.group(2)}={m.group(3)}"
            out.append(f"{line}  {comment}".rstrip())
            continue

        m = FIELD.match(code)
        if not m:
            sys.exit(f"{path}: cannot parse line: {raw!r}")
        type_, name = m.groups()
        base, array = split_array(type_)
        if base in TYPE_MAP:
            notes.append(f"type {base} -> {TYPE_MAP[base]} ({name})")
            base = TYPE_MAP[base]
        new_name = snake_case(name)
        if new_name != name:
            notes.append(f"renamed field {name} -> {new_name}")
        if not ROS2_FIELD_NAME.match(new_name):
            sys.exit(f"{path}: field name {new_name!r} is not valid in ROS 2")
        sections[-1].append((name, new_name))
        out.append(f"{base}{array} {new_name}  {comment}".rstrip())

    if notes:
        header = ["# Ported from ROS 1 by scripts/port_msgs_to_ros2.py:"]
        header += [f"#   {n}" for n in notes]
        out = header + out
    return "\n".join(out) + "\n", sections, notes


def used_packages(text, own_package):
    deps = set()
    for line in text.splitlines():
        code = line.split("#", 1)[0].strip()
        if code and code != "---" and "=" not in code:
            base = split_array(code.split()[0])[0]
            if "/" in base:
                deps.add(base.split("/")[0])
    deps.discard(own_package)
    return deps


def yaml_rule(package, kind, name, sections):
    lines = [
        "-",
        f"  ros1_package_name: '{package}'",
        f"  ros1_{kind}_name: '{name}'",
        f"  ros2_package_name: '{package}'",
        f"  ros2_{kind}_name: '{name}'",
    ]
    if kind == "message":
        parts = [("fields_1_to_2", sections[0])]
    else:
        parts = [("request_fields_1_to_2", sections[0]),
                 ("response_fields_1_to_2", sections[1])]
    for key, fields in parts:
        if any(a != b for a, b in fields):  # a part without renames needs no list
            lines.append(f"  {key}:")
            lines += [f"    {a}: '{b}'" for a, b in fields]
    return "\n".join(lines)


def package_xml(package, deps, has_rules):
    version, origin, author = PACKAGES[package]
    depends = "\n".join(f"  <depend>{d}</depend>" for d in sorted(deps))
    # Without renamed fields there are no rules, and ros1_bridge pairs the messages by
    # their names alone.
    rules = '\n    <ros1_bridge mapping_rules="mapping_rules.yaml"/>' if has_rules else ""
    return f"""<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>{package}</name>
  <version>{version}</version>
  <description>
    ROS 2 port of {package} from {origin}, for use
    with ros1_bridge. Generated by scripts/port_msgs_to_ros2.py; do not edit by hand.
  </description>

  <maintainer email="Techniksam@users.noreply.github.com">Samuel Galla</maintainer>
  <license>BSD-3-Clause</license>
  <author>{author}</author>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>

{depends}

  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>{rules}
  </export>
</package>
"""


def cmakelists(package, deps, files, has_rules):
    install = ("""
# ros1_bridge finds this file through the <export> tag in package.xml.
install(FILES mapping_rules.yaml DESTINATION share/${PROJECT_NAME})
""" if has_rules else "")
    finds = "\n".join(f"find_package({d} REQUIRED)" for d in sorted(deps))
    listed = "\n".join(f'  "{f}"' for f in files)
    if deps:
        listed += f'\n  DEPENDENCIES {" ".join(sorted(deps))}'
    return f"""# Generated by scripts/port_msgs_to_ros2.py; do not edit by hand.
cmake_minimum_required(VERSION 3.10)
project({package})

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
{finds}

rosidl_generate_interfaces(${{PROJECT_NAME}}
{listed}
)
{install}
ament_export_dependencies(rosidl_default_runtime)
ament_package()
"""


def port(package):
    src, dst = SRC / package, DST / package
    if dst.exists():
        shutil.rmtree(dst)
    files, rules, deps, summary = [], [], set(), []
    for kind, folder, ext in (("message", "msg", ".msg"), ("service", "srv", ".srv")):
        for path in sorted((src / folder).glob(f"*{ext}")):
            text, sections, notes = convert_file(path)
            (dst / folder).mkdir(parents=True, exist_ok=True)
            (dst / folder / path.name).write_text(text, encoding="utf-8", newline="\n")
            files.append(f"{folder}/{path.name}")
            deps |= used_packages(text, package)
            if any(a != b for section in sections for a, b in section):
                rules.append(yaml_rule(package, kind, path.stem, sections))
            summary += [f"  {folder}/{path.name}: {n}" for n in notes]

    header = ("# ros1_bridge mapping rules, generated by scripts/port_msgs_to_ros2.py.\n"
              "# One rule per message/service whose field names differ between ROS 1 and 2.\n"
              "# A rule lists ALL fields: fields not listed are not copied by the bridge.\n")
    if rules:
        (dst / "mapping_rules.yaml").write_text(header + "\n".join(rules) + "\n", encoding="utf-8", newline="\n")
    (dst / "package.xml").write_text(package_xml(package, deps, bool(rules)), encoding="utf-8", newline="\n")
    (dst / "CMakeLists.txt").write_text(cmakelists(package, deps, files, bool(rules)), encoding="utf-8", newline="\n")
    print(f"{package}: {len(files)} files, {len(rules)} mapping rules, depends on {sorted(deps)}")
    print("\n".join(summary))


if __name__ == "__main__":
    for p in PACKAGES:
        port(p)
