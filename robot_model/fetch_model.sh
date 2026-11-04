#!/usr/bin/env bash
# Downloads Baxter's robot model (the description of its links and joints, and the 3D
# shapes, about 35 MB) from Rethink Robotics' baxter_common into this folder, together
# with the model of Rethink's electric gripper (rethink_ee_description, into
# urdf/rethink_ee and meshes/rethink_ee):
#
#   bash robot_model/fetch_model.sh
#
# The model is not kept in this repository; it is fetched at the exact commit of
# baxter_common v1.2.0, the version both lab robots run. The Docker image "rviz" runs
# this script when it is built; on a machine with ROS 2 installed, run it once before
# colcon build (see docs/rviz.md).
set -euo pipefail

COMMIT=6c4b0f375fe4e356a3b12df26ef7c0d5e58df86e   # baxter_common v1.2.0
URL="https://github.com/RethinkRobotics/baxter_common/archive/${COMMIT}.tar.gz"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ -f "$HERE/urdf/baxter.urdf" ] && [ -d "$HERE/meshes" ] && [ -d "$HERE/urdf/rethink_ee" ]; then
  echo "The robot model is already here ($HERE/urdf, $HERE/meshes). Delete both folders to download again."
  exit 0
fi

TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

echo "Downloading baxter_common v1.2.0 (${COMMIT:0:7}) from GitHub ..."
if command -v curl > /dev/null; then
  curl --fail --location --silent --show-error "$URL" --output "$TMP/baxter_common.tar.gz"
else
  wget --quiet "$URL" --output-document "$TMP/baxter_common.tar.gz"
fi

tar -xzf "$TMP/baxter_common.tar.gz" -C "$TMP"
SOURCE="$TMP/baxter_common-$COMMIT/baxter_description"
rm -rf "$HERE/urdf" "$HERE/meshes"
cp -r "$SOURCE/urdf" "$SOURCE/meshes" "$HERE/"
# The gripper's model is a package of its own in ROS 1 (rethink_ee_description). Here it
# becomes part of this one, so that one package holds the whole robot: its files refer to
# themselves by the new place.
GRIPPER="$TMP/baxter_common-$COMMIT/rethink_ee_description"
cp -r "$GRIPPER/urdf" "$HERE/urdf/rethink_ee"
cp -r "$GRIPPER/meshes" "$HERE/meshes/rethink_ee"
find "$HERE/urdf" -name '*.xacro' -exec sed -i \
  -e 's|package://rethink_ee_description/meshes/|package://baxter_description/meshes/rethink_ee/|g' \
  -e 's|$(find rethink_ee_description)/urdf/|$(find baxter_description)/urdf/rethink_ee/|g' {} +
# Rethink's licence (BSD) travels with the files.
cp "$TMP/baxter_common-$COMMIT/LICENSE" "$HERE/meshes/LICENSE.rethink"
cp "$TMP/baxter_common-$COMMIT/LICENSE" "$HERE/urdf/LICENSE.rethink"

echo "Done: $(find "$HERE/meshes" -type f | wc -l) files in meshes/, model in urdf/baxter.urdf"
