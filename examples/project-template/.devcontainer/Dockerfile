# The container this project runs in: the bridge repository's client image (ROS 2 Jazzy,
# Baxter's messages, the Python library baxter_interface) plus the extra packages this
# project needs. VS Code builds it when the project is opened in the container, and
# again after "Dev Containers: Rebuild Container".
FROM ghcr.io/hull-robotics-society/baxter-client-jazzy:latest

# Extra packages from Ubuntu. Prefer this for anything that exists as a python3-... package
# (search: apt-cache search python3-<name>); such packages fit the ROS 2 installation.
# Add names to the list, one per line, each ending in a backslash.
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      python3-pip \
 && rm -rf /var/lib/apt/lists/*

# Extra packages from PyPI (pip), listed in requirements.txt.
# --break-system-packages: Ubuntu refuses pip installs into the system's Python unless told
# so. In this container that is fine: it holds nothing but this project, and it is built
# anew from this file whenever something goes wrong.
COPY requirements.txt /tmp/requirements.txt
RUN pip install --no-cache-dir --break-system-packages -r /tmp/requirements.txt
