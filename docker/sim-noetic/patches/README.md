# Changes to Rethink's sources for the simulator image

Each `*.patch` file here is applied to one of the cloned repositories when the image is
built (see `../Dockerfile`). The part of the file name before the first `-` is the
repository's folder: `baxter_simulator-...patch` is applied inside `baxter_simulator/`.
Each file says at its top why it is needed.
