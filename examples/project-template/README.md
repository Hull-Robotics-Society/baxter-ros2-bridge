# My Baxter project

A starting point for a project of your own. Copy this folder, rename it, and open it in
VS Code; `docs/own-project.md` in the bridge repository explains every step.

```
.devcontainer/     how VS Code starts the container (image, network, extra packages)
src/my_project/    the project's own code, as a Python package
scripts/           programs to run; short, they import from the package
tests/             tests for everything that needs no robot
pyproject.toml     the project's name and the settings of the tools
requirements.txt   extra Python packages (pip)
```

## Run

With the bridge running, in VS Code's terminal (inside the container):

```bash
python3 scripts/read_robot.py      # reads only
python3 scripts/draw_square.py     # THE ARM MOVES
python3 -m pytest                  # the tests; no robot needed
```
