# A project of your own, with VS Code

For anything beyond a single script: a folder of its own, under version control, edited
in VS Code with completion, type checking and a debugger, and run with one click.

The idea: your project is a folder on the Linux system the bridge runs on (WSL2 or a
Linux machine). VS Code opens that folder **inside a container** made from the `client`
image. The container has ROS 2, Baxter's messages and `baxter_interface`, so the editor
knows them and your programs run there directly.

A ready-made starting point is in
[examples/project-template/](../examples/project-template/).

## The layout

```
my-project/
├── .devcontainer/
│   ├── devcontainer.json     how VS Code starts the container
│   └── Dockerfile            the container: the client image plus your extra packages
├── src/
│   └── my_project/           your own code, as a Python package
│       ├── __init__.py
│       └── motions.py
├── scripts/                  programs you run; short, they import from the package
│   ├── read_robot.py
│   └── draw_square.py
├── tests/                    tests for everything that needs no robot
│   └── test_motions.py
├── pyproject.toml            the project's name and the settings of the tools
├── requirements.txt          extra Python packages
├── README.md
└── .gitignore
```

This is Python's usual "src layout". The split that matters is between `src/` and
`scripts/`: functions and classes go into the package, where they can be tested and used
from several programs; a script is the few lines that create the robot's parts and call
them. Working out *where* to move is arithmetic and can be tested without a robot; the
template's `motions.py` and its test show the pattern.

## Setting it up

Once, in VS Code on Windows: install the extensions **WSL** and **Dev Containers** (both
by Microsoft). Docker Desktop stays closed; VS Code then uses the Docker inside WSL.

Then, in the Ubuntu terminal (the Linux home folder, not under `/mnt/c`):

```bash
mkdir -p ~/projects
cp -r ~/baxter-ros2-bridge/examples/project-template ~/projects/my-project
cd ~/projects/my-project
git init
code .
```

VS Code opens, connected to WSL (the bottom-left corner says "WSL: Ubuntu"), and offers
**"Reopen in Container"**. Accept, or press F1 and choose "Dev Containers: Reopen in
Container". The first time takes a few minutes, because the container is built and the
extensions are installed in it. The corner then says "Dev Container: Baxter project".

To give the project its own name: rename the folder `src/my_project`, change the two
`from my_project...` imports in `scripts/` and `tests/`, and the `name` in
`pyproject.toml`.

Three settings in `.devcontainer/devcontainer.json` depend on your setup, and the file
explains each: the ROS 2 domain number, the network settings file (`wsl.xml` in WSL2,
`default.xml` on a normal Linux machine), and the path of the bridge repository.

## Working in it

Start the bridge first, in a normal Ubuntu terminal (not in VS Code):

```bash
cd ~/baxter-ros2-bridge && docker compose up -d bridge
```

Then, in VS Code:

| To | Do |
| --- | --- |
| run a program | open it and press the Run button (top right); or in the terminal: `python3 scripts/read_robot.py` |
| open a terminal in the container | menu Terminal, New Terminal. The library's tools work there too: `ros2 run baxter_interface enable_robot -s` |
| debug | set a breakpoint (click left of a line number), press F5, choose "Python File" |
| run the tests | the Testing panel (flask icon); or `python3 -m pytest` |
| commit | the Source Control panel, or `git` in VS Code's terminal. VS Code passes your git name and login into the container |

## What the editor checks

- **Completion and documentation** work for your own code, for `baxter_interface` (hover
  over a method to see its description) and for the ROS messages.
- **Type checking** runs in "basic" mode. `baxter_interface` says for every method what
  it takes and what it returns, so the editor marks mistakes in calls to the robot
  before the program runs: a misspelled method or argument, text where a number
  belongs, a missing argument, a function connected to an event that takes the wrong
  arguments. It also knows what can be `None`: `RangeSensor.distance()` is a number *or*
  `None` (nothing in range), and adding 1 to it without a check is marked.
- Give your own functions annotations as well
  (`def lift(limb: Limb, height: float) -> None:`); the checker is only as good as what
  is written down.
- **Style** is checked by Ruff as you type. It does not change your files by itself; to
  let it tidy a file, right-click in the editor and choose "Format Document".

Two things that may surprise:

- Commands that an `at_once()` block can take over (`move_...`, `set_pan`, the gripper's
  `open` and `close`) are shown as returning "... or `None`": inside such a block they
  return nothing, because they have not happened yet.
- ROS's own packages (`rclpy`, the message classes) describe themselves less completely.
  A message's list field, for example, has to be given a whole list
  (`goal.trajectory.points = points`); the checker rejects `.append()` on it.

## Installing more Python packages

`pip install` in VS Code's terminal works, but what it installs is gone the next time
the container is rebuilt. Packages therefore go into two files of the project, so that
the container always has them, on every machine:

**Packages from PyPI (`pip`):** add a line to `requirements.txt`, best with a version:

```
tabulate==0.9.0
```

**Packages from Ubuntu (`apt`):** many Python packages exist as `python3-<name>`
(`apt-cache search python3-scipy` in the terminal tells). They fit the ROS 2
installation, so prefer them where they exist. Add the name to the list in
`.devcontainer/Dockerfile`:

```dockerfile
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      python3-pip \
      python3-scipy \
 && rm -rf /var/lib/apt/lists/*
```

After either change: F1, **"Dev Containers: Rebuild Container"**.

Two cautions:

- **Not `numpy` and not `opencv-python` through pip.** The container has the versions
  that ROS 2 and `baxter_interface` were built with (NumPy 1.26, OpenCV 4.6). pip would
  replace them with newer ones that do not fit, and pictures from the cameras would
  stop working. If a package you install pulls in a newer NumPy, the rebuild shows it in
  its output; pin that package to an older version, or use the Ubuntu package.
- To try something quickly, `pip install --break-system-packages <name>` in the terminal
  is fine. Ubuntu asks for that flag before pip may touch the system's Python; in this
  container nothing else lives there. Put the package into `requirements.txt` once you
  keep it.

## Good to know

- **Files belong to root.** The container runs as root, so files created in it belong to
  root when seen from the plain Ubuntu terminal. Do everything, git included, from
  inside VS Code. If you need the files outside: `sudo chown -R $USER ~/projects/my-project`.
- **A new version of the library** comes with a new client image. Download it with
  `docker pull ghcr.io/hull-robotics-society/baxter-client-jazzy:latest`, then F1, "Dev
  Containers: Rebuild Container", so that the project's container is built on it.
- **The robot stays enabled** when a program ends or crashes. `ros2 run baxter_interface enable_robot -d`
  in the terminal disables it.
- **Without VS Code** the same container works from a terminal:

  ```bash
  cd ~/projects/my-project
  docker build -t my-project -f .devcontainer/Dockerfile .
  docker run --rm -it --network host --ipc host -e ROS_DOMAIN_ID=42 \
    -e FASTRTPS_DEFAULT_PROFILES_FILE=/config/dds/wsl.xml \
    -v ~/baxter-ros2-bridge/bridge_config/dds:/config/dds:ro \
    -v "$PWD":/project -w /project \
    my-project bash -c 'export PYTHONPATH=$PYTHONPATH:/project/src; python3 scripts/read_robot.py'
  ```

## Limitations

- Not tested yet: VS Code's debugger and its Testing panel in the container.
