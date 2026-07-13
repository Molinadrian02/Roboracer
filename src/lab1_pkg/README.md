# F1TENTH Lab 1: Intro to ROS 2

Containerized ROS 2 Humble workspace implementing publisher/subscriber nodes in both Python and C++, as part of the F1TENTH course.

## Overview

This lab implements two ROS 2 nodes:

- **`talker`**: publishes an `AckermannDriveStamped` message to the `drive` topic at maximum rate. The `speed` and `steering_angle` fields are set from the ROS 2 parameters `v` and `d`, respectively.
- **`relay`**: subscribes to the `drive` topic, multiplies the received `speed` and `steering_angle` values by 3, and republishes the result as a new `AckermannDriveStamped` message on the `drive_relay` topic.

Both nodes are implemented twice — once in Python and once in C++ — to compare the ROS 2 workflow across languages.

## Repository Structure

```
Roboracer/
├── Dockerfile              # ROS 2 Humble image with a non-root user matching the host UID/GID
├── docker-compose.yml      # Build & run configuration for the dev container
├── README.md
├── SUBMISSION.md           # Written answers to the lab's conceptual questions
└── src/
    └── lab1_pkg/
        ├── CMakeLists.txt
        ├── package.xml
        ├── scripts/         # Python nodes
        │   ├── talker.py
        │   └── relay.py
        ├── src/             # C++ nodes
        │   ├── talker_cpp.cpp
        │   └── relay_cpp.cpp
        └── launch/
            ├── lab1_launch.py       # Launches the Python nodes
            └── lab1_launch_cpp.py   # Launches the C++ nodes
```

## Environment Setup

The entire environment runs inside Docker to avoid host system dependency issues.

### Build the image

```bash
cd ~/Desktop/Roboracer
export HOST_UID=$(id -u)
export HOST_GID=$(id -g)
docker compose build
```

### Run the container

```bash
docker compose run --rm f1tenth_dev
```

This mounts the repository at `/f1tenth_ws` inside the container and drops you into a shell running as a non-root user (`devuser`) with the same UID/GID as your host user, so files created inside the container remain editable from the host.

## Building the ROS 2 package

Inside the container:

```bash
cd /f1tenth_ws
colcon build --packages-select lab1_pkg
source install/setup.bash
```

## Running the nodes

### Python

```bash
# Terminal 1
ros2 run lab1_pkg talker.py --ros-args -p v:=2.0 -p d:=0.5

# Terminal 2
ros2 run lab1_pkg relay.py
```

### C++

```bash
# Terminal 1
ros2 run lab1_pkg talker_cpp --ros-args -p v:=2.0 -p d:=0.5

# Terminal 2
ros2 run lab1_pkg relay_cpp
```

### Using the launch files

Instead of running each node manually, launch both at once:

```bash
# Python nodes, with v/d preset in the launch file
ros2 launch lab1_pkg lab1_launch.py

# C++ nodes
ros2 launch lab1_pkg lab1_launch_cpp.py
```

## Verifying the setup

With both nodes running, these commands should confirm everything is wired correctly:

```bash
ros2 topic list          # should show /drive and /drive_relay
ros2 topic info /drive
ros2 topic echo /drive
ros2 node list            # should show /talker and /relay
ros2 node info /talker
ros2 node info /relay
```

`/drive_relay` should show `speed` and `steering_angle` values equal to 3× whatever `talker` is publishing on `/drive`.

## Notes

- The package uses `ament_cmake` as its build type (rather than `ament_python`) so that it can contain both Python scripts and C++ executables in the same package.
- Python scripts are installed via `install(PROGRAMS ...)` in `CMakeLists.txt`; C++ nodes are compiled and installed via `add_executable(...)` + `ament_target_dependencies(...)` + `install(TARGETS ...)`.
- Node names (`talker`, `relay`) are fixed inside the source code regardless of language; only the executable/target names differ between the Python and C++ versions (`talker.py`/`talker_cpp`, `relay.py`/`relay_cpp`) to avoid naming collisions when running `ros2 run`.
