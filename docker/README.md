# Running in a container

ROS 2 Jazzy ships binaries for Ubuntu 24.04 only. On any other system (another Linux distribution, macOS,
Windows with WSL 2) run the simulation in the container defined here: Ubuntu 24.04 with ROS 2 Jazzy, Gazebo
Harmonic and every dependency of the workspace. On Ubuntu 24.04 itself, `./setup.sh` installs everything natively
and you do not need this.

The container path is newer than the native install and less tested; if the image does not build or a launch
misbehaves in it, please open an issue with the output.

The image holds only the dependencies. Your clone of the repository is mounted into the container at the same
path as on the host, so you edit files with your usual tools and build and run inside.

## Requirements

- Docker Engine 23 or newer (Linux), or Docker Desktop (macOS, Windows). https://docs.docker.com/get-docker/
- About 8 GB of disk for the image and a build, 16 GB with PX4.
- Tested on an x86-64 Linux host. arm64 hosts (Apple silicon, Raspberry Pi 5) should work, since ROS 2 and
  Gazebo publish arm64 packages, but this has not been tried.

## First run

```bash
git clone https://github.com/AJC-006/robot-fleet-sim.git
cd robot-fleet-sim
docker/run.sh ./setup.sh        # builds the image on first use (10-20 min), then builds the workspace inside it
```

`./setup.sh` in the container finds every apt package already installed, so it only checks rosdep, builds the
Micro-XRCE-DDS agent for the drone (into `.deps/`) and runs `colcon build`.

## Everyday use

```bash
docker/run.sh                    # a shell with ROS and the workspace sourced
docker/run.sh ros2 launch fleet_bringup sim.launch.py robot:=round world:=apartment gui:=false rviz:=false
docker/run.sh scripts/run_all_tests.sh --quick
```

Each `docker/run.sh` starts a fresh container and removes it on exit; everything you build or record lands in
the workspace on the host. To work in several terminals at once, open a shell in one and use
`docker exec -it <container> bash` in the others (`docker ps` shows the name), or start each command with
`docker/run.sh` (the containers share the host network, so ROS 2 topics flow between them).

| what | how |
|---|---|
| rebuild the image (after `scripts/install_deps.sh` or a `package.xml` changes) | `REBUILD=1 docker/run.sh true` |
| include PX4 SITL for the wyvern drone | `REBUILD=1 WITH_PX4=1 docker/run.sh true` (sets `PX4_DIR=/opt/PX4-Autopilot` in the image) |
| use another image name | `IMAGE=my-name docker/run.sh ...` |
| separate ROS/Gazebo traffic from other users | `ROS_DOMAIN_ID=7 GZ_PARTITION=me docker/run.sh ...` (passed through) |

## Graphics

The launches run headless by default in the container (`gui:=false rviz:=false`). Camera and lidar sensors need
OpenGL rendering: `docker/run.sh` passes the host's `/dev/dri` (Intel and AMD GPUs on Linux) into the container,
and without it Gazebo falls back to software rendering, which works but is slower.

To see the Gazebo GUI or RViz on a Linux host with X11 or XWayland:

```bash
xhost +local:                    # allow local containers to use your display
docker/run.sh --gui ros2 launch fleet_bringup sim.launch.py robot:=round world:=apartment
```

On macOS and Windows, run headless, record a bag (`ros2 bag record`) and open it on the host with PlotJuggler or
Lichtblick.

## Notes

- **Host network.** The container uses `--net=host` so ROS 2 discovery works between containers and with ROS 2 on
  the host. On Docker Desktop (4.34 or newer), turn on host networking in Settings, Resources, Network for that;
  inside a single container it is not needed.
- **One build at a time.** The container and a native install share `build/` and `install/`. When you switch
  between them, delete those two folders first (`rm -rf build install`), or CMake reports a cache mismatch.
- **Files you create** are owned by your user on Linux (the container runs with your uid). The shell prompt shows
  `I have no name!` because that uid has no account inside the image; this is harmless.
- **Home directory.** `HOME` inside the container is `.docker-home/` in the workspace (git-ignored), so the
  rosdep cache and shell history survive between runs.
