# Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic + every apt and rosdep dependency of robot-fleet-sim, for systems
# that cannot install ROS 2 Jazzy natively. The workspace is not copied in: docker/run.sh mounts it and you build it
# inside with ./setup.sh. See docker/README.md.
#
#   docker build -f docker/Dockerfile -t robot-fleet-sim .                          # from the repository root
#   docker build -f docker/Dockerfile --build-arg WITH_PX4=1 -t robot-fleet-sim .   # + PX4 SITL in /opt/PX4-Autopilot

FROM osrf/ros:jazzy-desktop

ARG WITH_PX4=
# PX4 commit that src/px4_msgs was generated from (same pin as setup.sh)
ARG PX4_COMMIT=e146fe4cc69459493196b172d9b4576dc4bce127
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# build tools for the agent and PX4, and the Gazebo (OSRF) apt source for the python3-gz-* bindings
RUN apt-get update \
 && apt-get install -y --no-install-recommends curl ca-certificates sudo git cmake build-essential \
 && curl -fsSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg]" \
         "http://packages.osrfoundation.org/gazebo/ubuntu-stable noble main" > /etc/apt/sources.list.d/gazebo-stable.list \
 && rm -rf /var/lib/apt/lists/*

# the project's apt list, then rosdep over the package.xml files (.dockerignore lets only those into the context)
COPY scripts/install_deps.sh /tmp/deps/scripts/install_deps.sh
COPY src /tmp/deps/src
RUN rosdep update --rosdistro jazzy \
 && LOG=/tmp/deps/install_deps.log bash /tmp/deps/scripts/install_deps.sh \
 && ! grep -q 'step(s) failed' /tmp/deps/install_deps.log \
 && rm -rf /var/lib/apt/lists/* /tmp/deps

# optional: PX4 SITL for the wyvern drone (about 20 min and 10 GB more)
RUN if [ -n "$WITH_PX4" ]; then \
      git clone https://github.com/PX4/PX4-Autopilot.git /opt/PX4-Autopilot \
   && cd /opt/PX4-Autopilot && git checkout "$PX4_COMMIT" && git submodule update --init --recursive \
   && bash Tools/setup/ubuntu.sh --no-nuttx \
   && make px4_sitl_default \
   && chmod -R a+rwX /opt/PX4-Autopilot && rm -rf /var/lib/apt/lists/*; \
    fi
ENV PX4_DIR=${WITH_PX4:+/opt/PX4-Autopilot}
