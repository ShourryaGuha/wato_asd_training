ARG BASE_IMAGE=ghcr.io/watonomous/robot_base/base:humble-ubuntu22.04

################################ Source ################################
FROM ${BASE_IMAGE} AS source

WORKDIR ${AMENT_WS}/src

# Robust APT in this stage (before any apt-get)
RUN set -eux; \
  printf 'Acquire::Retries "6";\nAcquire::http::Timeout "20";\nAcquire::https::Timeout "20";\nAcquire::ForceIPv4 "true";\n' \
    >/etc/apt/apt.conf.d/99-robust-apt; \
  sed -i 's|http://archive.ubuntu.com/ubuntu|mirror://mirrors.ubuntu.com/mirrors.txt|g' /etc/apt/sources.list; \
  sed -i 's|http://security.ubuntu.com/ubuntu|https://security.ubuntu.com/ubuntu|g' /etc/apt/sources.list; \
  apt-get update

# Copy in source code 
COPY src/wato_msgs wato_msgs

# Scan for rosdeps
RUN apt-get -qq update && rosdep update && \
    rosdep install --from-paths . --ignore-src -r -s \
        | (grep 'apt-get install' || true) \
        | awk '{print $3}' \
        | sort  > /tmp/colcon_install_list

################################# Dependencies ################################
FROM ${BASE_IMAGE} AS dependencies

# Robust APT in this stage (before any apt-get)
RUN set -eux; \
  printf 'Acquire::Retries "6";\nAcquire::http::Timeout "20";\nAcquire::https::Timeout "20";\nAcquire::ForceIPv4 "true";\n' \
    >/etc/apt/apt.conf.d/99-robust-apt; \
  sed -i 's|http://archive.ubuntu.com/ubuntu|mirror://mirrors.ubuntu.com/mirrors.txt|g' /etc/apt/sources.list; \
  sed -i 's|http://security.ubuntu.com/ubuntu|https://security.ubuntu.com/ubuntu|g' /etc/apt/sources.list; \
  apt-get update

# Install Foxglove deps
RUN DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
      curl \
      ros-humble-ros2bag \
      "ros-humble-rosbag2*" \
      ros-humble-foxglove-msgs \
  && rm -rf /var/lib/apt/lists/*

# Set up apt repo (universe et al.)
RUN apt-get update \
  && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
       lsb-release software-properties-common apt-transport-https \
  && apt-add-repository universe \
  && rm -rf /var/lib/apt/lists/*

# Install additional ROS dependencies
RUN apt-get update \
  && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
       ros-$ROS_DISTRO-foxglove-bridge \
       ros-$ROS_DISTRO-rosbridge-server \
       ros-$ROS_DISTRO-topic-tools \
       ros-$ROS_DISTRO-vision-msgs \
  && rm -rf /var/lib/apt/lists/*

# Install Rosdep requirements
COPY --from=source /tmp/colcon_install_list /tmp/colcon_install_list
RUN apt-fast install -qq -y --no-install-recommends $(cat /tmp/colcon_install_list)

# Copy in source code from source stage
WORKDIR ${AMENT_WS}
COPY --from=source ${AMENT_WS}/src src

# Dependency Cleanup
WORKDIR /
RUN apt-get -qq autoremove -y && apt-get -qq autoclean && apt-get -qq clean && \
    rm -rf /root/* /root/.ros /tmp/* /var/lib/apt/lists/* /usr/share/doc/*

################################ Build ################################
FROM dependencies AS build

# Build ROS2 packages
WORKDIR ${AMENT_WS}
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build \
        --cmake-args -DCMAKE_BUILD_TYPE=Release --install-base ${WATONOMOUS_INSTALL}

# Source and Build Artifact Cleanup 
RUN rm -rf src/* build/* devel/* install/* log/*

# Entrypoint will run before any CMD on launch. Sources ~/opt/<ROS_DISTRO>/setup.bash and ~/ament_ws/install/setup.bash
COPY docker/wato_ros_entrypoint.sh ${AMENT_WS}/wato_ros_entrypoint.sh
ENTRYPOINT ["./wato_ros_entrypoint.sh"]
