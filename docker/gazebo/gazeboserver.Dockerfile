ARG BASE_IMAGE=ghcr.io/watonomous/robot_base/base:humble-ubuntu22.04

################################ Source ################################
FROM ${BASE_IMAGE} AS source

WORKDIR ${AMENT_WS}/src

# Copy in source code 
COPY src/gazebo gazebo

# Scan for rosdeps
RUN apt-get -qq update && rosdep update && \
    rosdep install --from-paths . --ignore-src -r -s \
        | (grep 'apt-get install' || true) \
        | awk '{print $3}' \
        | sort  > /tmp/colcon_install_list

################################# Dependencies ################################
FROM ${BASE_IMAGE} AS dependencies

# # RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-keys A4B469963BF863CC
# RUN apt-get update && apt-get install ffmpeg libsm6 libxext6 -y

# RUN apt install -y lsb-release wget gnupg
# RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
# RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
# RUN apt-get -y update
# RUN apt-get -y install ros-${ROS_DISTRO}-ros-gz ignition-fortress
# RUN echo $GAZEBO_PLUGIN_PATH=/opt/ros/humble/lib
################################# Dependencies ################################

# Fix APT mirrors + retries before any installs
RUN set -eux; \
    printf 'Acquire::Retries "6";\nAcquire::http::Timeout "20";\nAcquire::https::Timeout "20";\nAcquire::ForceIPv4 "true";\n' \
      >/etc/apt/apt.conf.d/99-robust-apt; \
    sed -i 's|http://archive.ubuntu.com/ubuntu|mirror://mirrors.ubuntu.com/mirrors.txt|g' /etc/apt/sources.list; \
    sed -i 's|http://security.ubuntu.com/ubuntu|https://security.ubuntu.com/ubuntu|g' /etc/apt/sources.list; \
    apt-get update

# Install basics
RUN DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    ffmpeg libsm6 libxext6 lsb-release wget gnupg && \
    rm -rf /var/lib/apt/lists/*

# Add OSRF Gazebo repo and install ros-gz + ignition
RUN set -eux; \
    wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg; \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
      > /etc/apt/sources.list.d/gazebo-stable.list; \
    apt-get update; \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-ros-gz ignition-fortress; \
    rm -rf /var/lib/apt/lists/*

# Make the env var persistent (the old RUN echo didn't persist)
ENV GAZEBO_PLUGIN_PATH=/opt/ros/humble/lib


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
