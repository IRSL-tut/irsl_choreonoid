# create /choreonoid_ws
# choreonoid_ws: choreonoid
ARG BASE_IMAGE=ros:melodic-ros-base
FROM ${BASE_IMAGE}

LABEL maintainer="IRSL-tut (https://github.com/IRSL-tut) <faculty@irsl.eiiris.tut.ac.jp>"

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

###
RUN apt update -q -qq && \
    apt install -q -qq -y curl wget git python3-vcstool && \
    apt clean && \
    rm -rf /var/lib/apt/lists/

### create workspace
WORKDIR /choreonoid_ws
##
COPY config .irsl_choreonoid_config
RUN <<EOF
if [ "${ROS_DISTRO}" == "noetic" -o "${ROS_DISTRO}" == "one" ]; then
    INSTALL_FILE=$(pwd)/.irsl_choreonoid_config/dot.rosinstall
else
    INSTALL_FILE=$(pwd)/.irsl_choreonoid_config/dot.ros2install
fi
#
source /opt/ros/${ROS_DISTRO}/setup.bash
(mkdir src; cd src; vcs import --recursive < ${INSTALL_FILE})
rm -rf src/irsl_choreonoid
sed -i -e "s@osqp-cpp src/osqp++.cc@osqp-cpp SHARED src/osqp++.cc@g" src/qp_solvers/osqp-cpp/osqp-cpp/CMakeLists.txt
find src/prioritized_qp src/ik_solvers src/qp_solvers -name CMakeLists.txt -exec sed -i -e s@-std=c++[0-9][0-9]@-std=c++17@g {} \;
#
EOF

##
COPY . src/irsl_choreonoid

### install packages
RUN apt update -q -qq && \
    if $(echo -e "23.10\n$(lsb_release -s -r)" | sort -C -V); then \
        UBUNTU_VER=24.04; \
    else \
        UBUNTU_VER=$(lsb_release -s -r); \
    fi && \
    src/choreonoid/misc/script/install-requisites-ubuntu-${UBUNTU_VER}.sh && \
    if [ "$ROS_DISTRO" = "noetic" -o "$ROS_DISTRO" = "one" ]; then \
        apt install -q -qq -y python3-catkin-tools libreadline-dev ; \
    elif [ "$ROS_DISTRO" = "lyrical" -o  "$ROS_DISTRO" = "jazzy" -o "$ROS_DISTRO" = "humble" -o "$ROS_DISTRO" = "rolling" ]; then \
        apt install -q -qq -y libreadline-dev ; \
    else \
        apt install -q -qq -y python-catkin-tools libreadline-dev ; \
    fi && \
    apt install -q -qq -y libpulse-dev libsndfile-dev gstreamer1.0-libav libgstreamer-plugins-base1.0-dev libgstreamer1.0-dev \
    libbullet-dev libbullet-extras-dev libbulletml-dev && \
    rosdep update -y -q -r && \
    rosdep install -y -q -r --ignore-src --from-path src/irsl_choreonoid && \
    apt clean && \
    rm -rf /var/lib/apt/lists/

### build all
RUN <<EOF
_CMAKE_ARGS_='-DBUILD_TEST=ON -DBUILD_POSE_SEQ_PLUGIN=ON -DBUILD_BULLET_PLUGIN=ON -DBUILD_BALANCER_PLUGIN=ON -DBUILD_MOCAP_PLUGIN=ON -DBUILD_MEDIA_PLUGIN=ON'
_BUILD_TARGET_='irsl_choreonoid'
if [ "${ROS_DISTRO}" == "noetic" -o "${ROS_DISTRO}" == "one" ]; then
    source /opt/ros/${ROS_DISTRO}/setup.bash
    catkin config --cmake-args ${_CMAKE_ARGS_}
    catkin config --install
    catkin build ${_BUILD_TARGET_} --no-status --no-notify -p 1
else
    source /opt/ros/${ROS_DISTRO}/setup.bash
    colcon build --parallel-workers 1 --merge-install --event-handlers console_direct+ desktop_notification- log_command+ status- \
    --cmake-args ${_CMAKE_ARGS_} -DCMAKE_POLICY_VERSION_MINIMUM=3.5 --ament-cmake-args -DCMAKE_POLICY_VERSION_MINIMUM=3.5 --packages-up-to ${_BUILD_TARGET_}
fi
EOF

### TODO TEST
