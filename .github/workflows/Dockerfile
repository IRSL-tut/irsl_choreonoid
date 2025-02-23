# create /choreonoid_ws
# choreonoid_ws: choreonoid
ARG BASE_IMAGE=ros:melodic-ros-base
FROM ${BASE_IMAGE}

ARG UBUNTU_VER=18.04

LABEL maintainer "IRSL-tut (https://github.com/IRSL-tut) <faculty@irsl.eiiris.tut.ac.jp>"

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

###
RUN apt update -q -qq && \
    apt install -q -qq -y curl wget git && \
    apt clean && \
    rm -rf /var/lib/apt/lists/

### create workspace
WORKDIR /choreonoid_ws
##
COPY config .irsl_choreonoid_config
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    wstool init src .irsl_choreonoid_config/dot.rosinstall && \
    rm -rf src/irsl_choreonoid && \
    wstool update -t src && \
    patch -d src -p1 < .irsl_choreonoid_config/choreonoid_closed_ik.patch

##
COPY . src/irsl_choreonoid

### install packages
RUN apt update -q -qq && \
    apt install -q -qq -y sudo && \
    src/choreonoid/misc/script/install-requisites-ubuntu-${UBUNTU_VER}.sh && \
    if [ "$ROS_DISTRO" = "noetic" ]; then \
    apt install -q -qq -y python3-catkin-tools libreadline-dev ipython3; \
    else \
    apt install -q -qq -y python-catkin-tools libreadline-dev ipython3; fi && \
    apt clean && \
    rm -rf /var/lib/apt/lists/

### build all
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && catkin config --install && catkin build irsl_choreonoid --no-status --no-notify -p 1 && catkin clean -d -b --logs -y"

### TODO TEST
