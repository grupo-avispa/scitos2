ARG BASE_IMAGE="humble-ros-core-jammy"
ARG MIRA_WS=/opt/mira
ARG OVERLAY_WS=/opt/overlay_ws
ARG SYSTEM=ubuntu-2204lts-x64

# Install MIRA
FROM ubuntu:22.04 AS mira-base
ARG DEBIAN_FRONTEND=noninteractive
ARG MIRA_WS
ARG SYSTEM
RUN apt update && apt install --no-install-recommends -y \
    ca-certificates cmake curl doxygen g++ git subversion unzip wget \
    libxml2-dev libssl-dev \
    libsqlite3-dev libboost-all-dev \
    libogre-1.9-dev libsvn-dev \
    libopencv-dev binutils-dev \
    libiberty-dev libcurl4-gnutls-dev libprocps-dev \
    libqwt-qt5-dev libqt5webkit5-dev libqwtmathml-qt5-dev \
    libqt5opengl5-dev libqt5svg5-dev qt*5-dev qttools5-dev-tools && \
    rm -rf /var/lib/apt/lists/*
RUN curl -o mira-installer-binary.sh https://www.mira-project.org/downloads/mira-installer-binary.sh
RUN chmod +x mira-installer-binary.sh
RUN ./mira-installer-binary.sh -s ${SYSTEM} -d ${MIRA_WS}
# Setup MIRA environment variables
ENV MIRA_PATH=${MIRA_WS}
ENV PATH=$PATH:${MIRA_WS}/bin
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${MIRA_WS}/lib
# Install SCITOS and Tory Configs
RUN mirapackage --addurl ftp://ftp.metralabs-service.com/repos/MIRA-commercial/${SYSTEM}/MIRA-commercial.repo
RUN mirapackage -R
RUN mirapackage --noninteractive -I CANDriver SCITOS SCITOSConfigs ToryConfig

# Build Scitos2
FROM ros:${BASE_IMAGE} AS scitos2-base
ARG MIRA_WS
ARG OVERLAY_WS
ENV MIRA_PATH=${MIRA_WS}
COPY --from=mira-base ${MIRA_WS} ${MIRA_WS}
WORKDIR $OVERLAY_WS
RUN mkdir -p src
COPY . ./src/scitos2

# Install MIRA and ROS2 dependencies
RUN apt update && apt install --no-install-recommends -y \
    libopencv-dev libprocps-dev \
    python3-pip \
    ros-dev-tools \
    python3-vcstool \
    python3-colcon-clean \
    ros-$ROS_DISTRO-rmw-cyclonedds-cpp 
RUN vcs import src < src/scitos2/.github/repos.repos
RUN rosdep init && rosdep update
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    rosdep install -q -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO && \
    rm -rf /var/lib/apt/lists/*
RUN . /opt/ros/$ROS_DISTRO/setup.sh && colcon build
RUN colcon clean workspace --base-select build -y

FROM scitos2-base AS final
ARG MIRA_WS
ARG OVERLAY_WS
ENV MIRA_WS=${MIRA_WS}
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${MIRA_WS}/lib
ENV OVERLAY_WS=${OVERLAY_WS}

WORKDIR $OVERLAY_WS/src/scitos2
COPY ./docker/ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh
COPY ./docker/cyclonedds.xml /
ENTRYPOINT ["/ros_entrypoint.sh"]
