ARG BASE_IMAGE="lyrical-ros-core-resolute"
ARG MIRA_WS=/opt/mira
ARG OVERLAY_WS=/opt/overlay_ws
ARG SYSTEM=ubuntu-2604lts-x64

# Install MIRA
FROM ubuntu:26.04 AS mira-base
ARG DEBIAN_FRONTEND=noninteractive
ARG MIRA_WS
ARG SYSTEM
RUN apt update && apt install --no-install-recommends -y \
    ca-certificates cmake curl doxygen g++ git subversion unzip wget \
    libxml2-dev libssl-dev \
    libsqlite3-dev libboost-all-dev \
    libogre-1.9-dev libsvn-dev \
    libopencv-dev binutils-dev \
    libiberty-dev libcurl4-gnutls-dev libproc2-dev \
    qt6-base-dev qt6-5compat-dev qt6-svg-dev qt6-tools-dev qt6-webengine-dev libgl-dev && \
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
FROM ros:${BASE_IMAGE} AS scitos2-build
ARG MIRA_WS
ARG OVERLAY_WS
ENV MIRA_PATH=${MIRA_WS}
COPY --from=mira-base ${MIRA_WS} ${MIRA_WS}
WORKDIR $OVERLAY_WS
RUN mkdir -p src
COPY . ./src/scitos2

# Install MIRA and ROS2 dependencies
RUN apt update && apt install --no-install-recommends -y \
    libopencv-dev libproc2-dev \
    ros-dev-tools \
    ros-$ROS_DISTRO-rmw-cyclonedds-cpp
RUN rosdep init && rosdep update
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    rosdep install -q -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO && \
    rm -rf /var/lib/apt/lists/*
RUN . /opt/ros/$ROS_DISTRO/setup.sh && colcon build

# List what is needed at runtime: the exec dependencies of the workspace plus the packages that own
# every shared library linked by MIRA and by the installed workspace. The ROS packages are shipped as
# a tarball without headers, because their .deb files depend on the whole development toolchain
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    export LD_LIBRARY_PATH=${MIRA_WS}/lib:$(find install -name '*.so' -printf '%h\n' | sort -u | \
      sed "s|^|$PWD/|" | tr '\n' ':')$LD_LIBRARY_PATH && \
    { rosdep keys --from-paths src --dependency-types=exec --rosdistro $ROS_DISTRO | \
        grep -v '^scitos2' | sort -u | xargs -r rosdep resolve --rosdistro $ROS_DISTRO | \
        grep -v '^#' | tr ' ' '\n'; \
      find ${MIRA_WS} install \( -name '*.so*' -o -type f -perm -u+x \) -exec ldd {} + 2>/dev/null | \
        awk '/=> \//{print $3}' | sort -u | xargs -r readlink -f | xargs -r dpkg-query -S 2>/dev/null | \
        cut -d: -f1; \
    } | sort -u > /runtime-packages.txt && \
    grep -v '^ros-' /runtime-packages.txt > /apt-runtime.txt && \
    { grep '^ros-' /runtime-packages.txt; echo ros-$ROS_DISTRO-rmw-cyclonedds-cpp; } | \
      xargs apt-cache depends --recurse --no-recommends --no-suggests --no-conflicts --no-breaks \
        --no-replaces --no-enhances | grep "^ros-$ROS_DISTRO-" | sort -u | xargs dpkg -L | \
      grep -v -e '/include/' -e '\.a$' -e '/share/doc/' | sort -u | \
      tar -cf /ros-runtime.tar --no-recursion -T - 2>/dev/null

# Runtime image: only MIRA, the installed workspace and the libraries they need
FROM ros:${BASE_IMAGE} AS final
ARG MIRA_WS
ARG OVERLAY_WS
ENV MIRA_WS=${MIRA_WS}
ENV MIRA_PATH=${MIRA_WS}
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${MIRA_WS}/lib
ENV OVERLAY_WS=${OVERLAY_WS}

COPY --from=scitos2-build /apt-runtime.txt /tmp/apt-runtime.txt
RUN apt update && apt install --no-install-recommends -y \
    $(cat /tmp/apt-runtime.txt) && \
    rm -rf /var/lib/apt/lists/* /tmp/apt-runtime.txt
RUN --mount=type=bind,from=scitos2-build,source=/ros-runtime.tar,target=/tmp/ros-runtime.tar \
    tar -xf /tmp/ros-runtime.tar -C /
COPY --from=mira-base ${MIRA_WS} ${MIRA_WS}
COPY --from=scitos2-build ${OVERLAY_WS}/install ${OVERLAY_WS}/install

WORKDIR $OVERLAY_WS
COPY ./docker/ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh
COPY ./docker/cyclonedds.xml /
ENTRYPOINT ["/ros_entrypoint.sh"]
