##############################################################################
##                                 Base Image                               ##
##############################################################################
ARG ROS_DISTRO=jazzy
FROM ros:$ROS_DISTRO
ENV TZ=Europe/Berlin
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

##############################################################################
##                                 Global Dependecies                       ##
##############################################################################
ENV LANG=C.UTF-8 
ENV LC_ALL=C.UTF-8
ENV LC_ALL=C

RUN apt-get update && apt-get upgrade -y \
    && apt-get install --no-install-recommends -y \
    apt-utils \
    sudo \
    nano \
    curl \
    wget \
    build-essential \
    software-properties-common \
    cmake \
    git \
    gcc \
    libpoco-dev \
    libfmt-dev  \
    libeigen3-dev \
    python3-pip \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

##############################################################################
##                                 Create User                              ##
##############################################################################
ARG USER=mujoco_sim_docker
ARG UID=1000
ARG GID=$UID
ARG ROS_DOMAIN_ID=0
ENV USER=${USER}
# Delete user if it exists in container (e.g Ubuntu Noble: ubuntu)
RUN if id -u $UID ; then userdel `id -un $UID` ; fi

# Create the User again
RUN groupadd --gid $GID ${USER} \
    && useradd --uid $UID --gid $GID -m ${USER} \
    && echo "${USER} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USER} \
    && chmod 0440 /etc/sudoers.d/${USER}

# Add DDS profile 
COPY env/dds_profile.xml /home/${USER}
RUN chown ${USER}:${USER} /home/${USER}/dds_profile.xml
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/home/${USER}/dds_profile.xml
RUN echo "export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}" >> /etc/bash.bashrc

# Create base directories
USER ${USER}
RUN mkdir -p /home/${USER}/ros2_ws/src

##############################################################################
##                                 Dependencies                             ##
##############################################################################
USER root

RUN apt-get update && DEBIAN_FRONTEND=noninteractive && \
    apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-rviz2 \
    ros-$ROS_DISTRO-rqt* \
    && rm -rf /var/lib/apt/lists/*

USER ${USER}

##############################################################################
##                                 Build ROS                                ##
##############################################################################
WORKDIR /home/${USER}/ros2_ws
COPY --chown=$USER:$USER ros2_ws/src /home/$USER/ros2_ws/src

WORKDIR /home/$USER/ros2_ws

RUN sudo apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y

RUN . /opt/ros/$ROS_DISTRO/setup.sh && \ 
    colcon build --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --cmake-clean-cache --merge-install

RUN sudo sed --in-place --expression \
    '$isource "/home/$USER/ros2_ws/install/setup.bash"' \
    /ros_entrypoint.sh


CMD ["/bin/bash"]

