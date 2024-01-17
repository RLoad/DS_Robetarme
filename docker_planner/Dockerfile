# Handle ros distro
ARG ROS_DISTRO=noetic

FROM ghcr.io/aica-technology/ros-ws:${ROS_DISTRO} as ros-ws

# User provided arguments
ARG HOST_GID=1000
ARG GIT_NAME=""
ARG GIT_EMAIL=""
ARG USE_SIMD=OFF
ARG NB_CPU_RESIZED=6
# Tell docker we want to use bash instead of sh in general
SHELL ["/bin/bash", "-c"]

# Update cmake to a specific version
ARG CMAKE_VERSION="3.22.6"
WORKDIR /home/${USER}/
RUN sudo apt install build-essential libssl-dev
RUN wget https://github.com/Kitware/CMake/releases/download/v${CMAKE_VERSION}/cmake-${CMAKE_VERSION}.tar.gz
RUN tar -zxvf cmake-${CMAKE_VERSION}.tar.gz
WORKDIR /home/${USER}/cmake-${CMAKE_VERSION}
RUN ./bootstrap
RUN make -j  ${NB_CPU_RESIZED} .
RUN sudo make -j  ${NB_CPU_RESIZED} . install

### Add the user to the current GID of the host to avoid permisson issues in volumes
# AICA uses the same name for user and user group
ENV USER_GROUP=${USER}
WORKDIR /home/${USER}/
USER root
RUN if [ "HOST_GID" != "1000"] ; \
    then groupadd --gid ${HOST_GID} host_group && \
    usermod ${USER} -g ${HOST_GID} && \
    usermod ${USER} -a -G ${USER_GROUP}; fi
USER ${USER}

# Setup python version for noetic
RUN sudo apt update
RUN if [ "${ROS_DISTRO}" == "noetic" ] ; \
    then sudo apt install python-is-python3 ; fi
RUN pip install matplotlib

### Add a few tools
RUN sudo apt update && sudo apt install -y \
    bash-completion \
    silversearcher-ag \
    apt-transport-https \
    less \
    alsa-utils \
    ros-${ROS_DISTRO}-vrpn-client-ros \
    ros-${ROS_DISTRO}-ros-control \
    ros-${ROS_DISTRO}-ros-controllers \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-rosparam-shortcuts \
    && sudo apt upgrade -y && sudo apt clean

RUN sudo apt-get install libcgal-dev
FROM ros-ws as inverse-kinematics



# Add f/t package for optitrack
RUN git clone https://github.com/epfl-lasa/net-ft-ros.git


FROM inverse-kinematics as iiwa-dependencies


# Handle SIMD option
RUN if [ "${USE_SIMD}" = "ON" ] ; \
    then export CMAKE_CXX_FLAGS="-march=native -faligned-new" ; fi


# Remove temp files
RUN sudo ldconfig
RUN rm -rf /tmp/*


FROM  iiwa-dependencies as iiwa-simulation

### apt update and clean
RUN sudo apt update && sudo apt upgrade -y && sudo apt clean



RUN sudo apt install -y \
    python3-vcstool 

FROM  iiwa-simulation as finalisation

# Give bashrc back to user
WORKDIR /home/${USER}
RUN sudo chown -R ${USER}:${HOST_GID} .bashrc

# Add cmake option to bash rc if needed
RUN if [ "${USE_SIMD}" = "ON" ] ; \
    then echo "export ENABLE_SIMD=ON" >> /home/${USER}/.bashrc ; fi


FROM  finalisation as catkin-make

### Build ros workspace
WORKDIR /home/${USER}/ros_ws
RUN source /home/${USER}/.bashrc && rosdep install --from-paths src --ignore-src -r -y

RUN source /home/${USER}/.bashrc && catkin_make;

### Final apt clean
RUN sudo apt update && sudo apt upgrade -y && sudo apt clean

RUN echo export ROS_MASTER_URI=http://localhost:11311 >> ~/.bashrc
RUN echo export ROS_IP=localhost >> ~/.bashrc
