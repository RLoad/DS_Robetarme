# Use the official ROS Kinetic base image
FROM ros:kinetic
ARG ROS_DISTRO=kinetic
ENV USER ros

# create and configure a new user
ARG UID=1000
ARG GID=1000
RUN addgroup --gid ${GID} ${USER}
RUN adduser --gecos "Remote User" --uid ${UID} --gid ${GID} ${USER} && yes | passwd ${USER}
RUN usermod -a -G dialout ${USER}
RUN echo "${USER} ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/99_aptget
RUN chmod 0440 /etc/sudoers.d/99_aptget && chown root:root /etc/sudoers.d/99_aptget


# install core compilation and access dependencies for building the libraries
RUN apt-get update && apt-get install -y \
    autoconf \
    automake \
    build-essential \
    cmake \
    curl \
    g++ \
    gcc \
    git \
    gnupg2 \
    libtool \
    lsb-release \
    make \
    pkg-config \
    wget \
    bash-completion \
    silversearcher-ag \
    apt-transport-https \
    less \
    alsa-utils \
    ros-${ROS_DISTRO}-ros-control \
    ros-${ROS_DISTRO}-ros-controllers \
    libeigen3-dev \
    ros-${ROS_DISTRO}-vrpn-client-ros \
    ros-${ROS_DISTRO}-diagnostic-updater \
    libarmadillo-dev \
    && sudo apt-get upgrade -y && sudo apt-get clean \
    && rm -rf /var/lib/apt/lists/*
    

SHELL ["/bin/bash", "-c"]

# Set environment variables for the workspace
ENV CATKIN_WS /home/ros/catkin_ws
ENV CATKIN_SRC $CATKIN_WS/src


# Create a new catkin workspace
RUN mkdir -p $CATKIN_SRC && chown -R ros:ros $CATKIN_WS


# Set the working directory
WORKDIR $CATKIN_SRC

#install rospackage from github
RUN git clone -b master https://github.com/RLoad/net-ft-ros.git
RUN git clone -b nadia https://github.com/nbfigueroa/lpvDS-lib.git
RUN git clone -b master https://github.com/epfl-lasa/gaussian-process-regression.git
RUN git clone https://github.com/epfl-lasa/mathlib.git
RUN git clone -b devel https://github.com/epfl-lasa/fast-gmm.git
RUN git clone https://github.com/RLoad/utils.git
RUN git clone https://github.com/RLoad/cFunc.git
RUN git clone -b master https://github.com/nbfigueroa/lagsDS-lib.git
RUN wstool init
RUN wstool merge lagsDS-lib/dependencies.rosinstall 

RUN git clone https://github.com/RLoad/DS_Robetarme.git
RUN cp -r DS_Robetarme/ds_motion_robetarme .
RUN rm DS_Robetarme -r

RUN rosdep install --from-paths . --ignore-src --rosdistro kinetic 
WORKDIR $CATKIN_WS
# Initialize the catkin workspace
RUN source /opt/ros/kinetic/setup.bash && catkin_init_workspace $CATKIN_SRC

# Build the catkin workspace
RUN source /opt/ros/kinetic/setup.bash && cd $CATKIN_WS && catkin_make

# Set up environment variables and ROS setup for user "ros"
RUN echo "source /opt/ros/kinetic/setup.bash" >> /home/ros/.bashrc
RUN echo "source $CATKIN_WS/devel/setup.bash" >> /home/ros/.bashrc

RUN chown -R ros:ros /home/ros/catkin_ws
USER ${USER}

# This command will be executed when you start a shell in the container
CMD ["/bin/bash"]

