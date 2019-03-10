FROM gazebo:libgazebo7-xenial

# install packages
ADD https://deb.nodesource.com/setup_11.x /nodesource_setup.sh
RUN bash /nodesource_setup.sh
RUN apt-get update && apt-get install -q -y \
    build-essential \
    cmake \
    imagemagick \
    libboost-all-dev \
    libgts-dev \
    libjansson-dev \
    libtinyxml-dev \
    mercurial \
    nodejs \
    pkg-config \
    psmisc \
    xvfb \
    && rm -rf /var/lib/apt/lists/*

# clone gzweb
ENV GZWEB_WS /root/gzweb
RUN hg clone https://bitbucket.org/osrf/gzweb $GZWEB_WS
WORKDIR $GZWEB_WS

# build gzweb
RUN hg up gzweb_1.4.0 \
    && npm run deploy


# install packages
RUN apt-get update && apt-get install -q -y \
    dirmngr \
    gnupg2 \
    lsb-release \
    && rm -rf /var/lib/apt/lists/*

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 421C365BD9FF1F717815A3895523BAEEB01FA116

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    python-rosdep \
    python-rosinstall \
    python-vcstools \
    && rm -rf /var/lib/apt/lists/*

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# bootstrap rosdep
RUN rosdep init \
    && rosdep update

# install ros packages
ENV ROS_DISTRO kinetic
RUN apt-get update && apt-get install -y \
    ros-kinetic-ros-core=1.3.2-0* \
    ros-kinetic-gazebo-ros-pkgs \ 
    ros-kinetic-gazebo-ros-control \
    ros-kinetic-xacro \
    && rm -rf /var/lib/apt/lists/*

# setup environment
EXPOSE 8080
EXPOSE 7681
EXPOSE 11345

COPY ./entrypoint.sh /
ENTRYPOINT [ "/entrypoint.sh" ]
CMD bash