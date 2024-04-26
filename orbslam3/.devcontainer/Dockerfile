FROM mcr.microsoft.com/devcontainers/cpp:1-ubuntu-22.04

#Load ORBSLAM3, ORBSLAM3 dependencies and configure environment
COPY ./ORB_SLAM3 /ORB_SLAM3/
COPY ./dependencies /dependencies/
ARG DEBIAN_FRONTEND="noninteractive"
# ENV DISPLAY :0

#Install general dependencies
RUN apt update && apt install -y --no-install-recommends --fix-missing \
    build-essential binutils \
    cmake \
    libopencv-dev \
    libpython2.7-dev \
    freeglut3-dev \
    libglew-dev \
    libboost-dev \
    libboost-serialization-dev \
    libeigen3-dev \
    sudo \
    libssl-dev \
    libgtk2.0-dev \
    pkg-config

#Install Pangolin
WORKDIR /dependencies/Pangolin-0.8
RUN chmod +x ./scripts/install_prerequisites.sh 
RUN ./scripts/install_prerequisites.sh recommended
RUN mkdir build && cd build &&\
    cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-std=c++11 .. &&\
    make -j$(nproc) && make install

#Install openCV
WORKDIR /dependencies/opencv-4.4.0
RUN mkdir build && cd build &&\
    cmake -D CMAKE_BUILD_TYPE=Release -D BUILD_EXAMPLES=OFF  -D BUILD_DOCS=OFF -D BUILD_PERF_TESTS=OFF -D BUILD_TESTS=OFF -D CMAKE_INSTALL_PREFIX=/usr/local -D WITH_GTK=ON -D WITH_OPENGL=ON .. &&\
    make -j$(nproc) && make install

#Build ORBSLAM3
WORKDIR /ORB_SLAM3
RUN chmod +x ./build.sh
RUN ./build.sh
RUN ldconfig

# Import test images
COPY ./MH01 MH01

# For Windows: Set display environment variable to display GUIs 
ENV DISPLAY=host.docker.internal:0
