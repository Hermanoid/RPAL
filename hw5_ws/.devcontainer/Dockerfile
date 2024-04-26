FROM mcr.microsoft.com/devcontainers/cpp:1-ubuntu-22.04

ARG REINSTALL_CMAKE_VERSION_FROM_SOURCE="3.22.2"

# Optionally install the cmake for vcpkg
COPY ./reinstall-cmake.sh /tmp/

RUN if [ "${REINSTALL_CMAKE_VERSION_FROM_SOURCE}" != "none" ]; then \
        chmod +x /tmp/reinstall-cmake.sh && /tmp/reinstall-cmake.sh ${REINSTALL_CMAKE_VERSION_FROM_SOURCE}; \
    fi \
    && rm -f /tmp/reinstall-cmake.sh

# [Optional] Uncomment this section to install additional vcpkg ports.
# RUN su vscode -c "${VCPKG_ROOT}/vcpkg install <your-port-name-here>"

# [Optional] Uncomment this section to install additional packages.
# RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
#     && apt-get -y install --no-install-recommends <your-package-list-here>

# Install build dependencies
RUN apt-get clean && \
    apt-get update && \
    apt-get install -y --no-install-recommends --fix-missing \
        build-essential binutils \
        ca-certificates cmake cmake-qt-gui curl \
        dbus-x11 \
        ffmpeg \
        gdb gcc g++ gfortran git \
        tar \
        lsb-release \
        procps \
        manpages-dev \
        unzip \
        zip \
        wget \
        xauth \
        swig \
        python3-pip python3-dev python3-numpy python3-distutils \
        python3-setuptools python3-pyqt5 python3-opencv \
        libboost-python-dev libboost-thread-dev libatlas-base-dev libavcodec-dev \
        libavformat-dev libavutil-dev libcanberra-gtk3-module libeigen3-dev \
        libglew-dev libgl1-mesa-dev libgl1-mesa-glx libglib2.0-0 libgtk2.0-dev \
        libgtk-3-dev libjpeg-dev libjpeg8-dev libjpeg-turbo8-dev liblapack-dev \
        liblapacke-dev libopenblas-dev libopencv-dev libpng-dev libpostproc-dev \
        libpq-dev libsm6 libswscale-dev libtbb-dev libtbb2 libtesseract-dev \
        libtiff-dev libtiff5-dev libv4l-dev libx11-dev libxext6 libxine2-dev \
        libxrender-dev libxvidcore-dev libx264-dev libgtkglext1 libgtkglext1-dev \
        libvtk9-dev libdc1394-dev libgstreamer-plugins-base1.0-dev \
        libgstreamer1.0-dev libopenexr-dev \
        openexr \
        pkg-config \
        qv4l2 \
        v4l-utils \
        zlib1g-dev \
        locales \
        && locale-gen en_US.UTF-8 \
        && LC_ALL=en_US.UTF-8 \
        && rm -rf /var/lib/apt/lists/* \
        apt-get clean


# # Install OpenCV 
# ARG OPENCV_VERSION="4.8.0"

# WORKDIR /opencv

# RUN wget -O opencv.zip https://github.com/opencv/opencv/archive/${OPENCV_VERSION}.zip \
#     && wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/${OPENCV_VERSION}.zip \
#     && unzip opencv.zip \
#     && unzip opencv_contrib.zip \
#     && mv opencv-${OPENCV_VERSION} opencv \
#     && mv opencv_contrib-${OPENCV_VERSION} opencv_contrib


# RUN mkdir /opencv/opencv/build
# WORKDIR /opencv/opencv/build

# RUN cmake -D CMAKE_BUILD_TYPE=RELEASE \
#  -D CMAKE_INSTALL_PREFIX=/usr/local \
#  -D INSTALL_PYTHON_EXAMPLES=ON \
#  -D INSTALL_C_EXAMPLES=ON \
#  -D OPENCV_ENABLE_NONFREE=ON \
#  -D OPENCV_GENERATE_PKGCONFIG=ON \
#  -D OPENCV_EXTRA_MODULES_PATH=/opencv/opencv_contrib/modules \
#  -D PYTHON_EXECUTABLE=/usr/local/bin/python \
#  -D BUILD_EXAMPLES=ON .. \
#     && make -j$(nproc) && make install && ldconfig

# Create OPENCV on a layer 
# Build cores should be 1.5x the number of cores on the system (https://stackoverflow.com/questions/414714/compiling-with-g-using-multiple-cores)
ARG OPENCV_VERSION="4.9.0"
# Cut down to save my memory and soul
ARG BUILD_CORES=12 
WORKDIR /
RUN git clone -b ${OPENCV_VERSION} --single-branch --depth 1 https://github.com/opencv/opencv.git && \
    git clone -b ${OPENCV_VERSION} --single-branch --depth 1 https://github.com/opencv/opencv_contrib.git && \
    cd /opencv && \
    mkdir build -p && \
    cd /opencv/build && \
    cmake -D CMAKE_BUILD_TYPE=Release \
    -D WITH_CUDA=OFF  \
    -D BUILD_JAVA=OFF  \
    -D CMAKE_INSTALL_PREFIX=/opencv_install .. \
    -D INSTALL_C_EXAMPLES=OFF \
    -D INSTALL_PYTHON_EXAMPLES=OFF \
    -D OPENCV_GENERATE_PKGCONFIG=ON \
    -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
    -D BUILD_EXAMPLES=OFF ..  \
    -D BUILD_TESTS=OFF \
    -D BUILD_PERF_TESTS=OFF \
    -D BUILD_DOCS=OFF \
    -D BUILD_JAVA=OFF \
    -D BUILD_opencv_python2=OFF \
    -D BUILD_opencv_python3=OFF \
    -D BUILD_opencv_apps=OFF \
    # -D BUILD_opencv_xfeatures2d=OFF && \
    && make -j${BUILD_CORES} && make install && \
    cd / && \
    rm opencv -rf && \
    rm opencv_contrib -rf

RUN ldconfig



# RUN apt-get update 

# RUN apt-get install -y --no-install-recommends --fix-missing \
#     # Open3D
#     xorg-dev \
#     libxcb-shm0 \
#     libglu1-mesa-dev \
#     # Filament build-from-source
#     clang \
#     libc++-dev \
#     libc++abi-dev \
#     libsdl2-dev \
#     ninja-build \
#     libxi-dev \
#     clang-14 \
#     libglu1-mesa-dev \
#     libc++-14-dev \
#     libc++abi-14-dev \
#     libxcomposite-dev \
#     libxxf86vm-dev \
#     # ML
#     libtbb-dev \
#     # Headless rendering
#     libosmesa6-dev \
#     # RealSense
#     # libudev-de \
#     autoconf \ 
#     libtool 


# Set display environment variable to display GUIs in Windows
ENV DISPLAY=host.docker.internal:0



