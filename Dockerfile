# Use an Ubuntu base image
FROM ubuntu:22.04

# Set noninteractive mode to avoid timezone prompts
ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies
RUN apt-get update && apt-get install -y \
    g++ \
    cmake \
    git \
    mercurial \
    python3 \
    python3-pip \
    python3-setuptools \
    python3-numpy \
    qt5-qmake \
    qtbase5-dev \
    qtchooser \
    qttools5-dev-tools \
    gdb \
    valgrind \
    gsl-bin \
    libgsl-dev \
    libgsl27 \
    libgtk-3-dev \
    libsqlite3-dev \
    libxml2 \
    libxml2-dev \
    libboost-all-dev \
    libssl-dev \
    liblzma-dev \
    libpcap-dev \
    libopenmpi-dev \
    openmpi-bin \
    && rm -rf /var/lib/apt/lists/*

# Set working directory
WORKDIR /ns3

# Clone NS-3 (adjust version as needed)
RUN git clone --depth 1 --branch ns-3.39 https://gitlab.com/nsnam/ns-3-dev.git ns-3

WORKDIR /ns3/ns-3

# Build NS-3
RUN ./ns3 configure --enable-examples --enable-tests && \
    ./ns3 build

# Copy simulation file
COPY drone-swarm-manet.cc /ns3/ns-3/scratch/

# Compile the simulation
RUN ./ns3 build

# Default command to run the simulation
CMD ["./ns3", "run", "scratch/drone-swarm-manet"]
