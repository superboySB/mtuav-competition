FROM marcobright2023/mtuav-competition:standalone

RUN echo "deb http://archive.ubuntu.com/ubuntu/ focal main restricted universe multiverse" > /etc/apt/sources.list \
    && echo "deb http://archive.ubuntu.com/ubuntu/ focal-updates main restricted universe multiverse" >> /etc/apt/sources.list \
    && echo "deb http://archive.ubuntu.com/ubuntu/ focal-backports main restricted universe multiverse" >> /etc/apt/sources.list \
    && echo "deb http://archive.ubuntu.com/ubuntu/ focal-security main restricted universe multiverse" >> /etc/apt/sources.list

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y net-tools vim git htop cmake wget curl zip \
    unzip python3-pip build-essential g++ libssl-dev libasio-dev libglpk-dev pkg-config gdb

# VROOM的C++算法包
ARG VROOM_RELEASE=v1.13.0
WORKDIR /workspace
RUN echo "Cloning and installing vroom release ${VROOM_RELEASE}..." && \
    git clone  --recurse-submodules https://github.com/VROOM-Project/vroom.git && \
    cd vroom && \
    git fetch --tags && \
    git checkout -q $VROOM_RELEASE && \
    make -C /workspace/vroom/src -j$(nproc)

# LocalSolver
WORKDIR /workspace
RUN wget https://www.localsolver.com/downloads/12_0_20230915/LocalSolver_12_0_20230915_Linux64.run \
    && bash LocalSolver_12_0_20230915_Linux64.run \
    && pip install localsolver -i https://pip.localsolver.com


# rl4co
WORKDIR /workspace
RUN git clone https://github.com/kaist-silab/rl4co && cd rl4co && pip install .

WORKDIR /workspace
