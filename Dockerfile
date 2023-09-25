FROM nvcr.io/nvidia/pytorch:20.12-py3

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8 LANGUAGE=en_US:en LC_ALL=en_US.UTF-8
RUN apt-get update 
RUN DEBIAN_FRONTEND=noninteractive apt-get install -y tzdata net-tools vim git htop cmake wget curl zip unzip \
    build-essential g++ libssl-dev libasio-dev libglpk-dev pkg-config gdb libgoogle-glog-dev libboost-program-options-dev \
    libyaml-cpp-dev clang-tidy clang-format libyaml-cpp-dev build-essential ntpdate
RUN ntpdate -b -p 5 -u cn.ntp.org.cn

# VROOM的C++算法包 (for C++ 17)  (TODO)
# ARG VROOM_RELEASE=v1.13.0
# WORKDIR /workspace
# RUN echo "Cloning and installing vroom release ${VROOM_RELEASE}..." && \
#     git clone --recurse-submodules https://github.com/VROOM-Project/vroom.git && \
#     cd vroom && \
#     git fetch --tags && \
#     git checkout -q $VROOM_RELEASE && \
#     make -C /workspace/vroom/src -j$(nproc)

# LocalSolver
WORKDIR /workspace
RUN wget https://www.localsolver.com/downloads/12_0_20230915/LocalSolver_12_0_20230915_Linux64.run \
    && bash LocalSolver_12_0_20230915_Linux64.run \
    && pip install localsolver -i https://pip.localsolver.com

# rl4co
WORKDIR /workspace
RUN git clone https://github.com/kaist-silab/rl4co && cd rl4co && pip install .

# pybind的一个例子
WORKDIR /workspace
RUN pip install pytest && git clone https://github.com/pybind/pybind11.git && cd pybind11 && git checkout v2.11.1 \
    && mkdir build && cd build && cmake .. && make -j8 && make install
RUN cd /workspace && git clone https://github.com/zijinoier/mater && cd mater && mkdir build && cd build && cmake .. \
    && make -j16 && make install
