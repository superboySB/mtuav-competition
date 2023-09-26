FROM nvcr.io/nvidia/pytorch:22.04-py3

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8 LANGUAGE=en_US:en LC_ALL=en_US.UTF-8
RUN apt-get update && apt-get install -y locales && locale-gen en_US.UTF-8
RUN DEBIAN_FRONTEND=noninteractive apt-get install -y tzdata net-tools vim git htop wget curl zip unzip strace valgrind \
    build-essential g++ libssl-dev libasio-dev libglpk-dev gdb libgoogle-glog-dev libboost-program-options-dev cmake \
    libyaml-cpp-dev clang-tidy clang-format libyaml-cpp-dev build-essential ntpdate ca-certificates clang dirmngr doxygen \
    dpkg dpkg-dev file gnupg graphviz iwyu lldb netbase ninja-build npm pkgconf yamllint zlib1g-dev
RUN rm -rf /var/lib/apt/lists/*
RUN ntpdate -b -p 5 -u cn.ntp.org.cn

# 参考1：VROOM的C++算法包 (算是传统方法的优秀实现，速度在100ms级，但可能不太容易应对动态的需求、和复杂的约束，并且编译要求有冲突)
ARG VROOM_RELEASE=v1.13.0
WORKDIR /workspace
RUN echo "Cloning and installing vroom release ${VROOM_RELEASE}..." && \
    git clone --recurse-submodules https://github.com/VROOM-Project/vroom.git
    # cd vroom && \
    # git fetch --tags && \
    # git checkout -q $VROOM_RELEASE && \
    # make -C /workspace/vroom/src -j$(nproc)

# 参考2：LocalSolver (算是商业传统方法的优秀实现，和cuopt类似，区别是纯cpu并行加速，但也有上述问题)
WORKDIR /workspace
RUN wget https://www.localsolver.com/downloads/12_0_20230915/LocalSolver_12_0_20230915_Linux64.run \
    && bash LocalSolver_12_0_20230915_Linux64.run \
    && pip install localsolver -i https://pip.localsolver.com

# 参考3：rl4co（在叶神安利下，这波是比较想用的方法，但需要参考下面的pybind11的例子做一个C++的封装，因为cppRL这个老库的libtorch性能用的很差、并且没有用到显卡加速）
WORKDIR /workspace
RUN git clone https://github.com/kaist-silab/rl4co && cd rl4co && pip install .

# 参考4：pybind的一个例子
WORKDIR /workspace
RUN pip install pytest && git clone https://github.com/pybind/pybind11.git && cd pybind11 && git checkout v2.11.1 \
    && mkdir build && cd build && cmake .. && make -j8 && make install
RUN cd /workspace && git clone https://github.com/zijinoier/mater && cd mater && mkdir build && cd build && cmake .. \
    && make -j16 && make install

# 参考5：动态避障导航算法ORCA
# WORKDIR /workspace
# RUN git clone https://github.com/snape/RVO2-3D && cd RVO2-3D && mkdir build && cd build && cmake ../src \
#     && make -j16 && make install

