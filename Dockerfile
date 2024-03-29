FROM nvcr.io/nvidia/pytorch:22.12-py3

# System Requirements
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8 LANGUAGE=en_US:en LC_ALL=en_US.UTF-8
RUN apt-get update && apt-get install -y locales && locale-gen en_US.UTF-8
RUN DEBIAN_FRONTEND=noninteractive apt-get install -y tzdata net-tools vim git htop wget curl zip unzip build-essential dpkg \
    iputils-ping libssl-dev libglpk-dev gdb libgoogle-glog-dev libboost-program-options-dev cmake ca-certificates clang ntpdate gnupg \
    clang-tidy clang-format lsb-release netbase valgrind tmux libsuperlu-dev libfftw3-dev libadolc-dev libmpfr-dev 
RUN ln -sf /usr/share/zoneinfo/Asia/Shanghai /etc/localtime && echo "Asia/Shanghai" > /etc/timezone && \
    ntpdate -b -p 5 -u cn.ntp.org.cn

# Install OR-tools
# RUN cd /workspace && git clone https://github.com/google/or-tools
# RUN cd /workspace/or-tools && cmake -S . -B build -DBUILD_DEPS=ON && \
#     cmake --build build --config Release --target all -j -v && \
#     cmake --build build --config Release --target test -v && \
#     cmake --build build --config Release --target install -v

# Install Polylidar
RUN cd /workspace && git clone https://github.com/superboySB/polylidar && cd polylidar && mkdir build && cd build && \
    cmake .. -DFETCHCONTENT_QUIET=OFF -DPYTHON_EXECUTABLE=$(python3 -c "import sys; print(sys.executable)") && \
    make -j8 && ./bin/polylidar-simple
RUN pip install --upgrade pip && cmake --build . --target python-package --config Release -j4 && cd lib/python_package && pip install -e . && \
    cd ../../../ && pip install -r dev-requirements.txt

# Install Golong
RUN cd /workspace/ && wget https://dl.google.com/go/go1.21.3.linux-amd64.tar.gz && tar -C /usr/local -xzf go1.21.3.linux-amd64.tar.gz && \
    echo "export PATH=$PATH:/usr/local/go/bin" >> ~/.bashrc


# -------------------------------------------------------------
# 以下很多是一些用处不大的前期参考
#
# 参考1：VROOM的C++算法包 (算是传统方法的优秀实现，速度在100ms级，但可能不太容易应对动态的需求、和复杂的约束，并且编译要求有冲突)
# （没啥用）
# ARG VROOM_RELEASE=v1.13.0
# WORKDIR /workspace
# RUN echo "Cloning and installing vroom release ${VROOM_RELEASE}..." && \
#     git clone --recurse-submodules https://github.com/VROOM-Project/vroom.git
    # cd vroom && \
    # git fetch --tags && \
    # git checkout -q $VROOM_RELEASE && \
    # make -C /workspace/vroom/src -j$(nproc)

# 参考2：LocalSolver (算是商业传统方法的优秀实现，和cuopt类似，区别是纯cpu并行加速，但也有上述问题)
# （没啥用）
# WORKDIR /workspace
# RUN wget https://www.localsolver.com/downloads/12_0_20230915/LocalSolver_12_0_20230915_Linux64.run \
#     && bash LocalSolver_12_0_20230915_Linux64.run \
#     && pip install localsolver -i https://pip.localsolver.com

# 参考3：rl4co，在叶神安利下，这波是比较想用的方法，但需要参考下面的pybind11的例子做一个C++的封装，因为cppRL这个老库的libtorch性能用的很差、并且没有用到显卡加速
# （从目前SDK的封装实现来说，只能局部给一些api打包成python接口，全部打包比较困难）
# RUN cd /workspace && git clone https://github.com/kaist-silab/rl4co && cd rl4co && pip install --upgrade pip && pip install .

# 参考4：pybind的一个例子，用来设计python SDK
#（这个基于实时通信的代码不太好打包成Python，很多实现都给藏起来了）
# RUN cd /workspace && pip install pytest imageio && git clone https://github.com/pybind/pybind11.git && cd pybind11 && \
#     git checkout v2.11.1 && mkdir build && cd build && cmake .. && make -j8 && make install
# RUN cd /workspace && git clone https://github.com/zijinoier/mater && cd mater && mkdir build && cd build && \
#     cmake .. && make -j16 && make install

# 参考5：动态避障导航算法ORCA
#（问题是缺少静态障碍物、加速度约束，并且假设是互惠避障、可扩展性不强）
# WORKDIR /workspace
# RUN git clone https://github.com/snape/RVO2-3D && cd RVO2-3D && mkdir build && cd build && cmake .. && make -j16 && make install

# 参考6：Pybullet-drones开源的swarm规划控制
# （这次的比赛没有给MPC+贝塞尔曲线的规划空间）
# RUN apt-get install -y doxygen graphviz libeigen3-dev libboost-test-dev
# RUN cd /workspace && git clone --recursive https://github.com/superboySB/eigen-quadprog && cd eigen-quadprog && mkdir build && \
#     cd build && cmake .. && make && make install && ln -s /usr/include/eigen3/Eigen /usr/include/Eigen
# RUN mkdir /workspace/swarm_ws && mkdir /workspace/swarm_ws/src && cd /workspace/swarm_ws/src && \
#     git clone https://github.com/utiasDSL/AMSwarm
# RUN cd .. && catkin_make && source devel/setup.bash && rosrun amswarm swarm_am_nav
# -------------------------------------------------------------

WORKDIR /workspace

# [待最终稳定后再加] 在构建镜像时清理 apt 缓存，减小最终镜像的体积
# RUN apt-get clean && \
#     rm -rf /var/lib/apt/lists/*

CMD ["/bin/bash"]
