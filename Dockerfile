FROM px4io/px4-dev-ros-noetic

# System Requirements
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8 LANGUAGE=en_US:en LC_ALL=en_US.UTF-8
RUN apt-get update && apt-get install -y locales && locale-gen en_US.UTF-8
RUN cd /home/user && wget https://github.com/Kitware/CMake/releases/download/v3.27.7/cmake-3.27.7-linux-x86_64.tar.gz && \
    tar -zxvf cmake-3.27.7-linux-x86_64.tar.gz && mv cmake-3.27.7-linux-x86_64 /opt/cmake-3.27 && \
    echo 'export PATH=/opt/cmake-3.27/bin:$PATH' >> ~/.bashrc && source ~/.bashrc
RUN DEBIAN_FRONTEND=noninteractive apt-get install -y tzdata net-tools vim git htop wget curl zip unzip build-essential dpkg\
    libssl-dev libglpk-dev gdb libgoogle-glog-dev libboost-program-options-dev cmake ca-certificates clang ntpdate gnupg g++-10\
    libyaml-cpp-dev clang-tidy clang-format lsb-release netbase libnlopt-cxx-dev gfortran
RUN ln -sf /usr/share/zoneinfo/Asia/Shanghai /etc/localtim

# Install OR-tools
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-10 10 && \
    update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-10 10 && \
    update-alternatives --install /usr/bin/cc cc /usr/bin/gcc 30 && \
    update-alternatives --set cc /usr/bin/gcc && \
    update-alternatives --install /usr/bin/c++ c++ /usr/bin/g++ 30 && \
    update-alternatives --set c++ /usr/bin/g++
RUN cd /home/user && git clone https://github.com/google/or-tools && cd or-tools && cmake -S . -B build -DBUILD_DEPS=ON && \
    cmake --build build --config Release --target all -j -v && cmake --build build --config Release --target test -v
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 9 && \
    update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-9 9 && \
    update-alternatives --set gcc /usr/bin/gcc-9 && \
    update-alternatives --set g++ /usr/bin/g++-9

# -------------------------------------------------------------
# 以下是一些用处不会很大的前期参考
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
RUN cd /home/user && git clone https://github.com/kaist-silab/rl4co && cd rl4co && /usr/bin/python3 -m pip install --upgrade pip && pip3 install .

# 参考4：pybind的一个例子（这个基于实时通信的代码不太好打包成Python，很多实现都给藏起来了）
RUN cd /home/user && pip3 install pytest && git clone https://github.com/pybind/pybind11.git && cd pybind11 && \
    git checkout v2.11.1 && mkdir build && cd build && cmake .. && make -j8 && make install
RUN cd /home/user && git clone https://github.com/zijinoier/mater && cd mater && mkdir build && cd build && \
    cmake .. && make -j16 && make install

# 参考5：动态避障导航算法ORCA
#（问题是缺少静态障碍物、加速度约束，并且假设是互惠避障、可扩展性不强）
# WORKDIR /workspace
# RUN git clone https://github.com/snape/RVO2-3D && cd RVO2-3D && mkdir build && cd build && cmake .. && make -j16 && make install

# 参考6：Pybullet-drones开源的swarm规划控制
RUN cd /home/user && git clone --recursive https://github.com/jrl-umi3218/eigen-quadprog && cd eigen-quadprog && mkdir build && \
    cd build && cmake .. && make && make install && ln -s /usr/include/eigen3/Eigen /usr/include/Eigen
RUN mkdir /home/user/swarm_ws && mkdir /home/user/swarm_ws/src && cd /home/user/swarm_ws/src && \
    git clone https://github.com/utiasDSL/AMSwarm
# RUN cd .. && catkin_make && source devel/setup.bash && rosrun amswarm swarm_am_nav
# -------------------------------------------------------------

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc 
WORKDIR /home/user

# [待最终稳定后再加] 在构建镜像时清理 apt 缓存，减小最终镜像的体积
RUN apt-get clean && \
    rm -rf /var/lib/apt/lists/*

CMD ["/bin/bash"]