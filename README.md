# mtuav-competition
低空经济智能飞行管理挑战赛 性能赛（BIT-LINC）

# 运行流程
## 准备
开始比赛前检查下主机的时区设置，要求为Beijing时间（UTC+8）；同时做一次时间同步
```sh
sudo ntpdate -b -p 5 -u cn.ntp.org.cn
```
下拉美团、算法开发相关的docker镜像：
```sh
docker pull marcobright2023/mtuav-competition:standalone
docker build -t mtuav_image:1.0 .
```
按照设计，需要starttask后才能走可视化界面，所以后台要先启动一个container用于可视化界面。最好每次编译好程序后，都反复重启这个container。基本运行流程为：启动镜像--启动sdk--打开可视化界面--算法执行--关闭sdk--关闭镜像。

拉好镜像后，首先启动美团镜像，并放置在后台
```sh
docker run -id --name=mtuav-vis -p 8888:8888 -p 50051:50051 -v ./mt-log:/mt-log marcobright2023/mtuav-competition:standalone start
```
通常加载任务运行后，可以看到`./mt-log`中会有相应的日志打印，然后启动算法开发镜像，也放置在后台
```sh
docker run -itd --name=mtuav-alg --gpus all --privileged --network host mtuav_image:1.0
```

## [Main] 进入算法开发镜像里，不断迭代开发算法、启动SDK、打开可视化界面这一流程
```sh
docker exec -it mtuav-alg /bin/bash
```
然后在container里面拉一下本repo的代码
```sh
git clone https://github.com/superboySB/mtuav-competition && cd mtuav-competition
```
由于github限制，要先从[官方SDK下载页面](http://dpurl.cn/lLbhoTvz)里把`map/competition_map.bin`、`map/test_map.bin`文件拖进相应文件夹内（单机测试的时候暂不需要competition_map）。

**每次下拉更新代码、需要重新运行算法时**，需要先重启美团镜像（必要时也可以重启算法container，相当于重启）
```sh
docker stop mtuav-vis mtuav-alg
docker start mtuav-vis mtuav-alg
```
确保服务器执行`netstat -tulp`中有`8888`的监听后，再编译源码运行SDK
```sh
# 编译
mkdir build && cd build && cmake .. && make && make install

# 运行
./mtuav_sdk_example
```
之后，在客户机的浏览器启动网址`http://<server-ip>:8888`，内网（本机工作站）下可以直接该访问服务器。此时m要选择文件在sdk的`visualization/test`内部，一定要同时选择两个`.txt`文件。**注意，SDK在StartTask 后不要退出，退出后服务会关闭任务，同时也会关闭可视化程序。**

[TODO: 目前有问题、必须手动用备选方案启动manager里的start.sh] 在未公开ip的服务器上，可以先在客户端进行端口转发`ssh -L 8888:localhost:8888 -p 17003 ps@36.189.234.178`。


## [Optional] 备选方案
如果浏览器实在看不了可视化，也可以手动启动可视化界面:
```sh
docker exec -it mtuav-vis /bin/bash
cd manager/utmm/server
node server.js
```
注意这个是临时方案，只用于演示，连接SDK前要先关闭。

# 笔记
## 0923
目前只让队长进群，但可以联系这个人(微信15811043941)，周一开放sdk后看看能否进群，现在只有这些[文档](https://github.com/superboySB/mtuav-competition/tree/main/docs)、写的还不错。那个container里主要就是vue.js的前端代码，固定的、所以没什么可看的，都是一些属性信息，所以目前只能是猜测解决方案。

目前留意/drone/udss文件中的15个conf，似乎是一个与无人机配送任务（或类似的 VRP 问题）相关的复杂配置文件。它包括以下几个主要部分：

1. 无人机信息：包括无人机的唯一标识符（ID）、序列号（SN）、起始位置（经度、纬度、海拔、高度）和电量。

2. 任务信息：包括任务的全局唯一标识符（GUID）、对等方 ID（Peer ID）和任务 ID。

3. 地图服务器信息：提供地图服务器的 IP 地址和端口。

4. 货物信息：每个货物（或称为“货物单元”）都有其出生地（起始位置）、索引、名称和重量。

5. 无人机限制配置：这部分可能描述了无人机的各种限制，例如最大载货量、最大飞行范围、最大飞行速度等。

有机会还可以留意一下/manager里面的aoi相关内容，但估计都是静态网页标识，后端没有，不要再浪费时间研究。

考虑到这确实是一个涉及中心控制、离散优化、航迹规划等要素的问题，先看一些有意思的C++/Python code：
1. 与避障相关的path finding问题，或许可以解决无人机target中间的碰撞，例如可以把无人机从质点移动离散化为10m的方格移动，或尝试合进VRP的复杂约束中：[最新的方法（setting不太现实、因为无人机不能交换外卖）](https://kei18.github.io/tswap/)、[可视化（有一系列他的过去方法](https://github.com/kei18/mapf-visualizer)。我想把这个问题提出来做两层优化，主要还是仔细看了比赛文档：`**初赛有固定障碍物，复赛可能还会有移动障碍物**
2. 这好像有一个特别相关的[开源VRP库](https://github.com/VROOM-Project/vroom)，我估计要主力从这个库发力去改，来满足所有约束
3. 这好像是一个不讲武德的[闭源商业软件LocalSolver](https://www.localsolver.com/docs/last/exampletour/time-dependent-capacitated-vehicle-routing-problem-with-time-windows-tdcvrptw.html#)，实在太对口了，但应该不能直接拿来用（虽然我好想先用一下看看强者的结果hh、然后也好处理关注其它的约束），但咱说能不能用来生成ground truth来做learning呢：
4. 之前群里提到的RL4CO，也有[类似settings](https://rl4co.readthedocs.io/en/latest/_collections/tutorials/2-solving-new-problem.html)，看看C++实现RL是否可以：https://github.com/kaist-silab/rl4co
5. 考虑到其实这个比赛要么你就用learning，要么就用不到gpu，而我自己比较感兴趣的[CUDA-enabled爆速求解器](https://developer.nvidia.com/cuopt-logistics-optimization)，fancy但不太好搞，而且问题规模还不至于太大（从单机预览版的预设文件看，应该最多15个无人机、大概200-300个货物），可以先不考虑了。

配置rl4co的样例
```sh
cd rl4co-example && python mdpdp.py
```

配置VROOM的样例
```sh
cd /workspace/mtuav-competition/libvroom-example 

# 编译
make clean && make -j8

# 运行
./libvroom-example
```

这两天我就先用商业软件来做个入门配置吧,后续我还是得手动实现
```sh
# 生成机器校验码，用来去https://www.localsolver.com/my-licenses生成trial license，然后等待可以用了就开编
cd /workspace/mtuav-competition/localsolver-example && lskeygen

# 编译
g++ tdcvrptw.cpp -I/opt/localsolver_12_0/include -llocalsolver120 -lpthread -o tdcvrptw

# 运行
./tdcvrptw instances/R101.25.txt

# [Optional] 这里还有相应的python例子，可以对比C++的实现做上手学习
export PYTHONPATH=/opt/localsolver_12_0/bin/python
python tdcvrptw.py instances/R101.25.txt
```

## 0924
加一个接口案例用来巩固学习pybind11的用法，安装部分已经加入了dockerfile中
```sh
apt-get install libboost-program-options-dev libyaml-cpp-dev clang-tidy clang-format
pip install pytest
git clone https://github.com/pybind/pybind11.git 
cd pybind11
git checkout v2.5.0
mkdir build && cd build && cmake .. && make -j8
make install

cd /workspace && git clone https://github.com/zijinoier/mater
mkdir build
cd build
cmake ..
make -j16 && make install

python3 src/test_pybind.py
```
For detailed API of train_env, please `cd src` and see
```sh
import train_env as te
help(te)
```
这部分还得细细研究，之前做EDA、还有篮球AI的时候天泽也都尝试引入过，但跑通后续、细节很多。这部分看情况也会加进docker、pybind-example里。
