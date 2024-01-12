# mtuav-competition
第一届低空经济智能飞行管理挑战赛 性能赛（BIT-LINC队伍方案）

目前主要参考：
* 凹包提取算法，C++实现，打包为submodule：https://github.com/JeremyBYU/polylidar
* 基于vis-graph的路径规划算法，GO实现，打包为so文件：https://github.com/fzipp/pathfind
* Dockerfile中给出了其它使用过的第三方库，但均作放弃（包括OR-tools）

[![image.png](https://i.postimg.cc/hGXks0zZ/image.png)](https://postimg.cc/PLkVrYM1)

# 运行流程
## 准备
下拉美团提供的可视化docker镜像：
```sh
docker pull marcobright2023/mtuav-competition:standalone-final
```
下拉自研算法开发相关的docker镜像，如果需要显卡支持，可以参考这个[教程](https://docs.nvidia.com/ngc/gpu-cloud/ngc-user-guide/index.html#generating-api-key)进行NGC API的注册
```sh
docker login nvcr.io
Username: $oauthtoken
Password: <my-api-key>
```
然后就可以拉取镜像了
```sh
docker build -t mtuav_image:1.0 .
```
按照设计，需要starttask后才能走可视化界面，所以后台要先启动一个container用于可视化界面。最好每次编译好程序后，都反复重启这个container。基本运行流程为：启动镜像--启动sdk--打开可视化界面--算法执行--关闭sdk--关闭镜像。

拉好镜像后，首先启动美团镜像，并放置在后台
```sh
docker run -id --name=mtuav-vis -p 8888:8888 -p 50051:50051 -v ./mt-log:/mt-log marcobright2023/mtuav-competition:standalone-final start
```
通常加载任务运行后，可以看到`./mt-log`中会有相应的日志打印，然后启动算法开发镜像，也放置在后台
```sh
docker run -itd --privileged -v /tmp/.X11-unix:/tmp/.X11-unix:ro -e DISPLAY=$DISPLAY --gpus all --network=host --name=mtuav-alg mtuav_image:1.0 /bin/bash
```

## [Main] 进入算法开发镜像里，不断迭代开发算法、启动SDK、打开可视化界面这一流程
```sh
docker exec -it mtuav-alg /bin/bash
```
然后在container里面拉一下本repo的代码
```sh
git clone --recursive https://github.com/superboySB/mtuav-competition && cd mtuav-competition

# 如果你忘记了--recursive，则输入：git submodule update --init --recursive
```
由于github限制，要先从[官方SDK下载页面](http://dpurl.cn/lLbhoTvz)里把`map/final_map.bin`、`map/test_map.bin`文件拖进相应文件夹内（只有在线测试需要final_map.bin）。对于我们算法中的路径规划部分，这是一个单独的module，应该率先生成相应的so文件到本地，
```sh
cd /workspace/mtuav-competition/thirdparty/pathfind
go mod tidy
```
将这些模块放在C++的链接里面
```sh
go build -o /workspace/mtuav-competition/libs/libpathfindwrapper.so -buildmode=c-shared main.go pathfind.go polygonjson.go
```
建议把libs文件夹内同时生成的头文件放在`algorithm/include`内部，便于`cmake`。此外，我还有一个可以可视化效果的单独go模块，可以待选测试
```sh
cd /workspace/mtuav-competition/thirdparty/pathfind/cmd 
go run main.go draw.go polygonjson.go
```

### [本地调试] 连接本地镜像mock server
**每次下拉更新代码、需要重新运行算法时**，需要先重启美团镜像（必要时也可以重启算法container，相当于重启）
```sh
docker stop mtuav-vis mtuav-alg
docker start mtuav-vis mtuav-alg
```
确保服务器执行`netstat -tulp`中有`8888`的监听后，再编译源码运行SDK
```sh
# 直接编译
mkdir -p build && cd build && cmake .. -DFETCHCONTENT_QUIET=OFF -DPYTHON_EXECUTABLE=$(python3 -c "import sys; print(sys.executable)") && make -j8

# 运行
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/workspace/mtuav-competition/libs/ && ./mtuav_sdk_example
```
之后，在客户机的浏览器启动网址`http://<server-ip>:8888`，内网（本机工作站）下可以直接该访问服务器。此时m要选择文件在sdk的`visualization/test`内部，一定要同时选择两个`.txt`文件。**注意，SDK在StartTask 后不要退出，退出后服务会关闭任务，同时也会关闭可视化程序。**

【注意】在未公开ip的服务器上，可以先在客户端进行端口转发`ssh -L 8888:localhost:8888 -p 17003 ps@36.189.234.178`。



### [在线测试]：连接在线比赛系统
**注意**！！开始比赛前检查下主机的时区设置，要求为Beijing时间（UTC+8）；同时做一次时间同步，并且保证core dump没有问题，最好通过修改配置文件来允许生成core（[教程1](https://juejin.cn/post/7068889888527450125)）
```sh
ntpdate -b -p 5 -u cn.ntp.org.cn && ulimit -c unlimited
```
在算法镜像内，**每次下拉更新代码、需要重新运行算法时**，确保源代码中`planner.h`和`sdk_test_main.cpp`中相应的`Planner`和`Login`方法以及`map地址`配置正确，然后直接编译源码运行SDK，具体的编译和运行方式与本地调试一致。注意，不同于单机镜像，在线⽐赛系统地图尺寸更大、任务时间更久，且**没有可视化窗⼝**，⻜机状态只能通过代码来判断，规划时，要给⻜机电量留余量，极限规划可能引发坠机（电量<1%时随机坠机）。建议：先使⽤单机镜像调试代码（地图和场景相对简单），然后连接在线系统进⾏测试。强⼤的算⼒对算法会有些帮助，但不是决定性的。充分利⽤已知信息，提前最好预计算；充分利⽤多核计算能⼒；任何时候都优先考虑⻜⾏安全。


## Useful Tips
```sh
# 解决core dump
bash -c 'echo core-%e > /proc/sys/kernel/core_pattern' && ulimit -c unlimited

sysctl kernel.core_pattern

ulimit -a | grep core

gdb xxx core
or
valgrind --leak-check=full ./mtuav_sdk_example 2> valgrind_output.txt


# SIPP
/workspace/mtuav-competition/build/path_finding /workspace/mtuav-competition/params/task.xml /workspace/mtuav-competition/params/map.xml  /workspace/mtuav-competition/params/config.xml

./path_finding  /workspace/mtuav-competition/params/task.xml /workspace/mtuav-competition/params/map.xml  /workspace/mtuav-competition/params/config.xml

./path_finding  /workspace/mtuav-competition/params/task-drone-003.xml /workspace/mtuav-competition/params/map-drone-003.xml  /workspace/mtuav-competition/params/config.xml
```

## 留念
以下是个人开发过程中的留念，虽然这届比的不是很好，比如我忽视了对已知地图的“以查代算”，总想着假设我在一个开放场景，我只能做适度缓存+临时演算+极致推理加速，后来意识到这个方向有严重失误，不过得幸比赛本身赋予的高自由度，我还是把自己的想法实现到了最后一刻，并且也混了个复赛第7的完赛留念（btw前6名都给老多钱钱了┭┮﹏┭┮）。

[以下是Demo留念，明年再战！](docs/demo.mp4)