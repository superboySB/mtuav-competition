# mtuav-competition
低空经济智能飞行管理挑战赛 性能赛（BIT-LINC）

# Requirements
开始比赛前检查下主机的时区设置，要求为Beijing时间（UTC+8）；同时做一次时间同步
```sh
sudo ntpdate -b -p 5 -u cn.ntp.org.cn
```
下拉docker
```sh
docker build -t mtuav_image:1.0 .

docker run -itd --name=mtuav --gpus all --privileged --network host -v ./mt-log:/mt-log mtuav_image:1.0

docker exec -it mtuav /bin/bash
```
在客户端启动网址`http://<server-ip>:8888`，内网（本机工作站）下可以直接访问服务器。在未公开ip地服务器上，可以先在客户端进行端口转发`ssh -L 50051:localhost:50051 -L 8888:localhost:8888 -p 17003 ps@36.189.234.178`

