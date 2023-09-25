#ifndef CURRENT_GAME_INFO
#define CURRENT_GAME_INFO

#include <iostream>
#include <memory>
#include <mutex>
#include "mtuav_sdk.h"

namespace mtuav::algorithm {
// 动态信息：无人机信息，订单信息
// 线程安全的单例模式（只能生成一个类的实现）
class DynamicGameInfo {
   public:
    static std::shared_ptr<DynamicGameInfo> getDynamicGameInfoPtr();
    // 更新当前比赛信息,选手不需要调用此函数
    void udpate_current_info(std::vector<mtuav::DroneStatus> &input_drones,
                             std::map<int, mtuav::CargoInfo> &input_cargoes);
    // 获取最新的动态信息
    std::tuple<std::vector<mtuav::DroneStatus>, std::map<int, mtuav::CargoInfo>> get_current_info();

    // 设置任务结束标识符
    void set_task_stop_flag(bool f);
    // 获取任务结束标识符
    bool get_task_stop_flag();

   private:
    DynamicGameInfo(){};
    // 将最新的动态信息传递给算法类；

    std::vector<mtuav::DroneStatus> _current_drone_info;   // 无人机信息
    std::map<int, mtuav::CargoInfo> _current_cargo_info;  // 订单信息
    std::mutex _info_mutex;
    bool _task_stop_flag = false;  // 记录任务是否完成
};

// 当前赛况信息指针
static std::shared_ptr<DynamicGameInfo> current_game_info = nullptr;
// 单例标志位
static std::once_flag singleton_flag;
}  // namespace mtuav::algorithm

#endif