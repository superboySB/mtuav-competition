#include "current_game_info.h"
#include "mtuav_sdk.h"

// TODO 加log打印
namespace mtuav::algorithm {

std::shared_ptr<DynamicGameInfo> DynamicGameInfo::getDynamicGameInfoPtr() {
    std::call_once(singleton_flag, [&] {
        current_game_info = std::shared_ptr<DynamicGameInfo>(new DynamicGameInfo());
    });
    return current_game_info;
}

// 写入dorne、cargo 动态信息
void DynamicGameInfo::udpate_current_info(std::vector<mtuav::DroneStatus> &input_drones,
                                          std::map<int, mtuav::CargoInfo> &input_cargoes) {
    std::lock_guard<std::mutex> lock(this->_info_mutex);
    this->_current_drone_info.clear();
    this->_current_cargo_info.clear();
    for (auto &drone : input_drones) {
        this->_current_drone_info.push_back(drone);
    }
    for (auto &[id, cargo] : input_cargoes) {
        this->_current_cargo_info.insert({id, cargo});
    }
    return;
}

// 获取最新的动态信息
std::tuple<std::vector<mtuav::DroneStatus>, std::map<int, mtuav::CargoInfo>>
DynamicGameInfo::get_current_info() {
    std::lock_guard<std::mutex> lock(this->_info_mutex);
    return {this->_current_drone_info, this->_current_cargo_info};
}

// 设置任务结束标识符
void DynamicGameInfo::set_task_stop_flag(bool f) {
    std::lock_guard<std::mutex> lock(this->_info_mutex);
    this->_task_stop_flag = f;
}

// 获取任务结束标识符
bool DynamicGameInfo::get_task_stop_flag() {
    std::lock_guard<std::mutex> lock(this->_info_mutex);
    return this->_task_stop_flag;
}

}  // namespace mtuav::algorithm
