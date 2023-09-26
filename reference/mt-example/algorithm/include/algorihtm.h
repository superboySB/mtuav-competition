#ifndef ALGORITHM_H
#define ALGORITHM_H

#include <chrono>
#include <map>
#include <memory>
#include <vector>
#include "current_game_info.h"
#include "mtuav_sdk_planner.h"
#include "mtuav_sdk_types.h"
#include "planner.h"
#include "traj_generation.hpp"

// 用于表示当前无人机信息
using drones_info = std::vector<mtuav::DroneStatus>;
// 用于表示当前订单信息
using cargoes_info = std::map<int, mtuav::CargoInfo>;

using namespace ::mtuav;
namespace mtuav::algorithm {

// 算法基类 用于获取算法求解所需信息
class Algorithm {
   public:
    // update function
    void update_dynamic_info();
    void update_drone_info(const drones_info& latest_drone_info);
    void update_cargo_info(const cargoes_info& latest_cargo_info);
    // * setters
    void set_map_info(std::shared_ptr<Map> input_map);
    void set_task_info(std::unique_ptr<TaskInfo> input_task);
    void set_planner(std::shared_ptr<Planner> input_planner);

    // * slovers
    // * 求解函数，需要选手在自己的算法类中实现
    virtual int64_t solve() = 0;

    // 无人机动态信息
    std::vector<mtuav::DroneStatus> _drone_info;
    // 餐品动态信息
    std::map<int, mtuav::CargoInfo> _cargo_info;
    // 当前比赛场景信息包括换电站位置、无人机可永久停留点的位置等静态信息
    std::unique_ptr<TaskInfo> _task_info;
    // 地图指针， 地图静态信息
    std::shared_ptr<Map> _map;
    // planner指针，用于接入比赛系统
    std::shared_ptr<Planner> _planner;
    // 用于记录生成的flight
    static int64_t flightplan_num;
};

// 参赛选手需要自定义求解算法，可集成自Algorithm基类从而获取到问题信息
class myAlgorithm : public Algorithm {
   public:
    // 需要实现自己的求解函数，从而生成飞行计划
    // solve函数中求解当前环境下算法输出，并传递给仿真系统
    int64_t solve();

    // * 需要选手自行添加所需的函数
    // 示例：给定起点、终点，返回无人机WayPoint飞行轨迹与飞行时间
    std::tuple<std::vector<Segment>, int64_t> waypoints_generation(Vec3 start, Vec3 end);
    // 示例：给定起点、终点与无人机，返回无人机trajectory飞行轨迹与飞行时间
    std::tuple<std::vector<Segment>, int64_t> trajectory_generation(Vec3 start, Vec3 end, DroneStatus drone);
    // 打印segment的信息
    std::string segments_to_string(std::vector<Segment> segs);
};

// TODO: 依据自己的设计添加所需的类，下面举例说明一些常用功能类

// 用于估计当前时刻配送某订单的预期收益
class CargoValueCalculator {
   public:
    int64_t cargo_value(int64_t cargo_id) { return 100; }
};

// 用于计算当前时刻使用某无人机预期带来的收益
class DroneValueCalculator {
   public:
    int64_t drone_value(int64_t drone_id) { return 10; }
};

// 记录算法求解中间状态
class AlgorihtmStatesRecorder {
   public:
    std::vector<int64_t> _drones_to_scheduler;  // 记录用于
    std::vector<int64_t> _cargos_to_delivery;
};


}  // namespace mtuav::algorithm

// namespace mutav::algorithm

#endif