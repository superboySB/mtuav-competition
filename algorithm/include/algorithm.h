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
#include <unordered_map>
#include <future>

// 用于表示当前无人机信息
using drones_info = std::vector<mtuav::DroneStatus>;
// 用于表示当前订单信息
using cargoes_info = std::map<int, mtuav::CargoInfo>;

// 用于表示我想飞的无人机的信息
struct MyDroneInfo {
    bool has_init;
    bool has_sussessor;
    bool cargo_info_unchanged;
    int init_chosen_station_index;
    int drone_status;
    double drone_battery;
    double flying_height;

    std::vector<std::vector<int>> static_grid;
    std::string map_json;
    std::vector<int> current_cargo_ids;
    std::vector<int> unfinished_cargo_ids;
    std::vector<int> black_cargo_list;
    
    Vec3 drone_position;
    Vec3 target_break_position;  // TODO：还不清楚怎么用
    Vec3 target_charging_position;
    // double next_flight_time;
    // std::vector<mtuav::Segment> path_segs;  // TODO: 后续准备尝试插入悬停段，实现时空上完全无冲突
    // int current_seg_id;

    // 构造函数，可以考虑预置黑名单
    // black_cargo_list({41,337,79,187})
    MyDroneInfo() : flying_height(120), has_sussessor(false), drone_battery(100), 
        has_init(false), unfinished_cargo_ids({-1, -1, -1}),  drone_status(0),
        current_cargo_ids({-1,-1,-1}), cargo_info_unchanged(true)
       {
        target_charging_position.x =-1;
        target_charging_position.y =-1;
        target_charging_position.z =-1;

        target_break_position.x = -1;
        target_break_position.y = -1;
        target_break_position.z = -1;
    }
};


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
    // std::tuple<std::vector<Segment>, int64_t> waypoints_generation(Vec3 start, Vec3 end);
    // 示例：给定起点、终点与无人机，返回无人机trajectory飞行轨迹与飞行时间
    std::tuple<std::vector<Segment>, int64_t> trajectory_generation(Vec3 start, Vec3 end, DroneStatus drone, bool without_taking_off);
    // 打印segment的信息
    std::string segments_to_string(std::vector<Segment> segs);

    float map_min_x, map_max_x, map_min_y, map_max_y, map_min_z, map_max_z;
    std::vector<std::string> unused_drone_id;
    std::unordered_map<std::string, MyDroneInfo> my_drone_info; 
    std::vector<Vec3> generate_waypoints_by_a_star(Vec3 start, Vec3 end, DroneStatus drone);
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