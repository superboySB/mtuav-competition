#include <glog/logging.h>
#include <algorithm>    // C++ STL 算法库
#include "algorithm.h"  // 选手自行设计的算法头文件
#include "math.h"


namespace mtuav::algorithm {
// 算法基类Algorithm函数实现

void Algorithm::update_dynamic_info() {
    auto dynamic_info = DynamicGameInfo::getDynamicGameInfoPtr();
    if (dynamic_info == nullptr) {
        return;
    } else {
        auto [drone_info, cargo_info] = dynamic_info->get_current_info();
        this->_drone_info = drone_info;
        this->_cargo_info = cargo_info;
    }
}

void Algorithm::update_drone_info(const drones_info& latest_drone_info) {
    this->_drone_info.clear();
    for (auto& drone : latest_drone_info) {
        this->_drone_info.push_back(drone);
    }
    return;
}

void Algorithm::update_cargo_info(const cargoes_info& latest_cargo_info) {
    this->_cargo_info.clear();
    for (auto& [id, cargo] : latest_cargo_info) {
        this->_cargo_info[id] = cargo;
    }
    return;
}

void Algorithm::set_task_info(std::unique_ptr<TaskInfo> input_task) {
    this->_task_info = std::move(input_task);
}

void Algorithm::set_map_info(std::shared_ptr<Map> input_map) { this->_map = input_map; }

void Algorithm::set_planner(std::shared_ptr<Planner> input_planner) {
    this->_planner = input_planner;
}

double distance_2D(Vec3 A, Vec3 B) {
    return std::sqrt(std::pow(B.x - A.x, 2) + std::pow(B.y - A.y, 2));
}

double distance_3D(Vec3 A, Vec3 B) {
    return std::sqrt(std::pow(B.x - A.x, 2) + std::pow(B.y - A.y, 2) + std::pow(B.z - A.z, 2));
}

int estimate_total_time(const std::vector<CargoInfo>& selected_cargoes, 
                        const Vec3& initial_drone_position, 
                        int initial_time, 
                        int takeoff_time, 
                        int landing_time, 
                        double exp_speed) {
    int total_time = initial_time;
    Vec3 current_position = initial_drone_position;

    for (const auto& cargo : selected_cargoes) {
        // 计算到货物位置所需的时间
        int time_to_cargo = static_cast<int>( distance_2D(current_position,cargo.position) / exp_speed) 
                                + takeoff_time + landing_time;

        // 计算到目标位置所需的时间
        int time_to_target = static_cast<int>( distance_2D(cargo.position,cargo.target_position) / exp_speed) 
                                + takeoff_time + landing_time;

        // 更新总时间和当前位置
        total_time += time_to_cargo + time_to_target;
        current_position = cargo.target_position;

        // 检查时间限制
        if (total_time > cargo.latest_seconds_left) {
            return -1;  // 表示有一个订单不能在时间内完成
        }
    }

    return total_time;
}

// 使用该估算函数来确定是否接受新的订单。这样可以确保所有选中的订单都能在规定的时间内完成。
// 这里所有的时间单位暂时都用秒叭，速度用m/s
std::vector<CargoInfo> selectAndOrderCargoes( 
        const std::vector<CargoInfo>& available_cargoes,
        const Vec3& drone_position,
        double max_weight,
        int max_orders,
        double exp_speed,
        int takeoff_time,
        int landing_time,
        int current_time) {

    std::vector<CargoInfo> selected_cargoes;
    double current_weight = 0.0;
    Vec3 current_position = drone_position;
    auto remaining_cargoes = available_cargoes;  // 创建一个副本以便我们可以修改它

    while (!remaining_cargoes.empty() && selected_cargoes.size() < max_orders) {
        auto best_cargo_it = std::min_element(remaining_cargoes.begin(), remaining_cargoes.end(),
            [current_position, &current_weight, &current_time, max_weight, exp_speed, takeoff_time, landing_time](const CargoInfo& a, const CargoInfo& b) {
                // 计算到各货物的时间和距离
                int time_to_a = static_cast<int>( distance_2D(current_position,a.position) / exp_speed) + takeoff_time + landing_time
                             + static_cast<int>( distance_2D(a.position,a.target_position) / exp_speed) + takeoff_time + landing_time;
                int time_to_b = static_cast<int>( distance_2D(current_position,b.position) / exp_speed) + takeoff_time + landing_time
                             + static_cast<int>( distance_2D(b.position,b.target_position) / exp_speed) + takeoff_time + landing_time;

                // 计算是否能在限定时间内完成交付
                bool can_deliver_a = (current_time + time_to_a <= a.latest_seconds_left);
                bool can_deliver_b = (current_time + time_to_b <= b.latest_seconds_left);

                // 计算是否超过重量限制
                bool over_weight_a = (current_weight + a.weight > max_weight);
                bool over_weight_b = (current_weight + b.weight > max_weight);

                // 综合考虑
                if (!can_deliver_a && can_deliver_b) return true;
                if (can_deliver_a && !can_deliver_b) return false;
                if (over_weight_a && !over_weight_b) return true;
                if (!over_weight_a && over_weight_b) return false;

                // 如果其他所有条件都相等，则比较奖励与时间的比率
                if ((a.award / time_to_a) != (b.award / time_to_b)) {
                    return (a.award / time_to_a) < (b.award / time_to_b);
                }

                // 在所有其他条件都不能用来决定 "最佳" 订单时，使用 id 来比较
                return a.id < b.id;
            });

        // 尝试添加这个新订单，看看是否所有订单都能在规定时间内完成
        std::vector<CargoInfo> temp_selected_cargoes = selected_cargoes;
        temp_selected_cargoes.push_back(*best_cargo_it);
        
        int estimated_time = estimate_total_time(temp_selected_cargoes, current_position, current_time,
                                                 takeoff_time, landing_time, exp_speed);

        // 如果所有订单都能在规定时间内完成，那么正式添加这个新订单
        if (estimated_time != -1) {
            selected_cargoes.push_back(*best_cargo_it);
            current_weight += best_cargo_it->weight;
            current_position = best_cargo_it->target_position;
            current_time = estimated_time;  // 更新当前时间
        }

        // 从剩余货物列表中删除
        remaining_cargoes.erase(best_cargo_it);
    }

    // 此时，selected_cargoes 包含的都是能在规定时间内完成的订单
    return selected_cargoes;
}

bool areAllElementsMinusOne(const std::vector<int>& vec) {
    return std::all_of(vec.begin(), vec.end(), [](int x) { return x == -1; });
}

void removeAcceptedCargoes(std::vector<CargoInfo>& cargoes_to_delivery_and_no_accepted, 
                            const std::vector<int>& unfinished_order) {
    // 删除所有id出现在unfinished_order中的元素
    cargoes_to_delivery_and_no_accepted.erase(
        std::remove_if(
            cargoes_to_delivery_and_no_accepted.begin(), 
            cargoes_to_delivery_and_no_accepted.end(),
            [&unfinished_order](const CargoInfo& cargo) {
                return std::find(unfinished_order.begin(), unfinished_order.end(), cargo.id) != unfinished_order.end();
            }
        ),
        cargoes_to_delivery_and_no_accepted.end()
    );
}

void removeConflictCargoes(std::vector<CargoInfo>& cargoes_to_delivery_and_no_accepted, 
                           const std::vector<int>& unfinished_order,
                           std::map<int, mtuav::CargoInfo> cargo_info,
                           double safety_distance) {
    // 删除所有可能导致与其他无人机碰撞的货物
    cargoes_to_delivery_and_no_accepted.erase(
        std::remove_if(
            cargoes_to_delivery_and_no_accepted.begin(), 
            cargoes_to_delivery_and_no_accepted.end(),
            [&unfinished_order, safety_distance](const CargoInfo& cargo) {
                for (const auto& other_cargo_id : unfinished_order) {
                    if (other_cargo_id == -1) continue;
                    CargoInfo other_cargo = cargo_info.at(other_cargo_id);
                    if (distance_2D(cargo.position, other_cargo.position) < safety_distance+3) {  //加一个缓冲距离
                        return true;  // 删除这个可能导致碰撞的货物
                    }
                }
                return false;  // 保留这个货物
            }
        ),
        cargoes_to_delivery_and_no_accepted.end()
    );
}

bool operator==(const Vec3& a, const Vec3& b) {
    return a.x == b.x && a.y == b.y && a.z == b.z;
}

// TODO 需要参赛选手自行设计求解算法
// TODO 下面给出一个简化版示例，用于说明无人机飞行任务下发方式
int64_t myAlgorithm::solve() {
    // 处理订单信息，找出可进行配送的订单集合
    std::vector<CargoInfo> cargoes_to_delivery;
    for (auto& [id, cargo] : this->_cargo_info) {
        // 只有当cargo的状态为CARGO_WAITING时，才是当前可配送的订单
        if (cargo.status == CargoStatus::CARGO_WAITING) {
            cargoes_to_delivery.push_back(cargo);
        }
        // 帮助更新状态机中批量送外卖、已完成/再也不能送的外卖
        if (cargo.status == CargoStatus::CARGO_DELIVERED || cargo.status == CargoStatus::CARGO_FAILED){
            for (const auto& pair : my_drone_info) {
                auto& unfinished_cargoes = my_drone_info[pair.first].unfinished_cargo_ids;
                for (int i = unfinished_cargoes.size() - 1; i >= 0; --i) {
                    if (unfinished_cargoes[i] == cargo.id) {
                        unfinished_cargoes.erase(unfinished_cargoes.begin() + i);
                        unfinished_cargoes.push_back(-1);  // 在队尾补充一个-1
                    }
                }
            }
        }
        // TODO 依据订单信息定制化特殊操作
        // 太危险的订单是不是可以不接
    }
    LOG(INFO) << "cargo info size: " << this->_cargo_info.size()
              << ", cargo to delivery size: " << cargoes_to_delivery.size();

    // 处理无人机信息，找出当前未装载货物的无人机集合
    std::vector<DroneStatus> drones_to_pick;
    std::vector<DroneStatus> drones_need_recharge;
    std::vector<DroneStatus> drones_to_delivery;
    std::vector<DroneStatus> drones_to_hover;
    
    for (auto& drone : this->_drone_info) {
        // if (drone.drone_id != "drone-006") continue;  // 1016：先做一架飞机跑通全流程（grade：-1131.500000）
        // 先规划6架飞机，每一架都间距10米
        if (my_drone_info.find(drone.drone_id) == my_drone_info.end()){
            LOG(INFO) << "Now we do not use: " << drone.drone_id;
            continue;
        }

        // drone status为READY时，表示无人机当前没有飞行计划
        LOG(INFO) << "drone status, id: " << drone.drone_id
                  << ", drone status: " << int(drone.status);
        LOG(INFO) << "cargo info:";
        for (auto c : drone.delivering_cargo_ids) {
            LOG(INFO) << "c-id: " << c;
        }
        // 无人机状态为READY
        if (drone.status == Status::READY) {
            bool has_cargo = false;
            for (auto cid : drone.delivering_cargo_ids) {
                LOG(INFO) << "cid = " << cid;
                if (cid != -1) {
                    LOG(INFO) << "has cargo = true";
                    has_cargo = true;
                    break;
                }
            }

            // 如果发现电量够了，就取消目前目标充电站的占用啦
            if (drone.battery > 50){
                my_drone_info[drone.drone_id].target_station_position.x=-1;
                my_drone_info[drone.drone_id].target_station_position.y=-1;
                my_drone_info[drone.drone_id].target_station_position.z=-1;
            }

            if (areAllElementsMinusOne(my_drone_info[drone.drone_id].unfinished_cargo_ids)) { 
                if (drone.battery < 50) {
                    drones_need_recharge.push_back(drone);  // 注意优先去充电
                }
                else{
                    // 我需要无人机的最大重量，最大订单数量，当前时间等信息，选择和排序可配送的订单
                    DroneLimits dl = this->_task_info->drones.front().drone_limits;
                    std::vector<CargoInfo> cargoes_to_delivery_and_no_accepted = cargoes_to_delivery;

                    
                    for (const auto& pair : my_drone_info) {
                        // 删掉已经被其它飞机接单的cargo
                        removeAcceptedCargoes(cargoes_to_delivery_and_no_accepted, pair.second.unfinished_cargo_ids);

                        // 删掉其它飞机未来落点下的cargo（TODO：目前是否过于保守）
                        removeConflictCargoes(cargoes_to_delivery_and_no_accepted, pair.second.unfinished_cargo_ids,
                                            this->_cargo_info, 10);
                    }

                    std::vector<CargoInfo> delivery_order = selectAndOrderCargoes(cargoes_to_delivery_and_no_accepted, 
                                            drone.position, dl.max_weight, dl.max_cargo_slots, 15, 10, 10, 0);

                    std::vector<int> unfinished_order;
                    double total_weight = 0.0;  // 用于跟踪unfinished_order中的总重量
                    while (!delivery_order.empty() && unfinished_order.size() < dl.max_cargo_slots) {
                        // 取出delivery_order中最早加入的元素（在vector的前面）
                        CargoInfo earliest_order = delivery_order.front();
                        
                        // 检查添加这个订单后总重量是否会超过限制
                        if (total_weight + earliest_order.weight <= dl.max_weight) {
                            // 添加到unfinished_order，并更新总重量
                            unfinished_order.push_back(earliest_order.id);
                            total_weight += earliest_order.weight;
                        } else {
                            // 如果添加这个订单会导致超过重量限制，就跳出循环
                            break;
                        }
                        
                        // 从delivery_order中移除已经加入到unfinished_order的元素
                        delivery_order.erase(delivery_order.begin());
                    }

                    if (unfinished_order.size()>0){
                        while (unfinished_order.size() < dl.max_cargo_slots) {
                            unfinished_order.push_back(-1); 
                        }
                        my_drone_info[drone.drone_id].unfinished_cargo_ids = unfinished_order;
                        
                        drones_to_pick.push_back(drone);
                    } 
                }    
            } 
            else {
                if (drone.delivering_cargo_ids != my_drone_info[drone.drone_id].unfinished_cargo_ids){
                    drones_to_pick.push_back(drone); // 没有取到所有货物，先取货物 
                }
                else{
                    if (has_cargo){
                        drones_to_delivery.push_back(drone); // 取到所有货物，开始配送 
                    } 
                }
            }
            continue;
        } 
        // TODO 参赛选手需要依据无人机信息定制化特殊操作
    }
    LOG(INFO) << "drone info size: " << this->_drone_info.size()
              << ", drones to pick cargo: size: " << drones_to_pick.size()
              << ", drones to delivery size: " << drones_to_delivery.size()
              << ", drones need recharge size: " << drones_need_recharge.size();
    LOG(INFO) << "drones to pick cargo: ";
    for (auto d : drones_to_pick) {
        LOG(INFO) << d.drone_id;
    }

    LOG(INFO) << "drones to delivery cargo: ";
    for (auto d : drones_to_delivery) {
        LOG(INFO) << d.drone_id;
    }

    LOG(INFO) << "drones need recharge: ";
    for (auto d : drones_need_recharge) {
        LOG(INFO) << d.drone_id;
    }
    // 获取当前毫秒时间戳
    std::chrono::time_point<std::chrono::system_clock, std::chrono::milliseconds> tp =
        std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
    auto current = std::chrono::duration_cast<std::chrono::milliseconds>(tp.time_since_epoch());
    int64_t current_time = current.count();
    std::vector<std::tuple<std::string, FlightPlan>> flight_plans_to_publish;  //下发飞行任务

    
    //////////////////////////////////////////////////////////////////////////////////////////////////
    // 无人机与订单进行匹配，并生成飞行轨迹
    //////////////////////////////////////////////////////////////////////////////////////////////////
    // 示例策略1：为没有订单的无人机生成取订单航线
    // 取无人机和订单数量较小的值
    for (auto the_drone : drones_to_pick) {
        CargoInfo the_cargo;
        for (size_t j = 0; j < std::min(the_drone.delivering_cargo_ids.size(), 
                    my_drone_info[the_drone.drone_id].unfinished_cargo_ids.size()); ++j) {
            if (the_drone.delivering_cargo_ids[j] != my_drone_info[the_drone.drone_id].unfinished_cargo_ids[j]) {
                int chosen_cargo_id = my_drone_info[the_drone.drone_id].unfinished_cargo_ids[j];
                if (this->_cargo_info.find(chosen_cargo_id) != this->_cargo_info.end()) {
                    auto the_cargo = this->_cargo_info.at(chosen_cargo_id);
                }
                break;
            }
        }
        LOG(INFO) << "go to pick cargo, id: " << the_cargo.id << ", start: " << the_cargo.position.x
                  << " " << the_cargo.position.y << " " << the_cargo.position.z
                  << ", target: " << the_cargo.target_position.x << " "
                  << the_cargo.target_position.y << " " << the_cargo.target_position.z;
        FlightPlan pickup;
        // TODO 参赛选手需要自己实现一个轨迹生成函数或中转点生成函数（trajectory复杂/waypoint简单）
        auto [pickup_waypoints, pickup_flight_time] = this->trajectory_generation(
            the_drone.position, the_cargo.position, the_drone);  //暂时都使用轨迹生成函数，不使用中转点生成函数
        pickup.target_cargo_ids.push_back(the_cargo.id);
        pickup.flight_purpose = FlightPurpose::FLIGHT_TAKE_CARGOS;  // 飞行计划目标
        pickup.flight_plan_type = FlightPlanType::PLAN_TRAJECTORIES;  // 飞行计划类型：轨迹
        pickup.flight_id = std::to_string(++Algorithm::flightplan_num);
        pickup.takeoff_timestamp = current_time;  // 立刻起飞
        pickup.segments = pickup_waypoints;
        // 在下发飞行计划前，选手可以使用该函数自行先校验飞行计划的可行性
        // 注意ValidateFlightPlan 只能校验起点/终点均在地面上的飞行计划
        //（实际比赛时可以注释掉这里）
        // auto reponse_pickup = this->_planner->ValidateFlightPlan(drone_limits, your_flight_plan);
        // LOG(INFO) << "Result of ValidateFlightPlan: " << reponse_pickup;
        
        flight_plans_to_publish.push_back({the_drone.drone_id, pickup});
        LOG(INFO) << "Successfully generated flight plan, flight id: " << pickup.flight_id
                  << ", drone id: " << the_drone.drone_id
                  << ", flight purpose: " << int(pickup.flight_purpose)
                  << ", flight type: " << int(pickup.flight_plan_type)
                  << ", cargo id: " << the_cargo.id;

        // break;  // 每次只生成一条取货飞行计划
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////
    // 示例策略2：为已经取货的飞机生成送货飞行计划
    for (auto the_drone : drones_to_delivery) {
        int the_cargo_id = 0;
        // 找到货仓中第一个id不为-1的货物
        for (auto cid : the_drone.delivering_cargo_ids) {
            if (cid != -1) {
                the_cargo_id = cid;
                break;
            }
        }
        if (this->_cargo_info.find(the_cargo_id) != this->_cargo_info.end()) {
            auto the_cargo = this->_cargo_info.at(the_cargo_id);
            FlightPlan delivery;
            auto [delivery_traj, delivery_flight_time] = this->trajectory_generation(
                the_drone.position, the_cargo.target_position, the_drone);
            if (delivery_flight_time == -1) {
                // 轨迹生成失败
                LOG(INFO) << "trajectory generation failed. ";
                break;
            }
            delivery.flight_purpose = FlightPurpose::FLIGHT_DELIVER_CARGOS;
            delivery.flight_plan_type = FlightPlanType::PLAN_TRAJECTORIES;
            delivery.flight_id = std::to_string(++Algorithm::flightplan_num);
            delivery.takeoff_timestamp = current_time;
            delivery.segments = delivery_traj;
            delivery.target_cargo_ids.push_back(the_cargo.id);
            flight_plans_to_publish.push_back({the_drone.drone_id, delivery});
            LOG(INFO) << "Successfully generated flight plan, flight id: " << delivery.flight_id
                      << ", drone id: " << the_drone.drone_id
                      << ", flight purpose: " << int(delivery.flight_purpose)
                      << ", flight type: " << int(delivery.flight_plan_type)
                      << ", cargo id: " << the_cargo.id;
            // break;  // 每次只生成一条送货飞行计划
        }
    }


    //////////////////////////////////////////////////////////////////////////////////////////////////
    // 示例策略3：为电量小于指定数值的无人机生成换电航线
    for (auto the_drone : drones_need_recharge) {
        auto all_battery_stations = this->_task_info->battery_stations;
        std::vector<Vec3> available_battery_stations;
        for (const auto& station : all_battery_stations) {
            bool occupy_flag = false;
            for (const auto& pair : my_drone_info) {
                auto target = my_drone_info[pair.first].target_station_position;
                if (target == station) {
                    LOG(INFO) << "Charging Station occupied";
                    occupy_flag = true;
                    break;
                }
            }
            if (!occupy_flag){
                available_battery_stations.push_back(station);
            }
        }
        
        // 没有换电站，无法执行换电操作
        if (available_battery_stations.size() == 0) {
            LOG(INFO) << "there is no battery station. ";
            break;
        }
        // 依据距离当前无人机的具体排序
        std::sort(available_battery_stations.begin(), available_battery_stations.end(), [the_drone](Vec3 p1, Vec3 p2) {
            Vec3 the_drone_pos = the_drone.position;
            double p1_to_drone = std::sqrt(std::pow(p1.x - the_drone_pos.x, 2) +
                                           std::pow(p1.y - the_drone_pos.y, 2) +
                                           std::pow(p1.z - the_drone_pos.z, 2));
            double p2_to_drone = std::sqrt(std::pow(p2.x - the_drone_pos.x, 2) +
                                           std::pow(p2.y - the_drone_pos.y, 2) +
                                           std::pow(p2.z - the_drone_pos.z, 2));
            return p1_to_drone < p2_to_drone;
        });
        // 选择距离当前无人机最近的换电站
        int the_station_idx = 0;
        // 依次选择换电站
        Vec3 the_selected_station;
        if (the_station_idx < available_battery_stations.size()) {
            the_selected_station = available_battery_stations.at(the_station_idx);
            my_drone_info[the_drone.drone_id].target_station_position = the_selected_station;
        } else {
            break;
        }
        FlightPlan recharge;
        // TODO 参赛选手需要自己实现一个轨迹生成函数或中转点生成函数
        auto [recharege_traj, recharge_flight_time] = this->trajectory_generation(
            the_drone.position, the_selected_station, the_drone);  //此处使用轨迹生成函数
        recharge.flight_purpose = FlightPurpose::FLIGHT_EXCHANGE_BATTERY;
        recharge.flight_plan_type = FlightPlanType::PLAN_TRAJECTORIES;
        recharge.flight_id = std::to_string(++Algorithm::flightplan_num);
        recharge.takeoff_timestamp = current_time;  // 立刻起飞
        recharge.segments = recharege_traj;
        flight_plans_to_publish.push_back({the_drone.drone_id, recharge});
        if (!recharege_traj.empty()) {
            LOG(INFO) << "first point z: " << recharege_traj.front().position.z;
        } else {
            LOG(WARNING) << "recharege_traj is empty!";
        }
        LOG(INFO) << "Successfully generated flight plan, flight id: " << recharge.flight_id
                  << ", drone id: " << the_drone.drone_id
                  << ", flight purpose: " << int(recharge.flight_purpose)
                  << ", flight type: " << int(recharge.flight_plan_type) << ", cargo id: none";
        // break;  // 每次只生成一条换电飞行计划
    }

    // 下发所求出的飞行计划
    for (auto& [drone_id, flightplan] : flight_plans_to_publish) {
        auto publish_result = this->_planner->DronePlanFlight(drone_id, flightplan);
        LOG(INFO) << "Published flight plan, flight id: " << flightplan.flight_id
                  << ", successfully?: " << std::boolalpha << publish_result.success
                  << ", msg: " << publish_result.msg;
    }

    // TODO 找出需要悬停的无人机
    // 下发无人机悬停指令
    // for (auto& drone : drones_to_hover) {
    //     this->_planner->DroneHover(drone.drone_id);
    //     LOG(INFO) << "Send dorne hover commend, drone id: " << drone.drone_id;
    // }

    // 根据算法计算情况，得出下一轮的算法调用间隔，单位ms
    int64_t sleep_time_ms = 2000;
    // 依据需求计算所需的sleep time，这部分内容移到了主函数内部
    // sleep_time_ms = Calculate_sleep_time();
    return sleep_time_ms;
}


// 1023:自己维护一个恶心心的状态机
void myAlgorithm::update_my_map_and_task_info(DroneStatus drone) {}


// 官方原装的轨迹规划魔改版（复杂，有额外奖励）
std::tuple<std::vector<Segment>, int64_t> myAlgorithm::trajectory_generation(Vec3 start, Vec3 end,
                                                                             DroneStatus drone) {
    // 1023:自己维护一个恶心心的状态机
    my_drone_info[drone.drone_id].start_position = start;
    my_drone_info[drone.drone_id].end_position = end;
    


    double flying_height = my_drone_info[drone.drone_id].flying_height;
    
    std::vector<Segment> traj_segs;
    int64_t flight_time;
    // TODO 选手需要自行设计
    // 获取地图信息
    // this->_map;
    TrajectoryGeneration tg;  // 引用example中的轨迹生成算法
    // 定义四个轨迹点
    Segment p1, p2;
    Vec3 p1_pos, p2_pos;
    p1_pos.x = start.x;
    p1_pos.y = start.y;
    p1_pos.z = start.z;
    p1.position = p1_pos;

    p2_pos.x = start.x;
    p2_pos.y = start.y;
    p2_pos.z = flying_height;
    p2.position = p2_pos;

    p1.seg_type = 0;
    p2.seg_type = 0;
    Segment p3, p4;  // p3 终点上方高度120米，p4 终点
    Vec3 p3_pos;
    p3_pos.x = end.x;
    p3_pos.y = end.y;
    p3_pos.z = flying_height;
    p3.position = p3_pos;
    p3.seg_type = 1;
    p4.position = end;
    p4.seg_type = 2;

    // 获取无人机的性能指标
    // 此处假设所有无人机均为同型号（dzp：这是一个重要假设！！！）
    DroneLimits dl = this->_task_info->drones.front().drone_limits;

    // 生成p1->p2段轨迹点
    std::vector<mtuav::Segment> p1top2_segs;
    bool success_1 = tg.generate_traj_from_waypoints({p1.position, p2.position}, dl, 0, p1top2_segs);
    LOG(INFO) << "p1top2 traj gen: " << std::boolalpha << success_1;
    if (success_1 == false) {
        return {std::vector<mtuav::Segment>{}, -1};
    }
    int64_t p1top2_flight_time = p1top2_segs.back().time_ms;  // p1->p2飞行时间


    // 生成p2->p3段轨迹点（这里需要改进）
    std::vector<mtuav::Segment> p2top3_segs;
    bool success_2 = tg.generate_traj_from_waypoints({p2.position, p3.position}, dl, 1, p2top3_segs);
    LOG(INFO) << "p2top3 traj gen: " << std::boolalpha << success_2;
    if (success_2 == false) {
        return {std::vector<mtuav::Segment>{}, -1};
    }
    int64_t p2top3_flight_time = p2top3_segs.back().time_ms;  // p2->p3飞行时间
    
    
    // 生成p3->p4段轨迹点
    std::vector<mtuav::Segment> p3top4_segs;
    bool success_3 = tg.generate_traj_from_waypoints({p3.position, p4.position}, dl, 2, p3top4_segs);
    LOG(INFO) << "p3top4 traj gen: " << std::boolalpha << success_3;
    if (success_3 == false) {
        return {std::vector<mtuav::Segment>{}, -1};
    }
    int64_t p3top4_flight_time = p3top4_segs.back().time_ms;  // p3->p4飞行时间

    // 合并p1->p4多段轨迹
    // 处理p2->p3段轨 更新轨迹点时间
    int64_t p1top2_last_time = p1top2_segs.back().time_ms;
    LOG(INFO) << "p1top2_last_time " << p1top2_last_time;
    auto first_23 = p2top3_segs.begin();
    LOG(INFO) << "p1top3_FIRST_time " << first_23->time_ms;
    p2top3_segs.erase(first_23);
    LOG(INFO) << "p1top3_second_time " << first_23->time_ms;
    for (int i = 0; i < p2top3_segs.size(); i++) {
        p2top3_segs[i].time_ms = p2top3_segs[i].time_ms + p1top2_last_time;
    }
    // 处理p3->p4段轨 更新轨迹点时间
    int64_t p2top3_last_time = p2top3_segs.back().time_ms;
    LOG(INFO) << "p2top3_last_time " << p2top3_last_time;
    auto first_34 = p3top4_segs.begin();
    LOG(INFO) << "p1top4_FIRST_time " << first_34->time_ms;
    p3top4_segs.erase(first_34);
    LOG(INFO) << "p1top4_FIRST_time " << first_34->time_ms;
    for (int i = 0; i < p3top4_segs.size(); i++) {
        p3top4_segs[i].time_ms = p3top4_segs[i].time_ms + p2top3_last_time;
    }

    // 更新轨迹点时间后，合并轨迹
    std::vector<mtuav::Segment> p1top4_segs;
    p1top4_segs.insert(p1top4_segs.end(), p1top2_segs.begin(), p1top2_segs.end());
    p1top4_segs.insert(p1top4_segs.end(), p2top3_segs.begin(), p2top3_segs.end());
    p1top4_segs.insert(p1top4_segs.end(), p3top4_segs.begin(), p3top4_segs.end());

    // LOG(INFO) << "combined segs detail: ";
    // for (auto s : p1top4_segs) {
    //     LOG(INFO) << "seg, p: " << s.position.x << " " << s.position.y << " " << s.position.z
    //               << ", time_ms: " << s.time_ms << ", a: " << s.a.x << " " << s.a.y << " " << s.a.z
    //               << ", v: " << s.v.x << " " << s.v.y << " " << s.v.z << ", type: " << s.seg_type;
    // }

    // 计算p1->p4时间
    int64_t p1top4_flight_time = p1top2_flight_time + p2top3_flight_time + p3top4_flight_time;

    return {p1top4_segs, p1top4_flight_time};
}

std::string myAlgorithm::segments_to_string(std::vector<Segment> segs) {
    std::string str = "";
    for (auto s : segs) {
        auto x = s.position.x;
        auto y = s.position.y;
        auto z = s.position.z;
        std::string cor =
            "(" + std::to_string(x) + "," + std::to_string(y) + "," + std::to_string(z) + ")->";
        str = str + cor;
    }
    return str;
}

}  // namespace mtuav::algorithm