#ifndef PLANNER_H
#define PLANNER_H

#include <unistd.h>
#include <iostream>
#include "current_game_info.h"
#include "mtuav_sdk.h"

class Planner : public mtuav::PlannerAgent {
   public:
    explicit Planner(std::shared_ptr<mtuav::Map> map)
        : mtuav::PlannerAgent("127.0.0.1:50051", std::move(map)) {}

   protected:
    void OnSdkError(std::string error_msg) override { std::cout << error_msg; }

    // callback function when drone status updated or every 1 second
    void OnTaskStatus(int task_index, std::vector<mtuav::DroneStatus> status,
                      std::map<int, mtuav::CargoInfo> cargos) override {
        // saving to global cache
        // saving to global cache
        // 更新动态信息
        // 除非参赛选手清楚自己在做什么，否则不要修改这里
        auto dynamic_info = mtuav::algorithm::DynamicGameInfo::getDynamicGameInfoPtr();
        this->_count += 1;
        if (this->_count % 20 == 0) {
            std::cout << "OnDroneStatus, index: " << task_index << ", status: " << status.size()
                      << ", cargos: " << cargos.size() << std::endl;
            for (auto& s : status) {
                LOG(INFO) << "drone id: " << s.drone_id << ", status: " << int(s.status)
                          << ", position: " << s.position.x << "-" << s.position.y << "-"
                          << s.position.z << ", cargo id size: " << s.delivering_cargo_ids.size();
                for (auto id : s.delivering_cargo_ids) {
                    LOG(INFO) << "c-id: " << id;
                }
            }
        }

        dynamic_info->udpate_current_info(status, cargos);
    }

    // callback function when task is done or error
    void OnTaskDone(int task_index, bool done, float grade) override {
        std::cout << "OnTaskDone" << std::endl;
        // set finish=true
        // 设置task结束标识符
        // 除非参赛选手清楚自己在做什么，否则不要修改这里
        auto dynamic_info = mtuav::algorithm::DynamicGameInfo::getDynamicGameInfoPtr();
        dynamic_info->set_task_stop_flag(true);
    }

   private:
    int _count = 20;
};

#endif