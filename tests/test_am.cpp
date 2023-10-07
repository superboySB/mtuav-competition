#define _USE_MATH_DEFINES
#include <glog/logging.h>
#include <iostream>

#include "sim_am_swarm.hpp"

int main()
{
    nlohmann::json params = Optim::loadJsonFromFile("/workspace/mtuav-competition/algorithm/params/config_sim_swarm.json");

    int start_config = params["start_config"].get<int>(); 
    int end_config = params["end_config"].get<int>(); 
    bool read_config = params["read_config"].get<bool>();
    bool use_model = params["use_model"].get<bool>();
    std :: vector<float> noise = params["noise"].get<std::vector<float>>();
    std :: vector<float> num_drones = params["num_drones"].get<std::vector<float>>();

    for(int j = 0; j < num_drones.size(); j++){
        if(read_config)
            LOG(INFO) << "Agent size = " << num_drones[j] 
                      << " Configuration numbers = " << end_config - start_config + 1;
        int success_trials = 0;
        for(int i = start_config; i < end_config; i++){
            Simulator sim = Simulator(i, read_config, num_drones[j], use_model, noise);
            sim.runSimulation();
            sim.saveMetrics();

            if(sim.success)
                success_trials += 1;
            if(!read_config)
                break;
        }
        LOG(INFO) << "Success Trials = " << success_trials 
                  << " out of " << end_config;
        if(!read_config)
            break;
    }
    return 0;
}