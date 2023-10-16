#include <glog/logging.h>
#include "sim_am_swarm.hpp"

int main()
{
    Simulator sim = Simulator();
    sim.runSimulation();
    sim.saveMetrics();
    if(sim.success){
        LOG(INFO) << "Success Trials" << std::endl;
    }else{
        LOG(INFO) << "Failed" << std::endl;
    }
    return 0;
}