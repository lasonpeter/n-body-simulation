//
// Created by xenu on 21.11.24.
//

#ifndef SIMULATION_GOVERNOR_H
#define SIMULATION_GOVERNOR_H
#include "rigid_body.h"
#include <bits/stdc++.h>;

namespace sim{
class SimulationGovernor {
public:
    SimulationGovernor(int calculation_step_, int calculation_interval_);
    bool startSimulation(std::mutex& mtx);
    void asyncPauseSimulation();
    void pauseSimulation();
    bool saveSimulation(std::mutex& mtx);
    bool loadSimulation();
    void setBodies(std::vector<std::shared_ptr<RigidBody>> bodies);
    std::vector<std::shared_ptr<RigidBody>> getBodies();
    std::vector<std::shared_ptr<RigidBody>> bodies_;
    std::thread thread_;
    bool is_running=false;

private:
    //in seconds
    float calculation_step_=10.0f;
    //in nano seconds
    int calculation_interval_=1000000;
    //indicating whether the simulation is running
    void calculateGravity();
    void updateState();
};}



#endif //SIMULATION_GOVERNOR_H
