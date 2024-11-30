// simulation_governor.h
#ifndef SIMULATION_GOVERNOR_H
#define SIMULATION_GOVERNOR_H
#include "rigid_body.h"
#include <bits/stdc++.h>
#include "../data-structures/oct_tree.h"

namespace sim {
    class SimulationGovernor {
    public:
        SimulationGovernor(float calculation_step_, int calculation_interval_);

        bool startSimulation(std::mutex& mtx);
        bool startOctreeSimulation(std::mutex& mtx);
        void asyncPauseSimulation();
        void pauseSimulation();
        bool saveSimulation(std::mutex& mtx);
        bool loadSimulation();
        void setBodies(std::vector<std::shared_ptr<RigidBody>> bodies);
        std::vector<std::shared_ptr<RigidBody>> getBodies();
        std::vector<std::shared_ptr<RigidBody>> bodies_;
        std::thread thread_;
        bool is_running = false;
        std::shared_ptr<dstruct::OctTree> oct_tree;
    private:
        // in seconds
        float calculation_step_ = 10.0f;
        // in nanoseconds
        int calculation_interval_ = 1000000;
        // indicating whether the simulation is running
        void calculateGravity();
        void updateState();
    };
}

#endif // SIMULATION_GOVERNOR_H