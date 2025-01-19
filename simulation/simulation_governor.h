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

        float calculate_theta(RigidBody *body, dstruct::OctTree*);

        void calculate_force(RigidBody *body, dstruct::OctTree *oct_tree);

        void regenerate_tree();

        bool start_simulation(std::mutex &mtx);
        void async_pause_simulation();
        void pause_simulation();
        bool save_simulation(std::mutex& mtx);
        bool loadSimulation();
        std::vector<RigidBody*> rigid_bodies;
        std::thread thread_;
        float theta = 1.0f;
        Vector3 general_center_of_mass{};
        float general_mass{};
        bool is_running = false;
        dstruct::OctTree* oct_tree;
    private:
        // in seconds
        float calculation_step_ = 1.0f;
        // in nanoseconds
        int calculation_interval_ = 1000000;
    };
}

#endif // SIMULATION_GOVERNOR_H