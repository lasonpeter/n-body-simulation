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

        /**
         * Calculates the theta value for the given body and octree
         * @param body body to calculate theta against
         * @param oct_tree octree to calculate theta against
         * @return
         */
        float calculate_theta(RigidBody *body, dstruct::OctTree *);

        /**
         * Calculates the force acting on the body from the octree
         * @param body body to calculate the force for
         * @param oct_tree octree to calculate the force from
         */
        void calculate_force(RigidBody *body, dstruct::OctTree *oct_tree);

        /**
         * Regenerates the octree from the rigid bodies
         */
        void regenerate_tree();

        /**
         * Starts the simulation
         * @param mtx
         * @return
         */
        bool start_simulation(std::mutex &mtx);


        void async_pause_simulation();

        /**
         * Pauses the simulation
         */
        void pause_simulation();

        /**
         * Saves the simulation to a file
         * @param mtx
         * @return true if successful false otherwise
         */
        bool save_simulation(std::mutex &mtx);

        /**
         * Loads the simulation from a file WARNING ! DOES NOT WORK!
         * @return true if successful false otherwise
         */
        bool load_simulation();


        std::vector<RigidBody *> rigid_bodies;
        std::thread thread_;
        float theta = 0.5f;
        Vector3 general_center_of_mass{};
        float general_mass{};
        bool is_running = false;
        dstruct::OctTree *oct_tree;

    private:
        // in seconds
        float calculation_step_ = 1.0f;
        // in nanoseconds
        int calculation_interval_ = 1000000;
    };
}

#endif // SIMULATION_GOVERNOR_H
