// simulation_governor.cpp
#include "simulation_governor.h"
#include <raymath.h>
#include "planet.h"

namespace sim {
    SimulationGovernor::SimulationGovernor(int calculation_step_, int calculation_interval_) {
        this->calculation_interval_ = calculation_interval_;
        this->calculation_step_ = calculation_step_;
    }

    bool SimulationGovernor::startSimulation(std::mutex& mtx) {
        if(is_running)
            return false;
        is_running=true;
        try {
            std::cout << "Starting simulation" << std::endl;
            std::cout << bodies_.size() << std::endl;

            thread_ = std::thread([&, this]() {
                while (is_running) {
                    {
                        std::lock_guard<std::mutex> lock(mtx);
                        std::vector<Vector3> forces(bodies_.size());

                        forces.clear();
                        forces.resize(bodies_.size());

                        for (int i = 0; i < bodies_.size(); ++i) {
                            Vector3 force = {};
                            for (int j = 0; j < bodies_.size(); ++j) {
                                if (i != j) {
                                    force = Vector3Add(force, bodies_[i]->calculateGravityForce(bodies_[j].get()));
                                }
                            }
                            forces[i] = force;
                        }

                        for (int i = 0; i < bodies_.size(); ++i) {
                            bodies_[i]->updateState(&forces[i], calculation_step_);
                            /*std::cout << dynamic_cast<Planet*>(bodies_[i].get())->name << std::endl;
                            std::cout << "x:" << bodies_[i]->position.x << std::endl;
                            std::cout << "y:" << bodies_[i]->position.y << std::endl;
                            std::cout << "z:" << bodies_[i]->position.z << std::endl;*/
                        }
                    }
                    std::this_thread::sleep_for(std::chrono::nanoseconds(calculation_interval_));
                }
            });

            std::cout << "Simulation started." << std::endl;
        } catch (const std::exception& e) {
            std::cout << e.what() << std::endl;
            return false;
        }

        std::cout << "Simulation setup complete." << std::endl;
        return true;
    }

    void SimulationGovernor::setBodies(std::vector<std::shared_ptr<RigidBody>> bodies) {
        this->bodies_ = bodies;
    }

    std::vector<std::shared_ptr<RigidBody>> SimulationGovernor::getBodies() {
        return bodies_;
    }

    void SimulationGovernor::pauseSimulation() {
        if(thread_.joinable()) {
            is_running = false;
            thread_.join();
        }
    }

    void SimulationGovernor::asyncPauseSimulation() {
        is_running = false;
    }
    bool SimulationGovernor::saveSimulation(std::mutex& mtx) {
        std::vector<Planet> planets;
        {
            std::lock_guard<std::mutex> lock(mtx);
            for (auto& body : bodies_) {
                Planet planet = *dynamic_cast<Planet*>(body.get());
                planets.push_back(planet);
            }
        }
        auto result = glz::write_file_json(planets, "planets.json", std::string{});

        if (result) {
            std::cout << "Serialized JSON: " << std::endl;
        } else {
            std::string descriptive_error = glz::format_error(result);
            std::cerr << "Serialization error: " << descriptive_error << std::endl;
        }

        return true;
    }

    bool SimulationGovernor::loadSimulation() {
        std::vector<Planet> planets;
        auto result = glz::read_file_json(planets, "planets.json", std::string{});
        if (result.ec == glz::error_code::none) {
            std::cout << "Serialized JSON: " << std::endl;
        } else {
            std::string descriptive_error = glz::format_error(result);
            std::cerr << "Serialization error: " << descriptive_error << std::endl;
        }

        std::vector<std::shared_ptr<RigidBody>> rigid_bodies;
        for (auto& planet : planets) {
            rigid_bodies.push_back(std::make_shared<Planet>(planet));
        }
        this->bodies_ = rigid_bodies;

        return true;
    }
}