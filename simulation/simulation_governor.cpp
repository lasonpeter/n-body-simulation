// simulation_governor.cpp
#include "simulation_governor.h"
#include <raymath.h>

#include "constants.h"
#include "planet.h"

namespace sim {

    SimulationGovernor::SimulationGovernor(float calculation_step_, int calculation_interval_) {
        this->calculation_interval_ = calculation_interval_;
        this->calculation_step_ = calculation_step_;
    }

    ///
    float SimulationGovernor::calculate_theta(RigidBody *body, dstruct::OctTree *oct_tree) {
        if (body == nullptr) {
            std::cout << "NULL BODY" << std::endl;
        }
        return oct_tree->size / Vector3Distance(body->position, oct_tree->center_of_mass);
    }

    void SimulationGovernor::calculate_force(RigidBody *body, dstruct::OctTree *oct_tree) {
        if (body == nullptr) {
            std::cout << "NULL BODY return" << std::endl;
            return;
        }
        if (calculate_theta(body, oct_tree) > theta) {
            float distance = Vector3Distance(body->position, oct_tree->center_of_mass);
            float force = cnst::GRAVITY_CONSTANT * (body->mass * oct_tree->mass) / (distance * distance);
            Vector3 normal = Vector3Subtract(oct_tree->center_of_mass, body->position);
            normal = Vector3Normalize(normal);
            Vector3 vector3force = Vector3Scale(normal, force);
            body->force += vector3force;
        } else {
            if (!oct_tree->children.empty()) {
                for (auto child: oct_tree->children) {
                    calculate_force(body, child);
                }
            }
        }
    }

    struct Chunk {
        std::vector<RigidBody *> bodies;
    };

    void SimulationGovernor::regenerate_tree() {
        delete oct_tree;
        oct_tree = new dstruct::OctTree({0, 0, 0}, 60);
        float great_val = oct_tree->size;
        Vector3 center_of_mass{};
        float general_mass{};
        std::vector<RigidBody *> to_delete{};
        for (auto body: rigid_bodies) {
            if (abs(body->position.x) > great_val) {
                great_val = abs(body->position.x);
                // to_delete.push_back(body);
            } else if (abs(body->position.y) > great_val) {
                great_val = abs(body->position.y);
                // to_delete.push_back(body);
            } else if (abs(body->position.z) > great_val) {
                great_val = abs(body->position.z);
                //to_delete.push_back(body);
            }
            center_of_mass.x += (body->mass * body->position.x + center_of_mass.x * general_mass) / (
                general_mass + body->mass);
            center_of_mass.y += (body->mass * body->position.y + center_of_mass.y * general_mass) / (
                general_mass + body->mass);
            center_of_mass.z += (body->mass * body->position.z + center_of_mass.z * general_mass) / (
                general_mass + body->mass);
            general_mass += body->mass;
        }
        this->oct_tree->size = great_val * 2;
        general_center_of_mass = center_of_mass;
        for (auto body: rigid_bodies) {
            oct_tree->add_body(body);
        }
    }


    //Usuwało referencję po wyjściu z funkcji
    bool SimulationGovernor::start_simulation(std::mutex &mtx) {
        if (is_running)
            return false;
        is_running = true;
        try {
            std::cout << "Starting simulation" << std::endl;
            //std::cout << bodies_.size() << std::endl;
            thread_ = std::thread([&, this]() {
                while (is_running) {
                    {
                        //std::cout<<"-----------------------------------------------"<<std::endl;
                        std::lock_guard<std::mutex> lock(mtx);
                        int thread_count = std::thread::hardware_concurrency();
                        int bodies_per_thread = rigid_bodies.size() / thread_count;
                        std::vector<std::thread> threads{};
                        for (int i = 0; i < thread_count; ++i) {
                            threads.emplace_back([&, i,this]() {
                                for (int x = (i * bodies_per_thread); x < (i + 1) * bodies_per_thread; ++x) {
                                    if (x >= rigid_bodies.size()) {
                                        break;
                                    }
                                    rigid_bodies[x]->force = Vector3Zero();
                                    calculate_force(rigid_bodies[x], oct_tree);
                                }
                            });
                        }

                        for (auto &thread: threads) {
                            thread.join();
                        }
                        for (auto body: rigid_bodies) {
                            Vector3 force = body->force;
                            Vector3 acceleration = Vector3Scale(force, 1.0f / body->mass);
                            body->velocity += acceleration * calculation_step_;
                            body->position += body->velocity * calculation_step_;
                        }
                        regenerate_tree();
                    }
                    std::this_thread::sleep_for(std::chrono::nanoseconds(calculation_interval_));
                }
            });

            std::cout << "Simulation started." << std::endl;
        } catch (const std::exception &e) {
            std::cout << e.what() << std::endl;
            return false;
        }

        std::cout << "Simulation setup complete." << std::endl;
        return true;
    }


    struct StackPair {
        dstruct::OctTree *node;
        char children_index;
    };

    ///
    /// DEAD CODE
    ///
    /// Dead code for the future generations to see and learn from the mistakes of those who failed to succeed
    ///
    /// DEAD CODE
    ///

    void SimulationGovernor::pause_simulation() {
        if (thread_.joinable()) {
            is_running = false;
            thread_.join();
        }
    }


    void SimulationGovernor::async_pause_simulation() {
        is_running = false;
    }

    bool SimulationGovernor::save_simulation(std::mutex &mtx) {
        std::cout << "Saving" << std::endl;
        std::vector<Planet> planets{}; {
            std::lock_guard<std::mutex> lock(mtx);
            for (auto &body: rigid_bodies) {
                Planet planet = *dynamic_cast<Planet *>(body);
                planets.push_back(planet);
            }
        }
        //planets.des
        auto result = glz::write_file_json(planets, "planets.json", std::string{});

        if (result) {
            std::cout << "Serialized JSON: " << std::endl;
        } else {
            std::string descriptive_error = glz::format_error(result);
            std::cerr << "Serialization error: " << descriptive_error << std::endl;
        }
        return true;
    }

    bool SimulationGovernor::load_simulation() {
        std::vector<Planet *> planets;
        std::string buff{};
        auto result = glz::read_file_json(planets, "planets.json", buff);
        if (result.ec == glz::error_code::none) {
            std::cout << "Serialized JSON: " << std::endl;
        } else {
            std::string descriptive_error = glz::format_error(result, buff);
            std::cerr << "Serialization error: " << descriptive_error << std::endl;
            // Additionally print the field that caused the error:
            std::cerr << "Error field: " << result.location << std::endl;
        }

        std::vector<RigidBody *> rigid_bodies;
        for (auto &planet: planets) {
            rigid_bodies.push_back(planet);
        }
        this->rigid_bodies = rigid_bodies;

        return true;
    }
}
