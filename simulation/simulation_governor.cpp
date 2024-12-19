// simulation_governor.cpp
#include "simulation_governor.h"
#include <raymath.h>
#include "planet.h"

namespace sim {
    /*SimulationGovernor::SimulationGovernor(float calculation_step_, int calculation_interval_): oct_tree({0,0,0}) {
        this->calculation_interval_ = calculation_interval_;
        this->calculation_step_ = calculation_step_;
    }
    */

    SimulationGovernor::SimulationGovernor(float calculation_step_, int calculation_interval_) {
        this->calculation_interval_ = calculation_interval_;
        this->calculation_step_ = calculation_step_;
    }

    //rewrite this one to use the octree

    bool SimulationGovernor::startSimulation(std::mutex& mtx) {
        if(is_running)
            return false;
        is_running=true;
        try {
            std::cout << "Starting simulation"<< std::endl;
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
                                    //force = Vector3Add(force, bodies_[i]->calculateStep(bodies_[j].get(),calculation_step_));
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


        struct StackPair {
        std::shared_ptr<dstruct::OctTree> node;
        char children_index;
    };


    //go depth first, then for each leaf node calculate the forces between the leaf node and the neighbouring biggest nodes
    //repeat

    //Go to leaf, and then go to the

    //Uses depth first approach
    bool SimulationGovernor::startOctreeSimulation(std::mutex &mtx) {
        if (is_running)
            return false;
        is_running = true;
        try {
            std::cout << "Starting simulation" << std::endl;
            std::cout << bodies_.size() << std::endl;
            int x=0;
            thread_ = std::thread([&, this]() {
                while (is_running) {
                    {
                        {
                            std::lock_guard<std::mutex> lock(mtx);

                            /*
                            for (const auto &body: oct_tree->bodies_) {
                                body->acceleration = {0, 0, 0};
                            }*/

                            std::stack<StackPair> stack;
                            stack.push(StackPair{this->oct_tree, 0});

                            while (!stack.empty()) {
                                StackPair &current = stack.top();
                                //stack.pop();

                                // Process the current node
                                // To zostaje


                                if (current.node->children.empty()) {

                                    std::shared_ptr<dstruct::OctTree> final_leaf_node = current.node;
                                    std::shared_ptr<dstruct::OctTree> current_node = current.node->parent;
                                    while (true) {
                                        if(current_node->parent == nullptr) {
                                            //thats the root
                                            break;
                                        }

                                        auto children = current_node->children;
                                        for (auto oct_tree: children) {
                                            if(oct_tree == current_node) {
                                                std::cout<<"HIT"<<std::endl;
                                                continue;
                                            }
                                            final_leaf_node->calculateStep(oct_tree,calculation_step_);
                                            //SIMULATE AGAINST IT
                                        }
                                        current_node = current_node->parent;
                                    }
                                    for (int i = 0; i < current.node->bodies_.size(); ++i) {
                                        for (int x = 0; x < current.node->bodies_.size(); ++x) {
                                            if (x != i) {
                                                current.node->bodies_[i]->calculateStep(&*current.node->bodies_[x],calculation_step_);
                                                //calculate acceleration
                                            }
                                        }
                                    }
                                    //std::cout<<"    calculated leaf node"<< std::endl;
                                    stack.pop();
                                } else {
                                    if (current.children_index <= 7) {
                                        // Push the next child of the current node to the stack with children_index set to 0
                                        stack.push({current.node->children[current.children_index], 0});
                                        //std::cout<<"Current index:"<<static_cast<int>(current.children_index)<<std::endl;
                                        // Increment the children_index of the current node to indicate that the next child should be processed next
                                        current.children_index++;
                                    } else if (current.children_index == 8) {
                                        /*for (auto child: current.node->children) {
                                            current.node->mass += child->mass;
                                        }*/
                                        for (int i = 0; i < 7; ++i) {
                                            Vector3 acceleration{};
                                            for (int x = 0; x < 7; ++x) {
                                                if (x != i) {
                                                    if (current.node->children[i]->mass != 0 && current.node->children[x]->mass != 0) {
                                                        /*auto w = current.node->children[i]->calculateNodeAcceleration(current.node->children[x].get());
                                                        acceleration += w;*/
                                                    }
                                                }
                                            }
                                            /*for (auto body: current.node->children[i]->bodies_) {
                                                body->acceleration += acceleration;
                                            }*/
                                        }
                                        // Calculate the acceleration between the child nodes themselves
                                        // then pop the stack to return to the root
                                        //std::cout<<"Calculated acceleration between child nodes"<<std::endl;
                                        stack.pop();
                                    }
                                }
                            }
                            //std::cout<<"This was ROOT"<<std::endl;
                            /*for (auto body: oct_tree->bodies_) {
                                body->updateState(calculation_step_);
                            }*/
                            for (auto& body: oct_tree->bodies_) {

                                //auto pos = body->position;
                                body->position += body->velocity*calculation_step_;
                                /*pos = body->position - pos;
                                std::cout<<"Delta pos"<<pos.x<<","<<pos.y<<","<<pos.z<<std::endl;*/
                            }
                            if(x>10) {
                                oct_tree->generateDasOctree();
                                x=0;
                            }
                            x++;
                            std::cout<<"NEXT STEP"<<std::endl;
                            std::this_thread::sleep_for(std::chrono::nanoseconds(calculation_interval_));
                        }
                    }

                    //std::cout<<"calculated "<<oct_tree->bodies_.size()<<" bodies"<< std::endl;

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


    /*bool SimulationGovernor::startOctreeSimulation(std::mutex &mtx) {
        //go into each leaf node
        //calculate the force between the leaf node and neighbouring nodes and or leaf nodes

    }*/
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