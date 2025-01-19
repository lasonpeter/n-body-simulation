// simulation_governor.cpp
#include "simulation_governor.h"
#include <raymath.h>

#include "constants.h"
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

    float SimulationGovernor::calculate_theta(RigidBody* body, dstruct::OctTree* oct_tree) {
        if (body==nullptr) {
            std::cout<<"NULL BODY"<<std::endl;
        }
        //std::cout<<oct_tree->size/Vector3Distance(body->position,oct_tree->center_of_mass)<<std::endl;
        return  oct_tree->size/Vector3Distance(body->position,oct_tree->center_of_mass);
    }
    //rewrite this one to use the octree
    void SimulationGovernor::calculate_force(RigidBody* body,dstruct::OctTree* oct_tree) {

            if (body == nullptr) {
                std::cout<<"NULL BODY return"<<std::endl;
                return;
            }
            if (calculate_theta(body,oct_tree)>theta) {
                float distance = Vector3Distance(body->position,oct_tree->center_of_mass);
                float force= cnst::GRAVITY_CONSTANT*(body->mass*oct_tree->mass)/(distance*distance);
                Vector3 normal= Vector3Subtract(oct_tree->center_of_mass, body->position);
                normal = Vector3Normalize(normal);
                Vector3 vector3force =Vector3Scale(normal,force);
                body->force += vector3force;
            }else {
                if (!oct_tree->children.empty()) {
                    for (auto child: oct_tree->children) {
                        calculate_force(body,child);
                    }
                }
            }
    }

    struct Chunk {
        std::vector<RigidBody*> bodies;
    };
    void SimulationGovernor::regenerate_tree() {
        delete oct_tree;
        oct_tree= new dstruct::OctTree({0,0,0},60);
        float great_val = oct_tree->size;
        Vector3 center_of_mass{};
        float general_mass{};
        std::vector<RigidBody*> to_delete {};
        for (auto body: rigid_bodies) {
            if(abs(body->position.x)>great_val) {
                great_val=abs(body->position.x);
               // to_delete.push_back(body);
            }else
            if(abs(body->position.y)>great_val) {
                great_val=abs(body->position.y);
               // to_delete.push_back(body);
            }else
            if(abs(body->position.z)>great_val) {
                great_val=abs(body->position.z);
                //to_delete.push_back(body);
            }
            center_of_mass.x += (body->mass*body->position.x + center_of_mass.x*general_mass)/(general_mass+body->mass);
            center_of_mass.y += (body->mass*body->position.y + center_of_mass.y*general_mass)/(general_mass+body->mass);
            center_of_mass.z += (body->mass*body->position.z + center_of_mass.z*general_mass)/(general_mass+body->mass);
            general_mass+=body->mass;
        }
        /*for (auto body : to_delete) {
            // Remove the body from the rigid_bodies vector
            std::erase(rigid_bodies, body);
            // Delete the RigidBody object
            delete body;
        }*/
        this->oct_tree->size=great_val*2;
        //this->theta= theta/(60/great_val);
        general_center_of_mass=center_of_mass;

        //oct_tree->size=great_val*2;
        for (auto body: rigid_bodies) {
            oct_tree->add_body(body);
        }
    }

    //Usuwało referencję po wyjściu z funkcji
    bool SimulationGovernor::start_simulation(std::mutex& mtx) {
        if(is_running)
            return false;
        is_running=true;
        try {
            std::cout << "Starting simulation"<< std::endl;
            //std::cout << bodies_.size() << std::endl;
            thread_ = std::thread([&, this]() {
                while (is_running) {
                    {
                        std::cout<<"-----------------------------------------------"<<std::endl;
                        std::lock_guard<std::mutex> lock(mtx);
                        int thread_count =std::thread::hardware_concurrency();
                        int bodies_per_thread = rigid_bodies.size()/thread_count;
                        std::vector<std::thread> threads{};
                        for (int i = 0; i < thread_count; ++i) {
                            threads.emplace_back([&, i,this]() {
                                for (int x = (i*bodies_per_thread); x <(i+1)*bodies_per_thread; ++x) {
                                    if (x>=rigid_bodies.size()) {
                                        //std::cout<<"BREAK Thread:"<<i<<std::endl;
                                        break;
                                    }
                                    //std::cout<<"Thread:"<<i<<" Body:"<<x<<std::endl;
                                    rigid_bodies[x]->force=Vector3Zero();
                                    calculate_force(rigid_bodies[x],oct_tree);
                                }
                            });
                        }

                        for (auto &thread: threads) {
                            thread.join();
                        }
                        /*
                        for (auto body: rigid_bodies) {
                            body->force=Vector3Zero();
                            CalculateForce(body,oct_tree);
                        }*/
                        for (auto body: rigid_bodies) {
                            //F=m*a
                            //a=F/m
                            //v = a*t + v0
                            //s = v*t
                            Vector3 force = body->force;
                            Vector3 acceleration = Vector3Scale(force, 1.0f/body->mass);
                            body->velocity += acceleration* calculation_step_;
                            body->position += body->velocity*calculation_step_;

                        }
                        regenerate_tree();
                        //std::cout<<"Next"<<std::endl;
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
        dstruct::OctTree* node;
        char children_index;
    };


///
/// DEAD CODE
///
/// Dead code for the future generations to see and learn from the mistakes of those who failed to succeed
///
/// DEAD CODE
///



    //go depth first, then for each leaf node calculate the forces between the leaf node and the neighbouring biggest nodes
    //repeat

    //Go to leaf, and then go to the

    //Uses depth first approach
    /*bool SimulationGovernor::startOctreeSimulation(std::mutex &mtx) {
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
                            }#1#

                            std::stack<StackPair> stack;
                            stack.push(StackPair{this->oct_tree.get(), 0});

                            while (!stack.empty()) {
                                StackPair &current = stack.top();
                                //stack.pop();

                                // Process the current node
                                // To zostaje


                                if (current.node->children.empty()) {

                                    dstruct::OctTree* final_leaf_node = current.node;
                                    dstruct::OctTree* current_node = current.node->parent;
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
                                        stack.push(StackPair{current.node->children[current.children_index], 0});
                                        //std::cout<<"Current index:"<<static_cast<int>(current.children_index)<<std::endl;
                                        // Increment the children_index of the current node to indicate that the next child should be processed next
                                        current.children_index++;
                                    } else if (current.children_index == 8) {
                                        /*for (auto child: current.node->children) {
                                            current.node->mass += child->mass;
                                        }#1#
                                        for (int i = 0; i < 7; ++i) {
                                            Vector3 acceleration{};
                                            for (int x = 0; x < 7; ++x) {
                                                if (x != i) {
                                                    if (current.node->children[i]->mass != 0 && current.node->children[x]->mass != 0) {
                                                        /*auto w = current.node->children[i]->calculateNodeAcceleration(current.node->children[x].get());
                                                        acceleration += w;#1#
                                                    }
                                                }
                                            }
                                            /*for (auto body: current.node->children[i]->bodies_) {
                                                body->acceleration += acceleration;
                                            }#1#
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
                            }#1#
                            for (auto& body: oct_tree->bodies_) {

                                //auto pos = body->position;
                                body->position += body->velocity*calculation_step_;
                                /*pos = body->position - pos;
                                std::cout<<"Delta pos"<<pos.x<<","<<pos.y<<","<<pos.z<<std::endl;#1#
                            }
                            //oct_tree->generateDasOctree();
                            std::cout<<"NEXT STEP"<<std::endl;
                        }
                            std::this_thread::sleep_for(std::chrono::nanoseconds(calculation_interval_));
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
    }*/


    /*bool SimulationGovernor::startOctreeSimulation(std::mutex &mtx) {
        //go into each leaf node
        //calculate the force between the leaf node and neighbouring nodes and or leaf nodes

    }*/

    void SimulationGovernor::pause_simulation() {
        if(thread_.joinable()) {
            is_running = false;
            thread_.join();
        }
    }

    void SimulationGovernor::async_pause_simulation() {
        is_running = false;
    }
    /*bool SimulationGovernor::saveSimulation(std::mutex& mtx) {
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
    }*/

    /*bool SimulationGovernor::loadSimulation() {
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
    }*/
}