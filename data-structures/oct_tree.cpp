//
// Created by xenu on 24.11.24.
//

#include "oct_tree.h"

#include <iostream>
#include <queue>
#include <raymath.h>
#include <thread>

#include "../simulation/constants.h"
#include "../simulation/rigid_body.h"

//0-x<,y<,z<;
//1-x>,y<,z<;
//3-x<,y>,z<;
//4-x>.,y>,z<;


//Calculate great diff once and offset positions by it later on

namespace dstruct {
    OctTree::OctTree(Vector3 position_t,float size_t) {
        position=position_t;
        size=size_t;
    };
    OctTree::OctTree(Vector3 position_t,float size_t, const std::shared_ptr<OctTree> &parent_t) {
        position=position_t;
        size=size_t;
        parent = parent_t;
    };
    /*OctTree::OctTree(Vector3 position_t,std::shared_ptr<OctTree> parent_t) {
        position=position_t;
        parent=parent_t;
    };
    OctTree::OctTree(Vector3 position_t,std::shared_ptr<OctTree> parent_t,char id_t) {
        position=position_t;
        parent=parent_t;
        id=id_t;
    };
    */


    //
    void OctTree::generateOctree() {

        if(&children) {
            children.clear();
        }
        if(bodies_.size()<=MAX_BODIES) {
            return;
        }
        //offsetting the 3D chunk by the *diff*
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        children.push_back(std::make_shared<OctTree>(OctTree(position,size)));
        children.push_back(std::make_shared<OctTree>(OctTree(position,size)));
        children.push_back(std::make_shared<OctTree>(OctTree(position,size)));
        children.push_back(std::make_shared<OctTree>(OctTree(position,size)));
        children.push_back(std::make_shared<OctTree>(OctTree(position,size)));
        children.push_back(std::make_shared<OctTree>(OctTree(position,size)));
        children.push_back(std::make_shared<OctTree>(OctTree(position,size)));
        children.push_back(std::make_shared<OctTree>(OctTree(position,size)));
        children[0]->position;
        //std::cout<<"Position: "<<position.x<<","<<position.y<<","<<position.z<<std::endl;
        //std::cout<<"Position: "<<children[0]->position.x<<","<<children[0]->position.y<<","<<children[0]->position.z<<std::endl;
        for (const auto& body : bodies_) {
            /*if(children[0]) {
                std::cout<<"Is not null"<<std::endl;
            }*/
            if(body->position.x>position.x) {
                if(body->position.y>position.y) {
                    //Consider calculating center of mass of a given chunk, although  it might be useless for small MAX_BODIES values
                    if(body->position.z>position.z) {
                        children[0]->bodies_.push_back(body);
                    }else{
                        children[1]->bodies_.push_back(body);
                    }
                }else{
                    if(body->position.z>position.z) {
                        children[2]->bodies_.push_back(body);
                    }else{
                        children[3]->bodies_.push_back(body);
                    }
                }
            }
            else {
                if(body->position.y>position.y) {
                    if(body->position.z>position.z) {
                        children[4]->bodies_.push_back(body);
                    }else{
                        children[5]->bodies_.push_back(body);
                    }
                }else{
                    if(body->position.z>position.z) {
                        children[6]->bodies_.push_back(body);
                    }else{
                        children[7]->bodies_.push_back(body);
                    }
                }
            }
        }
        for (const auto& child: children) {
            child->generateOctree();
        }
    }

    //cache unfriendly breadth first search
    void OctTree::generateDasOctree() {

        if(!children.empty()) {
            children.clear();
        }
        //find the greatest x or y or z absolute value of the bodies
        float great=0;
        for (const auto& body : bodies_) {
            if(abs(body->position.x)>great) {
                great=abs(body->position.x);
            }
            if(abs(body->position.y)>great) {
                great=abs(body->position.y);
            }
            if(abs(body->position.z)>great) {
                great=abs(body->position.z);
            }
        }
        size=great;
        std::queue<std::shared_ptr<OctTree>> queue{};
        queue.push(std::shared_ptr<OctTree>(this));
        while (!queue.empty()) {
            auto node=queue.front();
            //Create children nodes because the current node has too many bodies
            if(node->bodies_.size()> MAX_BODIES) {
                std::shared_ptr<OctTree> child;
                child = std::make_shared<OctTree>(OctTree(Vector3{node->position.x+node->size/2,node->position.y+node->size/2,node->position.z+node->size/2},node->size/2,node));
                node->children.push_back(child);
                queue.push(child);

                child = std::make_shared<OctTree>(OctTree(Vector3{node->position.x+node->size/2,node->position.y+node->size/2,node->position.z-node->size/2},node->size/2,node));
                node->children.push_back(child);
                queue.push(child);

                child=std::make_shared<OctTree>(OctTree(Vector3{node->position.x+node->size/2,node->position.y-node->size/2,node->position.z+node->size/2},node->size/2,node));
                node->children.push_back(child);
                queue.push(child);

                child=std::make_shared<OctTree>(OctTree(Vector3{node->position.x+node->size/2,node->position.y-node->size/2,node->position.z-node->size/2},node->size/2,node));
                node->children.push_back(child);
                queue.push(child);

                child=std::make_shared<OctTree>(OctTree(Vector3{node->position.x-node->size/2,node->position.y+node->size/2,node->position.z+node->size/2},node->size/2,node));
                node->children.push_back(child);
                queue.push(child);

                child=std::make_shared<OctTree>(OctTree(Vector3{node->position.x-node->size/2,node->position.y+node->size/2,node->position.z-node->size/2},node->size/2,node));
                node->children.push_back(child);
                queue.push(child);

                child=std::make_shared<OctTree>(OctTree(Vector3{node->position.x-node->size/2,node->position.y-node->size/2,node->position.z+node->size/2},node->size/2,node));
                node->children.push_back(child);
                queue.push(child);

                child=std::make_shared<OctTree>(OctTree(Vector3{node->position.x-node->size/2,node->position.y-node->size/2,node->position.z-node->size/2},node->size/2,node));
                node->children.push_back(child);
                queue.push(child);


                //Populate the children
                for (const auto& body : node->bodies_) {
                    /*if(children[0]) {
                        std::cout<<"Is not null"<<std::endl;
                    }*/
                    if(body->position.x>node->position.x) {
                        if(body->position.y>node->position.y) {
                            if(body->position.z>node->position.z) {
                                node->children[0]->bodies_.push_back(body);
                                node->children[0]->mass += body->mass;
                                node->children[0]->center_of_mass = centerOfMass(node->children[0]->center_of_mass,node->children[0]->mass,body->position,body->mass);
                            }else{
                                node->children[1]->bodies_.push_back(body);
                                node->children[1]->mass += body->mass;
                                node->children[1]->center_of_mass = centerOfMass(node->children[1]->center_of_mass,node->children[1]->mass,body->position,body->mass);
                            }
                        }else{
                            if(body->position.z>node->position.z) {
                                node->children[2]->bodies_.push_back(body);
                                node->children[2]->mass += body->mass;
                                node->children[2]->center_of_mass = centerOfMass(node->children[2]->center_of_mass,node->children[2]->mass,body->position,body->mass);
                            }else{
                                node->children[3]->bodies_.push_back(body);
                                node->children[3]->mass += body->mass;
                                node->children[3]->center_of_mass = centerOfMass(node->children[3]->center_of_mass,node->children[3]->mass,body->position,body->mass);
                            }
                        }
                    }
                    else {
                        if(body->position.y>node->position.y) {
                            if(body->position.z>node->position.z) {
                                node->children[4]->bodies_.push_back(body);
                                node->children[4]->mass += body->mass;
                                node->children[4]->center_of_mass = centerOfMass(node->children[4]->center_of_mass,node->children[4]->mass,body->position,body->mass);
                            }else{
                                node->children[5]->bodies_.push_back(body);
                                node->children[5]->mass += body->mass;
                                node->children[5]->center_of_mass = centerOfMass(node->children[5]->center_of_mass,node->children[5]->mass,body->position,body->mass);
                            }
                        }else{
                            if(body->position.z>node->position.z) {
                                node->children[6]->bodies_.push_back(body);
                                node->children[6]->mass += body->mass;
                                node->children[6]->center_of_mass = centerOfMass(node->children[6]->center_of_mass,node->children[6]->mass,body->position,body->mass);
                            }else{
                                node->children[7]->bodies_.push_back(body);
                                node->children[7]->mass += body->mass;
                                node->children[7]->center_of_mass = centerOfMass(node->children[7]->center_of_mass,node->children[7]->mass,body->position,body->mass);
                            }
                        }
                    }
                }
            }
            //std::cout<<"Queue size:"<<queue.size()<<std::endl;
            queue.pop();
        }

    }

    ///
    /// @param oct_tree other octree to calculate against
    /// @param delta_t time difference
    void OctTree::calculateStep(const std::shared_ptr<OctTree> &oct_tree, const float delta_t) {


        //std::cout<<"Calculating step"<<std::endl;
        if(oct_tree->mass ==0 || this->mass ==0) {
            //std::cout<<"all is gone"<<std::endl;
            return;
        }
        Vector3 force{};
        float distance = Vector3Distance(this->center_of_mass,oct_tree->center_of_mass);
        if(distance==0) {
            return;
        }
        float force_mangintude = static_cast<float>(this->mass * oct_tree->mass * cnst::GRAVITY_CONSTANT) / (distance*distance);
        force = Vector3Subtract(oct_tree->center_of_mass,this->center_of_mass);
        force = Vector3Normalize(force);
        force = Vector3Scale(force,force_mangintude);
        //in here it is basically acceleration
        force = force/this->mass;
        /*if(force.x != force.x) {
            //print all the values
            std::cout<<"Force x:"<<force.x<<std::endl;
            std::cout<<"Force y:"<<force.y<<std::endl;
            std::cout<<"Force z:"<<force.z<<std::endl;
            std::cout<<"Distance:"<<distance<<std::endl;
            std::cout<<"Force magnitude:"<<force_mangintude<<std::endl;
            std::cout<<"Mass:"<<this->mass<<std::endl;
            std::cout<<"Mass:"<<oct_tree->mass<<std::endl;

        }*/
        //std::cout << force.x<<std::endl;
        for (auto body: oct_tree->bodies_) {
            body->velocity += (force)*delta_t;
        }
    }

    Vector3 OctTree::centerOfMass(Vector3 vec1, ulong mass1,Vector3 vec2, ulong mass2) {
        Vector3 center_of_mass{};
        center_of_mass.x = (vec1.x * mass1 + vec2.x * mass2) / (mass1 + mass2);
        center_of_mass.y = (vec1.y * mass1 + vec2.y * mass2) / (mass1 + mass2);
        center_of_mass.z = (vec1.z * mass1 + vec2.z * mass2) / (mass1 + mass2);
        return center_of_mass;
    }
}