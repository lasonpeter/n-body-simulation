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
    OctTree::OctTree(Vector3 position_t,float size_t, OctTree* parent_t) {
        position=position_t;
        size=size_t;
        parent = parent_t;
    }
    void OctTree::updateCenterOfMass(sim::RigidBody* body) {
        //mass+=body->mass;
        {
            center_of_mass.x = (body->mass*body->position.x+center_of_mass.x*mass)/(mass+body->mass);

            //mass+=body->mass;
            center_of_mass.y = (body->mass*body->position.y+center_of_mass.y*mass)/(mass+body->mass);

            //mass+=body->mass;
            center_of_mass.z = (body->mass*body->position.z+center_of_mass.z*mass)/(mass+body->mass);
            mass+=body->mass;
        }
    }

    void OctTree::add_body(sim::RigidBody *body) {
        if (body_count>1) {
            /*std::cout<<"Branch"<<std::endl;
            std::cout<<"Body count:"<<body_count<<std::endl;*/
            body_count++;
            /*std::cout<<"Putting into octant:"<<GetOctant(body)<<std::endl;*/
            updateCenterOfMass(body);
            children[GetOctant(body)]->add_body(body);
        }
        else if (body_count == 0) {
                /*std::cout<<"New leaf"<<std::endl;
                std::cout<<"Body count:"<<body_count<<std::endl;*/
                body_count++;
            updateCenterOfMass(body);
                bodies_.push_back(body);
        }
        else if (body_count==1) {
            /*std::cout<<"Creating new"<<std::endl;
            std::cout<<"Body count:"<<body_count<<std::endl;*/

            // Create new Octant children
            //I
            children.push_back(new OctTree(Vector3{position.x+size/2,position.y+size/2,position.z+size/2},size/2,this));
            //II
            children.push_back(new OctTree(Vector3{position.x-size/2,position.y+size/2,position.z+size/2},size/2,this));
            //III
            children.push_back(new OctTree(Vector3{position.x-size/2,position.y-size/2,position.z+size/2},size/2,this));
            //IV
            children.push_back(new OctTree(Vector3{position.x+size/2,position.y-size/2,position.z+size/2},size/2,this));
            //V
            children.push_back(new OctTree(Vector3{position.x+size/2,position.y+size/2,position.z-size/2},size/2,this));
            //VI
            children.push_back(new OctTree(Vector3{position.x-size/2,position.y+size/2,position.z-size/2},size/2,this));
            //VII
            children.push_back(new OctTree(Vector3{position.x-size/2,position.y-size/2,position.z-size/2},size/2,this));
            //VIII
            children.push_back(new OctTree(Vector3{position.x+size/2,position.y-size/2,position.z-size/2},size/2,this));
            //TODO: This needs to also move the currently existing particle into a new Octant
            //sets previous to the child that the body might belong to
            sim::RigidBody *old = bodies_[0];
            bodies_.pop_back();
            /*std::cout<<"Putting old into octant:"<<GetOctant(old)<<std::endl;*/
            children[GetOctant(old)]->add_body(old);
            /*std::cout<<"Putting new into octant:"<<GetOctant(old)<<std::endl;*/
            updateCenterOfMass(body);
            children[GetOctant(body)]->add_body(body);
            body_count++;
        }
    }

    int OctTree::GetOctant(sim::RigidBody* body) {
        if(body->position.x>position.x) {
            if(body->position.y>position.y) {
                if(body->position.z>position.z) {
                    return 0;
                }else{
                    return 4;
                }
            }else{
                if(body->position.z>position.z) {
                    return 3;
                }else{
                    return 7;
                }
            }
        }
        else {
            if(body->position.y>position.y) {
                if(body->position.z>position.z) {
                    return 1;
                }else{
                    return 5;
                }
            }else{
                if(body->position.z>position.z) {
                    return 2;
                }else{
                    return 6;
                }
            }
        }
        return 0;
    }
}