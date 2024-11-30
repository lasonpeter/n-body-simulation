//
// Created by xenu on 24.11.24.
//

#include "oct_tree.h"

#include <iostream>
#include <queue>
#include <raymath.h>
#include <thread>

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
    void OctTree::GenerateDasOctree(std::shared_ptr<OctTree> oct_tree) {

        std::queue<std::shared_ptr<OctTree>> queue{};
        queue.push(oct_tree);
        while (!queue.empty()) {
            auto node=queue.front();
            //Create children nodes because the current node has too many bodies
            if(node->bodies_.size()> MAX_BODIES) {
                std::shared_ptr<OctTree> child;
                child = std::make_shared<OctTree>(OctTree(Vector3{node->position.x+node->size/2,node->position.y+node->size/2,node->position.z+node->size/2},node->size/2));
                node->children.push_back(child);
                queue.push(child);

                child = std::make_shared<OctTree>(OctTree(Vector3{node->position.x+node->size/2,node->position.y+node->size/2,node->position.z-node->size/2},node->size/2));
                node->children.push_back(child);
                queue.push(child);

                child=std::make_shared<OctTree>(OctTree(Vector3{node->position.x+node->size/2,node->position.y-node->size/2,node->position.z+node->size/2},node->size/2));
                node->children.push_back(child);
                queue.push(child);

                child=std::make_shared<OctTree>(OctTree(Vector3{node->position.x+node->size/2,node->position.y-node->size/2,node->position.z-node->size/2},node->size/2));
                node->children.push_back(child);
                queue.push(child);

                child=std::make_shared<OctTree>(OctTree(Vector3{node->position.x-node->size/2,node->position.y+node->size/2,node->position.z+node->size/2},node->size/2));
                node->children.push_back(child);
                queue.push(child);

                child=std::make_shared<OctTree>(OctTree(Vector3{node->position.x-node->size/2,node->position.y+node->size/2,node->position.z-node->size/2},node->size/2));
                node->children.push_back(child);
                queue.push(child);

                child=std::make_shared<OctTree>(OctTree(Vector3{node->position.x-node->size/2,node->position.y-node->size/2,node->position.z+node->size/2},node->size/2));
                node->children.push_back(child);
                queue.push(child);

                child=std::make_shared<OctTree>(OctTree(Vector3{node->position.x-node->size/2,node->position.y-node->size/2,node->position.z-node->size/2},node->size/2));
                node->children.push_back(child);
                queue.push(child);


                //Populate the children
                for (auto body : node->bodies_) {
                    /*if(children[0]) {
                        std::cout<<"Is not null"<<std::endl;
                    }*/
                    if(body->position.x>node->position.x) {
                        if(body->position.y>node->position.y) {
                            if(body->position.z>node->position.z) {
                                node->children[0]->bodies_.push_back(body);
                            }else{
                                node->children[1]->bodies_.push_back(body);
                            }
                        }else{
                            if(body->position.z>node->position.z) {
                                node->children[2]->bodies_.push_back(body);
                            }else{
                                node->children[3]->bodies_.push_back(body);
                            }
                        }
                    }
                    else {
                        if(body->position.y>node->position.y) {
                            if(body->position.z>node->position.z) {
                                node->children[4]->bodies_.push_back(body);
                            }else{
                                node->children[5]->bodies_.push_back(body);
                            }
                        }else{
                            if(body->position.z>node->position.z) {
                                node->children[6]->bodies_.push_back(body);
                            }else{
                                node->children[7]->bodies_.push_back(body);
                            }
                        }
                    }
                }
            }
            //std::cout<<"Queue size:"<<queue.size()<<std::endl;
            queue.pop();
        }

    }

}