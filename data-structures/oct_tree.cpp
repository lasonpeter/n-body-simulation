//
// Created by xenu on 24.11.24.
//

#include "oct_tree.h"

#include <iostream>
#include <thread>

#include "../simulation/rigid_body.h"

//0-x<,y<,z<;
//1-x>,y<,z<;
//3-x<,y>,z<;
//4-x>.,y>,z<;


//Calculate great diff once and offset positions by it later on

namespace dstruct {
    OctTree::OctTree(Vector3 position_t) {
        position=position_t;
    };
    OctTree::OctTree(Vector3 position_t,std::shared_ptr<OctTree> parent_t) {
        position=position_t;
        parent=parent_t;
    };
    OctTree::OctTree(Vector3 position_t,std::shared_ptr<OctTree> parent_t,char id_t) {
        position=position_t;
        parent=parent_t;
        id=id_t;
    };


    //
    void OctTree::generateOctree(float diff_x, float diff_y, float diff_z) {
        if(&children) {
            children.clear();
        }
        if(bodies_.size()<=MAX_BODIES) {
            return;
        }
        //offsetting the 3D chunk by the *diff*
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        children.push_back(std::make_shared<OctTree>(OctTree(Vector3{position.x+diff_x/2,position.y+diff_y/2,position.z+diff_z/2})));
        children.push_back(std::make_shared<OctTree>(OctTree(Vector3{position.x+diff_x/2,position.y+diff_y/2,position.z-diff_z/2})));
        children.push_back(std::make_shared<OctTree>(OctTree(Vector3{position.x+diff_x/2,position.y-diff_y/2,position.z+diff_z/2})));
        children.push_back(std::make_shared<OctTree>(OctTree(Vector3{position.x+diff_x/2,position.y-diff_y/2,position.z-diff_z/2})));
        children.push_back(std::make_shared<OctTree>(OctTree(Vector3{position.x-diff_x/2,position.y+diff_y/2,position.z+diff_z/2})));
        children.push_back(std::make_shared<OctTree>(OctTree(Vector3{position.x-diff_x/2,position.y+diff_y/2,position.z-diff_z/2})));
        children.push_back(std::make_shared<OctTree>(OctTree(Vector3{position.x-diff_x/2,position.y-diff_y/2,position.z+diff_z/2})));
        children.push_back(std::make_shared<OctTree>(OctTree(Vector3{position.x-diff_x/2,position.y-diff_y/2,position.z-diff_z/2})));
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
            child->generateOctree(diff_x/2,diff_y/2,diff_z/2);
        }
    }

    void OctTree::GenerateDasOctree(float diff_x, float diff_y, float diff_z,std::shared_ptr<OctTree> oct_tree) {
        std::shared_ptr<OctTree> last_oct_tree= oct_tree;
        std::vector<std::shared_ptr<OctTree>> final_children;
        std::vector<std::shared_ptr<OctTree>> oct_trees;


        char current_iterator=0;
        while (true) {
            if(last_oct_tree->bodies_.size()> OctTree::MAX_BODIES) {
                char x=0;
                //Create children
                last_oct_tree->children.push_back(std::make_shared<OctTree>(OctTree(Vector3{last_oct_tree->position.x+diff_x/2,last_oct_tree->position.y+diff_y/2,last_oct_tree->position.z+diff_z/2},last_oct_tree,++x)));
                last_oct_tree->children.push_back(std::make_shared<OctTree>(OctTree(Vector3{last_oct_tree->position.x+diff_x/2,last_oct_tree->position.y+diff_y/2,last_oct_tree->position.z-diff_z/2},last_oct_tree,++x)));
                last_oct_tree->children.push_back(std::make_shared<OctTree>(OctTree(Vector3{last_oct_tree->position.x+diff_x/2,last_oct_tree->position.y-diff_y/2,last_oct_tree->position.z+diff_z/2},last_oct_tree,++x)));
                last_oct_tree->children.push_back(std::make_shared<OctTree>(OctTree(Vector3{last_oct_tree->position.x+diff_x/2,last_oct_tree->position.y-diff_y/2,last_oct_tree->position.z-diff_z/2},last_oct_tree,++x)));
                last_oct_tree->children.push_back(std::make_shared<OctTree>(OctTree(Vector3{last_oct_tree->position.x-diff_x/2,last_oct_tree->position.y+diff_y/2,last_oct_tree->position.z+diff_z/2},last_oct_tree,++x)));
                last_oct_tree->children.push_back(std::make_shared<OctTree>(OctTree(Vector3{last_oct_tree->position.x-diff_x/2,last_oct_tree->position.y+diff_y/2,last_oct_tree->position.z-diff_z/2},last_oct_tree,++x)));
                last_oct_tree->children.push_back(std::make_shared<OctTree>(OctTree(Vector3{last_oct_tree->position.x-diff_x/2,last_oct_tree->position.y-diff_y/2,last_oct_tree->position.z+diff_z/2},last_oct_tree,++x)));
                last_oct_tree->children.push_back(std::make_shared<OctTree>(OctTree(Vector3{last_oct_tree->position.x-diff_x/2,last_oct_tree->position.y-diff_y/2,last_oct_tree->position.z-diff_z/2},last_oct_tree,++x)));
                //Populate the children
                for (auto body : last_oct_tree->bodies_) {
                    /*if(children[0]) {
                        std::cout<<"Is not null"<<std::endl;
                    }*/
                    if(body->position.x>last_oct_tree->position.x) {
                        if(body->position.y>last_oct_tree->position.y) {
                            if(body->position.z>last_oct_tree->position.z) {
                                last_oct_tree->children[0]->bodies_.push_back(body);
                            }else{
                                last_oct_tree->children[1]->bodies_.push_back(body);
                            }
                        }else{
                            if(body->position.z>last_oct_tree->position.z) {
                                last_oct_tree->children[2]->bodies_.push_back(body);
                            }else{
                                last_oct_tree->children[3]->bodies_.push_back(body);
                            }
                        }
                    }
                    else {
                        if(body->position.y>last_oct_tree->position.y) {
                            if(body->position.z>last_oct_tree->position.z) {
                                last_oct_tree->children[4]->bodies_.push_back(body);
                            }else{
                                last_oct_tree->children[5]->bodies_.push_back(body);
                            }
                        }else{
                            if(body->position.z>last_oct_tree->position.z) {
                                last_oct_tree->children[6]->bodies_.push_back(body);
                            }else{
                                last_oct_tree->children[7]->bodies_.push_back(body);
                            }
                        }
                    }
                }
            }else {
                current_iterator=last_oct_tree->id;
                last_oct_tree=last_oct_tree->parent;
                //Return to the parent
            }
            if(current_iterator==7) {
                if(!last_oct_tree->parent) {
                    return;
                }
                current_iterator=0;
                last_oct_tree=last_oct_tree->parent;
            }
            current_iterator++;
        }

    }
}
