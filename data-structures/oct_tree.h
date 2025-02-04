//
// Created by xenu on 24.11.24.
//

#ifndef OCT_TREE_H
#define OCT_TREE_H
#include <raylib.h>
#include <vector>
#include "../simulation/rigid_body.h"



///////
//////
///
///
///TODO Calculate center of mass



namespace dstruct {
    class OctTree {
        public:
        ~OctTree() {
                for (auto child : children) {
                    delete child;
                }
                children.clear();
            }
            Vector3 position{};
            std::vector<sim::RigidBody*> bodies_ ={};
            std::vector<OctTree*> children{};
            Vector3 center_of_mass{};
            float size{};
            float mass{};
            int body_count{};
            OctTree* parent =nullptr;

            OctTree(Vector3 position_t, float size_t);

            OctTree(Vector3 position_t, float size_t, OctTree* parent_t);

            /**
             * Generates an octree structure from available bodies
             * @param diff_x
             * @param diff_y
             * @param diff_z
             */

            /**
             * Updates the center of mass of the octree
             * @param body to add to the center of mass
             */
            void updateCenterOfMass(sim::RigidBody *body);

            /**
             * Adds a body to the octree
             * @param body body to add
             */
            void add_body(sim::RigidBody *body);

            /**
             * Returns the octant the body belongs to
             * @param body body to check
             * @return octant the body belongs to
             */
            int GetOctant(sim::RigidBody *body);
            //void calculateStep(const std::shared_ptr<OctTree> &oct_tree, float delta_t);

            static constexpr int MAX_BODIES=1;

    };
}



#endif //OCT_TREE_H
