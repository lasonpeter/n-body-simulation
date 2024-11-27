//
// Created by xenu on 24.11.24.
//

#ifndef OCT_TREE_H
#define OCT_TREE_H
#include <raylib.h>
#include <vector>
#include "../simulation/rigid_body.h"

namespace dstruct {
    class OctTree {
        public:
            OctTree(Vector3 position_t);

            OctTree(Vector3 position_t, std::shared_ptr<OctTree> parent_t);

            OctTree(Vector3 position_t, std::shared_ptr<OctTree> parent_t, char id_t);
            Vector3 position{};
            // maybe could optimize this by storing an array of pointers to the bodies only on the leafs and not the branches
            std::vector< std::shared_ptr<sim::RigidBody>> bodies_{};
            std::vector<std::shared_ptr<OctTree>> children{};
            char id;

            /**
             * Generates an octree structure from available bodies
             * @param diff_x
             * @param diff_y
             * @param diff_z
             */
            void generateOctree(float diff_x, float diff_y, float diff_z);
            static constexpr int MAX_BODIES=12;
            std::shared_ptr<OctTree> parent;
        void iterativeGenerateOctree(float diff_x, float diff_y, float diff_z);
        void static GenerateDasOctree(float diff_x, float diff_y, float diff_z, std::shared_ptr<OctTree> oct_tree);
    };
}



#endif //OCT_TREE_H
