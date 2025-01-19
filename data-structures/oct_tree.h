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
            // maybe could optimize this by storing an array of pointers to the bodies only on the leafs and not the branches
            std::vector<sim::RigidBody*> bodies_ ={};
            std::vector<OctTree*> children{};
            Vector3 center_of_mass{};
            float size{};
            float mass{};
            int body_count{};
            OctTree* parent =nullptr;
        //char id;

            OctTree(Vector3 position_t, float size_t);

            OctTree(Vector3 position_t, float size_t, OctTree* parent_t);


            //OctTree(Vector3 position_t, float size_t, const std::shared_ptr<OctTree> &parent_t);
            /**
             * Generates an octree structure from available bodies
             * @param diff_x
             * @param diff_y
             * @param diff_z
             */


            void updateCenterOfMass(sim::RigidBody *body);

            void add_body(sim::RigidBody *body);

            int GetOctant(sim::RigidBody *body);

            void calculateStep(OctTree *&oct_tree, float delta_t);

            void generateOctree();

            void generateDasOctree();

            //void calculateStep(const std::shared_ptr<OctTree> &oct_tree, float delta_t);

            static Vector3 centerOfMass(Vector3 vec1, ulong mass1, Vector3 vec2, ulong mass2);

            static constexpr int MAX_BODIES=1;
            //std::shared_ptr<OctTree> parent; not now
            void iterativeGenerateOctree(float diff_x, float diff_y, float diff_z);

            //static void GenerateDasOctree(std::shared_ptr<OctTree> oct_tree);

            //void CalculateStep(std::shared_ptr<OctTree> oct_tree, float delta_t);

            //void CalculateForce(std::shared_ptr<OctTree> oct_tree);
    };
}



#endif //OCT_TREE_H
