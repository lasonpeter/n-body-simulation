//
// Created by xenu on 21.11.24.
//

#ifndef RIGID_BODY_H
#define RIGID_BODY_H
#include <raylib.h>
#include <sys/types.h>

namespace sim {
    class RigidBody {
    public:
        virtual ~RigidBody()= default;
        RigidBody();

        void calculateStep(RigidBody *rigid_body2, float delta_t);

        ulong mass; //It's a ulong instead of uint due to memory alignment which makes class with int mass have the same size in memory as one with ulong
        Vector3 position{};
        Vector3 velocity{};
        Vector3 calculateGravityForce(RigidBody* rigid_body2);
        void updateState(Vector3* force,float delta_t);
    private:
        //Vector3 CalculateExplosionForce();
        Vector3 calculateCollisionForce(RigidBody* other);
    };
}
#endif //RIGID_BODY_H
