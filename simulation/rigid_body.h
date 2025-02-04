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
        virtual ~RigidBody() = default;

        RigidBody();
        float mass{};
        Vector3 position{};
        Vector3 force{};
        Vector3 velocity{};
    };
}
#endif //RIGID_BODY_H
