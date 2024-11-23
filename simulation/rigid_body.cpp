//
// Created by xenu on 21.11.24.
//

#include "rigid_body.h"

#include "constants.h"
#include "raymath.h"
#include <nlohmann/json.hpp>

namespace sim {


    RigidBody::RigidBody() {
    }

    Vector3 RigidBody::calculateGravityForce(RigidBody* rigid_body2) {
        Vector3 force = {0, 0, 0};
        float distance = Vector3Distance(this->position, rigid_body2->position);
        float force_magnitude =static_cast<float>( this->mass * rigid_body2->mass* cnst::GRAVITY_CONSTANT) / (distance * distance);
        force = Vector3Subtract(rigid_body2->position, this->position);
        force = Vector3Normalize(force);
        force = Vector3Scale(force, force_magnitude);
        return force;
    }

    void RigidBody::updateState(Vector3 *force,float delta_t) {
        this->velocity = Vector3Add(velocity,Vector3Scale(*force, 1.0f / this->mass)*delta_t);
        this->position = Vector3Add(this->position, Vector3Scale(this->velocity, delta_t));
    }

    Vector3 RigidBody::calculateCollisionForce(RigidBody* other) {

    }

}
