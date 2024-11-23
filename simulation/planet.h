//
// Created by xenu on 21.11.24.
//

#ifndef PLANET_H
#define PLANET_H
#include <raylib.h>
#include <string>
#include <glaze/json.hpp>

#include "rigid_body.h"
//#include "rigid_body.h"




class Planet: public sim::RigidBody {
public:
    uint id;
    Color color{};
    std::string name;
    //generate a constructor
    Planet();
    Planet(Vector3, long long unsigned int, Vector3, float, Color, std::string, uint);
    struct glaze {
        using T = Planet;
        static constexpr auto value = glz::object(
            "id", &Planet::id,
            "color", &Planet::color,
            "name", &Planet::name,
            "position", &Planet::position,
            "velocity", &Planet::velocity,
            "mass", &Planet::mass,
            "bounciness", &Planet::bounciness
        );
    };
};
#endif //PLANET_H
