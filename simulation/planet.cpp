//
// Created by xenu on 21.11.24.
//
#include "planet.h"
#include  <nlohmann/json.hpp>

Planet::Planet() {
}

Planet::Planet(Vector3 initial_position, unsigned long long mass, Vector3 velocity, float bounciness, Color color, std::string name, uint id){
    this->position = initial_position;
    this->mass = mass;
    this->velocity = velocity;
    this->color = color;
    this->name = name;
    this->id = id;
}