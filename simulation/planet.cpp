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
    //this->acceleration = acceleration;
    this->color = color;
    this->name = name;
    this->id = id;
}



/*
void Planet::from_json(const nlohmann::json& j, Planet& planet)
{
    j.at("id").get_to(planet.id);
    j.at("name").get_to(planet.name);
    j.at("position").get_to(planet.velocity);
    j.at("velocity").get_to(planet.velocity);
    j.at("mass").get_to(planet.mass);
    j.at("color").get_to(planet.color);
    j.at("bounciness").get_to(planet.bounciness);
}

void Planet::to_json(nlohmann::json& j, const Planet& planet)
{
    j = nlohmann::json{
        {"id", planet.id},
        {"name", planet.name},
        {"position", planet.position},
        {"velocity", planet.velocity},
        {"mass", planet.mass},
        {"color", planet.color},
        {"bounciness", planet.bounciness}
    };
    */


