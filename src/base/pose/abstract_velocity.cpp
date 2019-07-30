#include "abstract_velocity.h"

using namespace mace::pose;

Velocity::Velocity(const std::string &posName):
    name(posName)
{

}

Velocity::Velocity(const Velocity &copy)
{
    this->name = copy.name;
    this->dimension = copy.dimension;
}

void Velocity::updateVelocityName(const std::string &nameString)
{
    this->name = nameString;
}

std::string Velocity::getName() const
{
    return this->name;
}
