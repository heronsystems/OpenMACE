#include "abstract_velocity.h"

using namespace mace::pose;

Abstract_Velocity::Abstract_Velocity(const std::string &posName):
    name(posName)
{

}

Abstract_Velocity::Abstract_Velocity(const Abstract_Velocity &copy)
{
    this->name = copy.name;
    this->dimension = copy.dimension;
}

void Abstract_Velocity::updateVelocityName(const std::string &nameString)
{
    this->name = nameString;
}

std::string Abstract_Velocity::getName() const
{
    return this->name;
}
