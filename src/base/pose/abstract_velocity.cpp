#include "abstract_velocity.h"

using namespace mace::pose;

Velocity::Velocity(const std::string &posName):
    Kinematic_BaseInterface(), name(posName)
{

}

Velocity::Velocity(const Velocity &copy):
    Kinematic_BaseInterface (copy)
{
    this->name = copy.name;
}

void Velocity::updateVelocityName(const std::string &nameString)
{
    this->name = nameString;
}

std::string Velocity::getName() const
{
    return this->name;
}
