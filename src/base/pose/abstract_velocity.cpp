#include "abstract_velocity.h"

namespace mace {
namespace pose {

const uint16_t Velocity::ignoreAllVelocities;


Velocity::Velocity(const std::string &posName):
    Kinematic_BaseInterface(), name(posName)
{
    this->dimensionMask = ignoreAllVelocities;
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

} //end of namespace pose
} //end of namespace mace
