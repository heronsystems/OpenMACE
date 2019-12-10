#include "kinematic_definitions.h"

Kinematic_BaseInterface::Kinematic_BaseInterface()
{

}

Kinematic_BaseInterface::Kinematic_BaseInterface(const Kinematic_BaseInterface &copy)
{
    this->dimension = copy.dimension;
    this->dimensionMask = copy.dimensionMask;
}

Kinematic_BaseInterface::~Kinematic_BaseInterface()
{

}

