#include "abstract_cartesian_position.h"

using namespace mace::pose;

Abstract_CartesianPosition::Abstract_CartesianPosition(const CartesianFrameTypes &explicitFrame, const std::string &posName):
    Position (posName), cartesianFrameType(explicitFrame)
{

}


Abstract_CartesianPosition::Abstract_CartesianPosition(const Abstract_CartesianPosition &copy):
    Position (copy)
{
    this->cartesianFrameType = copy.cartesianFrameType;
}

CoordinateSystemTypes Abstract_CartesianPosition::getCoordinateSystemType() const
{
    return CoordinateSystemTypes::CARTESIAN;
}

void Abstract_CartesianPosition::setCoordinateFrame(const CartesianFrameTypes &explicitFrame)
{
    this->cartesianFrameType = explicitFrame;
}

mace::CartesianFrameTypes Abstract_CartesianPosition::getCartesianFrameType() const
{
    return this->cartesianFrameType;
}

mace::CoordinateFrameTypes Abstract_CartesianPosition::getExplicitCoordinateFrame() const
{
    return getCoordinateFrame(cartesianFrameType);
}

bool Abstract_CartesianPosition::areEquivalentCartesianFrames(const Abstract_CartesianPosition &obj) const
{
    return this->cartesianFrameType == obj.getCartesianFrameType();
}


