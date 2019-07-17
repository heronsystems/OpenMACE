#include "abstract_cartesian_position.h"

using namespace mace::pose;

Abstract_CartesianPosition::Abstract_CartesianPosition(const CartesianFrameTypes &explicitFrame, const std::string &posName):
    Position (posName), cartesianFrameType(explicitFrame)
{

}

Abstract_CartesianPosition::Abstract_CartesianPosition(const CartesianFrameTypes &explicitFrame, const double &x, const double &y, const std::string &posName):
    Position (posName), PositionInterface(x,y), cartesianFrameType(explicitFrame)
{

}


Abstract_CartesianPosition::Abstract_CartesianPosition(const Abstract_CartesianPosition &copy):
    Position (copy), PositionInterface(copy)
{
    this->cartesianFrameType = copy.cartesianFrameType;
}

PositionType Abstract_CartesianPosition::getPositionType() const
{
    return PositionType::CARTESIAN;
}

void Abstract_CartesianPosition::setCoordinateFrame(const CartesianFrameTypes &explicitFrame)
{
    this->cartesianFrameType = explicitFrame;
}

CartesianFrameTypes Abstract_CartesianPosition::getCartesianFrameType() const
{
    return this->cartesianFrameType;
}

CoordinateFrameTypes Abstract_CartesianPosition::getExplicitCoordinateFrame() const
{
    return getCoordinateFrame(cartesianFrameType);
}

bool Abstract_CartesianPosition::areEquivalentCartesianFrames(const Abstract_CartesianPosition &obj) const
{
    return this->cartesianFrameType == obj.getCartesianFrameType();
}


