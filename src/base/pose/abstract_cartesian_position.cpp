#include "abstract_cartesian_position.h"

using namespace mace::pose;

Abstract_CartesianPosition::Abstract_CartesianPosition(const CartesianFrameTypes &explicitFrame, const std::string &posName):
    Position (posName), cartesianFrameType(explicitFrame)
{

}

Abstract_CartesianPosition::Abstract_CartesianPosition(const CartesianFrameTypes &explicitFrame, const double &x, const double &y, const std::string &posName):
    Position (posName), cartesianFrameType(explicitFrame), PositionInterface(x,y)
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

CoordinateFrameTypes Abstract_CartesianPosition::getExplicitCoordinateFrame() const
{
    return getCoordinateFrame(cartesianFrameType);
}

