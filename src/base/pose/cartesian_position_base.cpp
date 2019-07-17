#include "abstract_cartesian_position.h"

using namespace mace::pose;

Abstract_CartesianPosition::Abstract_CartesianPosition(const CartesianFrameTypes &explicitFrame, const std::string &posName):
    Position (posName), explicitFrameType(explicitFrame)
{

}

Abstract_CartesianPosition::Abstract_CartesianPosition(const CartesianFrameTypes &explicitFrame, const std::string &posName, const double &x, const double &y):
    Position (posName), explicitFrameType(explicitFrame), PositionInterface()
{

}


Abstract_CartesianPosition::Abstract_CartesianPosition(const Abstract_CartesianPosition &copy):
    Position (copy)
{
    this->explicitFrameType = copy.explicitFrameType;
}

PositionType Abstract_CartesianPosition::getPositionType() const
{
    return PositionType::CARTESIAN;
}

CoordinateFrame Abstract_CartesianPosition::getExplicitCoordinateFrame() const
{
    return getCoordinateFrame(explicitFrameType);
}

