#include "abstract_geodetic_position.h"

using namespace mace::pose;

Abstract_GeodeticPosition::Abstract_GeodeticPosition(const GeodeticFrameTypes &explicitFrame, const std::string &posName):
    Position (posName), geodeticFrameType(explicitFrame)
{

}

Abstract_GeodeticPosition::Abstract_GeodeticPosition(const GeodeticFrameTypes &explicitFrame, const double &latitude, const double &longitude, const std::string &posName):
    Position (posName), geodeticFrameType(explicitFrame), PositionInterface(longitude,latitude)
{

}


Abstract_GeodeticPosition::Abstract_GeodeticPosition(const Abstract_GeodeticPosition &copy):
    Position (copy), PositionInterface(copy)
{
    this->geodeticFrameType = copy.geodeticFrameType;
}

PositionType Abstract_GeodeticPosition::getPositionType() const
{
    return PositionType::GEODETIC;
}

CoordinateFrameTypes Abstract_GeodeticPosition::getExplicitCoordinateFrame() const
{
    return getCoordinateFrame(geodeticFrameType);
}

