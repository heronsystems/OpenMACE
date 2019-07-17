#include "abstract_geodetic_position.h"

using namespace mace::pose;

Abstract_GeodeticPosition::Abstract_GeodeticPosition(const GeodeticFrameTypes &explicitFrame, const std::string &posName):
    Position (posName), geodeticFrameType(explicitFrame)
{

}

Abstract_GeodeticPosition::Abstract_GeodeticPosition(const GeodeticFrameTypes &explicitFrame, const double &latitude, const double &longitude, const std::string &posName):
    Position (posName), PositionInterface(longitude,latitude), geodeticFrameType(explicitFrame)
{

}


Abstract_GeodeticPosition::Abstract_GeodeticPosition(const Abstract_GeodeticPosition &copy):
    Position (copy), PositionInterface(copy)
{
    this->geodeticFrameType = copy.geodeticFrameType;
}

bool Abstract_GeodeticPosition::areEquivalentGeodeticFrames(const Abstract_GeodeticPosition &obj) const
{
    return this->geodeticFrameType == obj.geodeticFrameType;
}

PositionType Abstract_GeodeticPosition::getPositionType() const
{
    return PositionType::GEODETIC;
}

void Abstract_GeodeticPosition::setCoordinateFrame(const GeodeticFrameTypes &explicitFrame)
{
    this->geodeticFrameType = explicitFrame;
}

GeodeticFrameTypes Abstract_GeodeticPosition::getGeodeticCoordinateFrame() const
{
    return this->geodeticFrameType;
}

CoordinateFrameTypes Abstract_GeodeticPosition::getExplicitCoordinateFrame() const
{
    return getCoordinateFrame(geodeticFrameType);
}

