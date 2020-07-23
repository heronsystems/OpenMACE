#include "abstract_geodetic_position.h"

using namespace mace::pose;

Abstract_GeodeticPosition::Abstract_GeodeticPosition(const GeodeticFrameTypes &explicitFrame, const std::string &posName):
    Position (posName), PositionInterface(), geodeticFrameType(explicitFrame)
{

}

Abstract_GeodeticPosition::Abstract_GeodeticPosition(const Abstract_GeodeticPosition &copy):
    Position (copy), PositionInterface()
{
    this->geodeticFrameType = copy.geodeticFrameType;
}

bool Abstract_GeodeticPosition::areEquivalentGeodeticFrames(const Abstract_GeodeticPosition &obj) const
{
    return this->geodeticFrameType == obj.geodeticFrameType;
}

CoordinateSystemTypes Abstract_GeodeticPosition::getCoordinateSystemType() const
{
    return CoordinateSystemTypes::GEODETIC;
}

void Abstract_GeodeticPosition::setCoordinateFrame(const GeodeticFrameTypes &explicitFrame)
{
    this->geodeticFrameType = explicitFrame;
}

mace::GeodeticFrameTypes Abstract_GeodeticPosition::getGeodeticCoordinateFrame() const
{
    return this->geodeticFrameType;
}

mace::CoordinateFrameTypes Abstract_GeodeticPosition::getExplicitCoordinateFrame() const
{
    return getCoordinateFrame(geodeticFrameType);
}

QJsonObject Abstract_GeodeticPosition::toJSON(const int &vehicleID, const std::string &dataType) const
{
    QJsonObject json = toJSON_base(vehicleID,dataType);
    QJsonObject location;
    updateQJSONObject(location);
    json["location"] = location;
    return json;
}
