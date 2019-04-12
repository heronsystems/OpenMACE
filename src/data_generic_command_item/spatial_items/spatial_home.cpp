#include "spatial_home.h"

namespace CommandItem {

COMMANDITEM SpatialHome::getCommandType() const
{
    return COMMANDITEM::CI_NAV_HOME;
}

std::string SpatialHome::getDescription() const
{
    return "This stores the home location for a vehicle";
}

bool SpatialHome::hasSpatialInfluence() const
{
    return true;
}

std::shared_ptr<AbstractCommandItem> SpatialHome::getClone() const
{
    return std::make_shared<SpatialHome>(*this);
}

void SpatialHome::getClone(std::shared_ptr<AbstractCommandItem> &command) const
{
    command = std::make_shared<SpatialHome>(*this);
}

SpatialHome::SpatialHome():
    AbstractSpatialAction(0,0)
{

}

SpatialHome::SpatialHome(const pose::GeodeticPosition_3D &position):
    AbstractSpatialAction(0,0)
{
    this->position->setX(position.getLatitude());
    this->position->setY(position.getLongitude());
    this->position->setZ(position.getAltitude());
}

SpatialHome::~SpatialHome()
{

}

SpatialHome::SpatialHome(const SpatialHome &obj):
    AbstractSpatialAction(obj)
{

}

SpatialHome::SpatialHome(const int &systemOrigin, const int &systemTarget):
    AbstractSpatialAction(systemOrigin,systemTarget)
{

}

mace_home_position_t SpatialHome::getMACECommsObject() const
{
    mace_home_position_t homePosition;
    homePosition.latitude = position->getX() * pow(10,7);
    homePosition.longitude = position->getY() * pow(10,7);
    homePosition.altitude = position->getZ() * pow(10,3);
    return homePosition;
}

mace_message_t SpatialHome::getMACEMsg(const uint8_t systemID, const uint8_t compID, const uint8_t chan) const
{
    mace_message_t msg;
    mace_home_position_t homePosition = getMACECommsObject();
    mace_msg_home_position_encode_chan(systemID,compID,chan,&msg,&homePosition);
    return msg;
}

std::ostream& operator<<(std::ostream& os, const SpatialHome& t)
{
    std::stringstream stream;
    stream.precision(6);
    stream << std::fixed << "Spatial Home: "<< t.position->getX() << ", "<< t.position->getY() << ", "<< t.position->getZ() << ".";
    os << stream.str();

    return os;
}

} //end of namespace CommandItem
