#include "spatial_home.h"

namespace command_item {

COMMANDTYPE SpatialHome::getCommandType() const
{
    return COMMANDTYPE::CI_NAV_HOME;
}

std::string SpatialHome::getDescription() const
{
    return "This stores the home location for a vehicle";
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

SpatialHome::SpatialHome(const mace::pose::Position* homePosition):
    AbstractSpatialAction(0,0)
{
    this->setPosition(homePosition);
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

/** Interface imposed via Interface_CommandItem<mace_command_long_t> */
void SpatialHome::toMACEComms_CommandItem(mace_command_long_t &obj) const
{
    Interface_CommandItem::initializeCommandItem(obj);
    populateCommandItem_FromPosition(obj);
}

/** End of interface imposed via Interface_CommandItem<mace_command_long_t> */

/** Interface imposed via AbstractCommandItem */
bool SpatialHome::generateMACECOMMS_MissionItemMSG(mace_mission_item_t &msg) const
{
    //Interface_MissionItem::initializeMissionItem(msg);
    //temp command object since we know that we can map the two together
    mace_command_long_t cmdLong;
    toMACEComms_CommandItem(cmdLong);
    Interface_CommandItem::populateMissionitem(cmdLong,msg);

}

bool SpatialHome::fromMACECOMMS_MissionItemMSG(const mace_mission_item_t &msg) const
{

}

bool SpatialHome::generateMACEMSG_MissionItem(mace_message_t &msg) const
{

}

bool SpatialHome::generateMACEMSG_CommandItem(mace_message_t &msg) const
{

}
/** End of interface imposed via Interface_CommandItem<mace_command_short_t> */



//bool SpatialHome::getMACECommsObject(mace_home_position_t &obj) const
//{
//    bool validRequest = true;
//    if(position->getCoordinateSystemType() == CoordinateSystemTypes::GEODETIC)
//    {
//        if(position->is3D())
//        {
//            mace::pose::GeodeticPosition_3D* currentPosition = position->positionAs<mace::pose::GeodeticPosition_3D>();
//            obj.latitude = static_cast<int32_t>(currentPosition->getLatitude() * pow(10,7));
//            obj.longitude = static_cast<int32_t>(currentPosition->getLongitude() * pow(10,7));
//            obj.altitude = static_cast<int32_t>(currentPosition->getAltitude() * pow(10,3));
//        }
//        else if(position->is2D()) {
//            mace::pose::GeodeticPosition_2D* currentPosition = position->positionAs<mace::pose::GeodeticPosition_2D>();
//            obj.latitude = static_cast<int32_t>(currentPosition->getLatitude() * pow(10,7));
//            obj.longitude = static_cast<int32_t>(currentPosition->getLongitude() * pow(10,7));
//        }
//    }
//    else if(position->getCoordinateSystemType() == CoordinateSystemTypes::CARTESIAN)
//    {
//        if(position->is3D())
//        {
//            mace::pose::CartesianPosition_3D* currentPosition = position->positionAs<mace::pose::CartesianPosition_3D>();
//            obj.latitude = static_cast<int32_t>(currentPosition->getXPosition() * pow(10,7));
//            obj.longitude = static_cast<int32_t>(currentPosition->getYPosition() * pow(10,7));
//            obj.altitude = static_cast<int32_t>(currentPosition->getAltitude() * pow(10,3));
//        }
//        else if(position->is2D()) {
//            mace::pose::CartesianPosition_2D* currentPosition = position->positionAs<mace::pose::CartesianPosition_2D>();
//            obj.latitude = static_cast<int32_t>(currentPosition->getXPosition() * pow(10,7));
//            obj.longitude = static_cast<int32_t>(currentPosition->getYPosition() * pow(10,7));
//        }
//    }
//    else {
//        validRequest = false;
//    }

//    return validRequest;
//}

//bool SpatialHome::getMACEMsg(const uint8_t systemID, const uint8_t compID, const uint8_t chan, mace_message_t &msg) const
//{
//    mace_home_position_t homeMSG;
//    bool validRequest = getMACECommsObject(homeMSG);
//    if(validRequest)
//        mace_msg_home_position_encode_chan(systemID,compID,chan,&msg,&homeMSG);
//    return validRequest;
//}

//!
//! \brief printPositionalInfo
//! \return
//!
std::string SpatialHome::printSpatialCMDInfo() const
{
    std::stringstream ss;
    if(isPositionSet())
        this->position->printPositionLog(ss);
    return ss.str();
}

std::ostream& operator<<(std::ostream& os, const SpatialHome& t)
{

    std::stringstream stream;
    stream.precision(6);
    //stream << std::fixed << "Spatial Home: "<< t.position->getX() << ", "<< t.position->getY() << ", "<< t.position->getZ() << ".";
    os << stream.str();

    return os;
}

} //end of namespace CommandItem
