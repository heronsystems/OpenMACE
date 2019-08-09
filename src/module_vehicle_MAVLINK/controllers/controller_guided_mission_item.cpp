#include "controller_guided_mission_item.h"

namespace MAVLINKVehicleControllers {

template <>
void ControllerGuidedMissionItem<command_item::SpatialWaypoint>::FillMissionItem(const command_item::SpatialWaypoint &commandItem, mavlink_mission_item_t &mavlinkItem)
{
    mavlinkItem.command = MAV_CMD_NAV_WAYPOINT;
    const mace::pose::Position* basePosition = commandItem.getPosition();

    if(basePosition->getCoordinateSystemType() == CoordinateSystemTypes::GEODETIC){
        mavlinkItem.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
        if(basePosition->is2D())
        {
            const mace::pose::GeodeticPosition_2D* castPosition = basePosition->positionAs<mace::pose::GeodeticPosition_2D>();
            mavlinkItem.x = static_cast<float>(castPosition->getLatitude());
            mavlinkItem.y = static_cast<float>(castPosition->getLongitude());
            mavlinkItem.z = static_cast<float>(0.0);
        }
        else if(basePosition->is3D())
        {
            const mace::pose::GeodeticPosition_3D* castPosition = basePosition->positionAs<mace::pose::GeodeticPosition_3D>();
            mavlinkItem.x = static_cast<float>(castPosition->getLatitude());
            mavlinkItem.y = static_cast<float>(castPosition->getLongitude());
            mavlinkItem.z = static_cast<float>(castPosition->getAltitude());
        }
    }
    else if(basePosition->getCoordinateSystemType() == CoordinateSystemTypes::CARTESIAN)
    {
        if(basePosition->is2D())
        {
            const mace::pose::CartesianPosition_2D* castPosition = basePosition->positionAs<mace::pose::CartesianPosition_2D>();
            mavlinkItem.x = static_cast<float>(castPosition->getXPosition());
            mavlinkItem.y = static_cast<float>(castPosition->getYPosition());
            mavlinkItem.z = static_cast<float>(0.0);
        }
        else if(basePosition->is3D())
        {
            const mace::pose::CartesianPosition_3D* castPosition = basePosition->positionAs<mace::pose::CartesianPosition_3D>();
            mavlinkItem.x = static_cast<float>(castPosition->getXPosition());
            mavlinkItem.y = static_cast<float>(castPosition->getYPosition());
            mavlinkItem.z = static_cast<float>(castPosition->getAltitude());
        }
    }
}

}// end of namespace MAVLINKVehicleControllers
