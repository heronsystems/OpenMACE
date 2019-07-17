#include "controller_guided_mission_item.h"

namespace MAVLINKVehicleControllers {

    template <>
    void ControllerGuidedMissionItem<CommandItem::SpatialWaypoint>::FillMissionItem(const CommandItem::SpatialWaypoint &commandItem, mavlink_mission_item_t &mavlinkItem)
    {
        mavlinkItem.command = MAV_CMD_NAV_WAYPOINT;
        Base3DPosition pos = commandItem.getPosition();

        if(pos.getCoordinateFrame() == Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT){
            mavlinkItem.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
        }
        else if(pos.getCoordinateFrame() == Data::CoordinateFrameType::CF_LOCAL_ENU)
        {
            mavlinkItem.frame = MAV_FRAME_LOCAL_ENU;
        }
        else{
            //KEN FIX THIS
            mavlinkItem.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
        }

        mavlinkItem.x = static_cast<float>(pos.getY());
        mavlinkItem.y = static_cast<float>(pos.getX());
        mavlinkItem.z = static_cast<float>(pos.getZ());
    }

}// end of namespace MAVLINKVehicleControllers
