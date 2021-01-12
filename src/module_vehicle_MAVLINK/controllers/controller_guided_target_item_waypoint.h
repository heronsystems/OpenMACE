#ifndef CONTROLLER_GUIDED_TARGET_ITEM_WAYPOINT_H
#define CONTROLLER_GUIDED_TARGET_ITEM_WAYPOINT_H

#include <mavlink.h>

#include "common/common.h"

#include "controllers/actions/action_broadcast.h"

#include "module_vehicle_MAVLINK/mavlink_coordinate_frames.h"
#include "module_vehicle_MAVLINK/mavlink_entity_key.h"
#include "module_vehicle_MAVLINK/controllers/common.h"

#include "data_generic_command_item/spatial_items/spatial_waypoint.h"


namespace MAVLINKUXVControllers {

using namespace mace::pose;

using GuidedTGTWPBroadcast = Controllers::ActionBroadcast<
    mavlink_message_t,
    MavlinkEntityKey,
    BasicMavlinkController_ModuleKeyed<command_item::SpatialWaypoint>,
    command_item::SpatialWaypoint,
    mavlink_mission_item_t
>;

class ControllerGuidedTargetItem_WP : public BasicMavlinkController_ModuleKeyed<command_item::SpatialWaypoint>,
        public GuidedTGTWPBroadcast
{
private:

    std::unordered_map<MaceCore::ModuleCharacteristic, MaceCore::ModuleCharacteristic> m_CommandRequestedFrom;

protected:

    void Construct_Broadcast(const command_item::SpatialWaypoint &commandItem, const MavlinkEntityKey &sender, mavlink_mission_item_t &targetItem) override
    {
        UNUSED(sender);

        targetItem = initializeMAVLINKMissionItem();
        targetItem.target_system = commandItem.getTargetSystem();
        targetItem.target_component = static_cast<uint8_t>(MaceCore::ModuleClasses::VEHICLE_COMMS);

        targetItem.command = MAV_CMD_NAV_WAYPOINT;
        const mace::pose::Position* basePosition = commandItem.getPosition();

        if(basePosition->getCoordinateSystemType() == CoordinateSystemTypes::GEODETIC){
            targetItem.frame = getMAVLINKCoordinateFrame(basePosition->getExplicitCoordinateFrame());
            if(basePosition->is2D())
            {
                const mace::pose::GeodeticPosition_2D* castPosition = basePosition->positionAs<mace::pose::GeodeticPosition_2D>();
                targetItem.x = static_cast<float>(castPosition->getLatitude());
                targetItem.y = static_cast<float>(castPosition->getLongitude());
                targetItem.z = static_cast<float>(0.0);
            }
            else if(basePosition->is3D())
            {
                const mace::pose::GeodeticPosition_3D* castPosition = basePosition->positionAs<mace::pose::GeodeticPosition_3D>();
                targetItem.x = static_cast<float>(castPosition->getLatitude());
                targetItem.y = static_cast<float>(castPosition->getLongitude());
                targetItem.z = static_cast<float>(castPosition->getAltitude());
            }
        }
        else if(basePosition->getCoordinateSystemType() == CoordinateSystemTypes::CARTESIAN)
        {
            if(basePosition->is2D())
            {
                const mace::pose::CartesianPosition_2D* castPosition = basePosition->positionAs<mace::pose::CartesianPosition_2D>();
                targetItem.x = static_cast<float>(castPosition->getXPosition());
                targetItem.y = static_cast<float>(castPosition->getYPosition());
                targetItem.z = static_cast<float>(0.0);
            }
            else if(basePosition->is3D())
            {
                const mace::pose::CartesianPosition_3D* castPosition = basePosition->positionAs<mace::pose::CartesianPosition_3D>();
                targetItem.x = static_cast<float>(castPosition->getXPosition());
                targetItem.y = static_cast<float>(castPosition->getYPosition());
                targetItem.z = static_cast<float>(castPosition->getAltitude());
            }
        }
    }

protected:
    mavlink_mission_item_t initializeMAVLINKMissionItem()
    {
        mavlink_mission_item_t missionItem;
        missionItem.autocontinue = 1;
        missionItem.command = MAV_CMD_NAV_WAYPOINT;
        missionItem.current = 2;
        missionItem.frame = MAV_FRAME_GLOBAL;
        missionItem.param1 = 0.0;
        missionItem.param2 = 0.0;
        missionItem.param3 = 0.0;
        missionItem.param4 = 0.0;
        missionItem.seq = 0;
        missionItem.target_system = 0;
        missionItem.target_component = 0;
        missionItem.x = 0.0;
        missionItem.y = 0.0;
        missionItem.z = 0.0;
        missionItem.mission_type = 0;

        return missionItem;
    }

public:
    ControllerGuidedTargetItem_WP(const Controllers::IMessageNotifier<mavlink_message_t, MavlinkEntityKey> *cb, TransmitQueue *queue, int linkChan) :
        BasicMavlinkController_ModuleKeyed<command_item::SpatialWaypoint>(cb, queue, linkChan),
        GuidedTGTWPBroadcast(this, MavlinkEntityKeyToSysIDCompIDConverter<mavlink_mission_item_t>(mavlink_msg_mission_item_encode_chan))
    {

    }

private:


};

} //end of namespace MAVLINKVehicleControllers


#endif // CONTROLLER_GUIDED_TARGET_ITEM_WAYPOINT_H
