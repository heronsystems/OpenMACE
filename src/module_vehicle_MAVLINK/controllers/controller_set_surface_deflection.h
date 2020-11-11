#ifndef CONTROLLER_SET_SURFACE_DEFLECTION_H
#define CONTROLLER_SET_SURFACE_DEFLECTION_H

#include <mavlink.h>

#include "common/common.h"

#include "controllers/actions/action_send.h"
#include "controllers/actions/action_finish.h"
#include "controllers/actions/action_broadcast.h"

#include "module_vehicle_MAVLINK/mavlink_entity_key.h"

#include "module_vehicle_MAVLINK/controllers/common.h"

#include "data_generic_command_item/do_items/action_set_surface_deflection.h"

namespace MAVLINKUXVControllers {

using SurfaceDeflectionBroadcast = Controllers::ActionBroadcast<
    mavlink_message_t,
    MavlinkEntityKey,
    BasicMavlinkController_ModuleKeyed<command_item::Action_SetSurfaceDeflection>,
    command_item::Action_SetSurfaceDeflection,
    mavlink_set_surface_deflection_normalized_t
>;

class Controller_SetSurfaceDeflection : public BasicMavlinkController_ModuleKeyed<command_item::Action_SetSurfaceDeflection>,
        public SurfaceDeflectionBroadcast
{
private:

    std::unordered_map<MaceCore::ModuleCharacteristic, MaceCore::ModuleCharacteristic> m_CommandRequestedFrom;

    mavlink_set_surface_deflection_normalized_t m_deflectionMSG;

protected:

    void Construct_Broadcast(const command_item::Action_SetSurfaceDeflection &commandItem, const MavlinkEntityKey &sender, mavlink_set_surface_deflection_normalized_t &deflectionItem) override
    {
        UNUSED(sender);

        deflectionItem = initializeMAVLINKDeflection();
        deflectionItem.target_system = commandItem.getTargetSystem();
        deflectionItem.target_component = static_cast<uint8_t>(MaceCore::ModuleClasses::VEHICLE_COMMS);

        FillTargetItem(*currentTarget->targetAs<command_target::DynamicTarget_Orientation>(),targetItem);

        m_deflectionMSG = deflectionItem;
    }

protected:
    void FillTargetItem(const command_item::Action_SetSurfaceDeflection &command, mavlink_set_surface_deflection_normalized_t &mavlinkItem)
    {
        mavlinkItem.aileron_deflection = command._surfaceDeflection.roll;
        mavlinkItem.elevator_deflection = command._surfaceDeflection.pitch;
        mavlinkItem.rudder_deflection = command._surfaceDeflection.yaw;
        mavlinkItem.throttle_deflection = command._surfaceDeflection.throttle;
    }

    mavlink_set_surface_deflection_normalized_t initializeMAVLINKDeflection()
    {
        mavlink_set_surface_deflection_normalized_t deflectionItem;

        deflectionItem.elevator_deflection = 0.0;
        deflectionItem.aileron_deflection = 0.0;
        deflectionItem.rudder_deflection = 0.0;
        deflectionItem.throttle_deflection = 0.0;

        deflectionItem.target_component =0;
        deflectionItem.target_system = 0;
        deflectionItem.surface_mask = (uint8_t)0; //by default we want to ignore all of the values
        return deflectionItem;
    }

public:
    Controller_SetSurfaceDeflection(const Controllers::IMessageNotifier<mavlink_message_t, MavlinkEntityKey> *cb, TransmitQueue *queue, int linkChan) :
        BasicMavlinkController_ModuleKeyed<command_item::Action_SetSurfaceDeflection>(cb, queue, linkChan),
        SurfaceDeflectionBroadcast(this, MavlinkEntityKeyToSysIDCompIDConverter<mavlink_set_surface_deflection_normalized_t>(mavlink_msg_set_surface_deflection_normalized_encode_chan))
    {

    }


};

} //end of namespace MAVLINKVehicleControllers


#endif // CONTROLLER_SET_SURFACE_DEFLECTION_H
