#ifndef CONTROLLER_SET_SURFACE_DEFLECTION_H
#define CONTROLLER_SET_SURFACE_DEFLECTION_H

#include <mavlink.h>

#include "common/common.h"

#include "controllers/actions/action_broadcast.h"

#include "module_vehicle_MAVLINK/mavlink_entity_key.h"

#include "module_vehicle_MAVLINK/controllers/common.h"

#include "data_generic_command_item/do_items/action_set_surface_deflection.h"


namespace MAVLINKUXVControllers {

using OverrideSurfaceDeflectionBroadcast = Controllers::ActionBroadcast<
    mavlink_message_t,
    MavlinkEntityKey,
    BasicMavlinkController_ModuleKeyed<command_item::Action_SetSurfaceDeflection>,
    command_item::Action_SetSurfaceDeflection,
    mavlink_execute_surface_deflection_override_t
>;


class Controller_SetSurfaceDeflection : public BasicMavlinkController_ModuleKeyed<command_item::Action_SetSurfaceDeflection>,
        public OverrideSurfaceDeflectionBroadcast
{
protected:

    void Construct_Broadcast(const command_item::Action_SetSurfaceDeflection &command, const MavlinkEntityKey &sender, mavlink_execute_surface_deflection_override_t &override) override
    {
        UNUSED(sender);

        override = initializeSurfaceDeflection();
        PopulateSurfaceDeflection(command, override);
    }

protected:
    void PopulateSurfaceDeflection(const command_item::Action_SetSurfaceDeflection &command, mavlink_execute_surface_deflection_override_t &mavlinkItem)
    {
        mavlinkItem.deflection_rudder = static_cast<float>(command._surfaceDeflection._rudder);
        mavlinkItem.deflection_aileron = static_cast<float>(command._surfaceDeflection._aileron);
        mavlinkItem.deflection_elevator = static_cast<float>(command._surfaceDeflection._elevator);
        mavlinkItem.deflection_throttle = static_cast<float>(command._surfaceDeflection._throttle);
    }

    mavlink_execute_surface_deflection_override_t initializeSurfaceDeflection()
    {
        mavlink_execute_surface_deflection_override_t surfaceOverride;

        surfaceOverride.deflection_rudder = 0;
        surfaceOverride.deflection_aileron = 0;
        surfaceOverride.deflection_elevator = 0;
        surfaceOverride.deflection_throttle = 0;

        return surfaceOverride;
    }

public:
    Controller_SetSurfaceDeflection(const Controllers::IMessageNotifier<mavlink_message_t, MavlinkEntityKey> *cb, TransmitQueue *queue, int linkChan) :
        BasicMavlinkController_ModuleKeyed<command_item::Action_SetSurfaceDeflection>(cb, queue, linkChan),
        OverrideSurfaceDeflectionBroadcast(this, MavlinkEntityKeyToSysIDCompIDConverter<mavlink_execute_surface_deflection_override_t>(mavlink_msg_execute_surface_deflection_override_encode_chan))
    {

    }

};

} //end of namespace MAVLINKVehicleControllers


#endif // CONTROLLER_SET_SURFACE_DEFLECTION_H
