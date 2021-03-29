#ifndef MAVLINK_CONTROLLER_COMMAND_SET_SURFACE_DEFLECTION_H
#define MAVLINK_CONTROLLER_COMMAND_SET_SURFACE_DEFLECTION_H

#include <iostream>

#include "mavlink.h"

#include "common/common.h"

#include "controllers/actions/action_broadcast.h"
#include "controllers/actions/action_unsolicited_receive.h"

#include "controllers/controllers_MAVLINK/common.h"

#include "data_generic_command_item/do_items/action_set_surface_deflection.h"

namespace MAVLINKUXVControllers {

namespace ModuleController {

using ActionSend_SurfaceDeflectionBroadcast = Controllers::ActionBroadcast<
    mavlink_message_t,
    MaceCore::ModuleCharacteristic,
    BasicExternalLinkController_ModuleKeyed<command_item::Action_SetSurfaceDeflection>,
    command_item::Action_SetSurfaceDeflection,
    mavlink_execute_surface_deflection_override_t
>;

//Receive a broadcasted test procedural event position, accept and finish (no response)
using ActionReceive_SurfaceDeflectionBroadcast = Controllers::ActionUnsolicitedReceive<
    mavlink_message_t,
    MaceCore::ModuleCharacteristic,
    BasicExternalLinkController_ModuleKeyed<command_item::Action_SetSurfaceDeflection>,
    MaceCore::ModuleCharacteristic,
    command_item::Action_SetSurfaceDeflection,
    mavlink_execute_surface_deflection_override_t,
    MAVLINK_MSG_ID_EXECUTE_SURFACE_DEFLECTION_OVERRIDE
>;

class ControllerCommand_SetSurfaceDeflection : public BasicExternalLinkController_ModuleKeyed<command_item::Action_SetSurfaceDeflection>,
        public ActionSend_SurfaceDeflectionBroadcast,
        public ActionReceive_SurfaceDeflectionBroadcast
{

protected:

    void Construct_Broadcast(const command_item::Action_SetSurfaceDeflection &cmd, const MaceCore::ModuleCharacteristic &sender, mavlink_execute_surface_deflection_override_t &deflection) override
    {
        UNUSED(sender);
        deflection = initializeSurfaceDeflection();
        PopulateSurfaceDeflection(cmd, deflection);
    }

    bool Construct_FinalObject(const mavlink_execute_surface_deflection_override_t &msg, const MaceCore::ModuleCharacteristic &sender, MaceCore::ModuleCharacteristic &key, command_item::Action_SetSurfaceDeflection &data) override
    {
        key = sender;
        data.setTargetSystem(sender.ModuleID);
        data.setOriginatingSystem(sender.MaceInstance);
        data._surfaceDeflection._aileron = static_cast<float>(msg.deflection_aileron);
        data._surfaceDeflection._elevator = static_cast<float>(msg.deflection_elevator);
        data._surfaceDeflection._rudder = static_cast<float>(msg.deflection_rudder);
        data._surfaceDeflection._throttle = static_cast<float>(msg.deflection_throttle);
        return true;
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
    ControllerCommand_SetSurfaceDeflection(const Controllers::IMessageNotifier<mavlink_message_t, MaceCore::ModuleCharacteristic> *cb, TransmitQueue *queue, int linkChan) :
        BasicExternalLinkController_ModuleKeyed<command_item::Action_SetSurfaceDeflection>(cb, queue, linkChan, "SetSurfaceDeflection", false),
        ActionSend_SurfaceDeflectionBroadcast(this, ModuleToSysIDCompIDConverter<mavlink_execute_surface_deflection_override_t>(mavlink_msg_execute_surface_deflection_override_encode_chan)),
        ActionReceive_SurfaceDeflectionBroadcast(this, mavlink_msg_execute_surface_deflection_override_decode)
    {

    }

};

} //end of namespace ModuleController

} //end of namespace MAVLINKVehicleControllers

#endif // MAVLINK_CONTROLLER_COMMAND_SET_SURFACE_DEFLECTION_H
