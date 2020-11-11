#ifndef AP_STATE_FLIGHT_AI_DEFLECTION_H
#define AP_STATE_FLIGHT_AI_DEFLECTION_H

#include <mavlink.h>

#include <iostream>


#include "module_vehicle_ardupilot/flight_states/abstract_state_ardupilot.h"

#include "data_generic_command_item/command_item_components.h"

#include "module_vehicle_MAVLINK/controllers/controller_set_surface_deflection.h"

namespace ardupilot {
namespace state{

class AP_State_FlightAI_Deflection : public AbstractStateArdupilot
{
public:
    AP_State_FlightAI_Deflection();

    void OnExit() override;

public:
    AbstractStateArdupilot* getClone() const override;

    void getClone(AbstractStateArdupilot** state) const override;

public:
    hsm::Transition GetTransition() override;

public:
    bool handleCommand(const std::shared_ptr<AbstractCommandItem> command) override;

    void Update() override;

    void OnEnter() override;

    void OnEnter(const std::shared_ptr<AbstractCommandItem> command) override;

public:
    void constructAndSendTarget(const command_item::Action_SetSurfaceDeflection &command)
    {
        MavlinkEntityKey sender = 255;
        static_cast<MAVLINKUXVControllers::Controller_SetSurfaceDeflection*>(Owner().ControllersCollection()->At("AttitudeTargetController"))->Broadcast(command, sender);
    }
};

} //end of namespace state
} //end of namespace ardupilot

#endif // AP_STATE_FLIGHT_AI_DEFLECTION_H
