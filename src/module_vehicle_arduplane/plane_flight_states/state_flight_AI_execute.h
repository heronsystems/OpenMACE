#ifndef AP_STATE_FLIGHT_AI_EXECUTE_H
#define AP_STATE_FLIGHT_AI_EXECUTE_H

#include <iostream>

#include <mavlink.h>

#include "module_vehicle_ardupilot/flight_states/abstract_state_ardupilot.h"

#include "data_generic_command_item/command_item_components.h"

namespace ardupilot {
namespace state{

class AP_State_FlightAI_Abort;

class AP_State_FlightAI_ExecuteAbort;
class AP_State_FlightAI_ExecuteDeflection;
class AP_State_FlightAI_ExecuteEnd;

class AP_State_FlightAI_Execute : public AbstractStateArdupilot
{
public:
    AP_State_FlightAI_Execute();

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

    void OnEnter(const command_item::Action_SetSurfaceDeflection &command);

    void OnEnter(const std::shared_ptr<AbstractCommandItem> command) override;

public:
    bool shouldExecuteModeTransition(const uint8_t &mode) override
    {
        if(mode == PLANE_MODE::PLANE_MODE_AI_DEFLECTION)
        {
            return false;
        }
        else
        {
            ardupilot::state::AbstractStateArdupilot* currentInnerState = static_cast<ardupilot::state::AbstractStateArdupilot*>(GetImmediateInnerState());
            if(currentInnerState == nullptr)
                return true;
            else
                return currentInnerState->shouldExecuteModeTransition(mode);
        }
    }

private:
    void setupAIMode();

private:
    command_item::Action_SetSurfaceDeflection m_SurfaceDeflection;

};

} //end of namespace state
} //end of namespace ardupilot

#endif // AP_STATE_FLIGHT_AI_EXECUTE_H
