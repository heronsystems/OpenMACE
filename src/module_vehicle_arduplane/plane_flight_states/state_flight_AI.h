#ifndef AP_STATE_FLIGHT_AI_H
#define AP_STATE_FLIGHT_AI_H

#include <mavlink.h>

#include <iostream>

#include "module_vehicle_ardupilot/flight_states/abstract_state_ardupilot.h"
#include "data_generic_command_item/command_item_components.h"


namespace ardupilot {

namespace state{

class AP_State_FlightAI_Abort;
class AP_State_FlightAI_Initialize;
class AP_State_FlightAI_Execute;
class AP_State_FlightAI_EvalEnd;

class AP_State_FlightAI : public AbstractStateArdupilot
{
public:
    AP_State_FlightAI();

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

    void OnEnter(const Action_SetSurfaceDeflection &command);

    void OnEnter(const command_item::Action_InitializeTestSetup &initialization);

public:
    bool shouldExecuteModeTransition(const uint8_t &mode) override
    {
        ardupilot::state::AbstractStateArdupilot* currentInnerState = static_cast<ardupilot::state::AbstractStateArdupilot*>(GetImmediateInnerState());
        if(currentInnerState == nullptr)
            return true;
        return currentInnerState->shouldExecuteModeTransition(mode);
    }

private:
    command_item::Action_SetSurfaceDeflection m_SurfaceDeflection;

    command_item::Action_InitializeTestSetup m_InitializationConditions;
};

} //end of namespace state
} //end of namespace arudcopter


#endif // AP_STATE_FLIGHT_AI_H
