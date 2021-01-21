#ifndef AP_STATE_FLIGHT_AI_INITIALIZE_H
#define AP_STATE_FLIGHT_AI_INITIALIZE_H

#include <iostream>

#include <mavlink.h>

#include "module_vehicle_ardupilot/flight_states/abstract_state_ardupilot.h"

#include "data_generic_command_item/command_item_components.h"

namespace ardupilot {
namespace state{

class AP_State_FlightAI_TestEnd;
class AP_State_FlightAI_Execute;

class AP_State_FlightAI_Initialize_ABORT;
class AP_State_FlightAI_Initialize_ROUTE;

class AP_State_FlightAI_Initialize : public AbstractStateArdupilot
{
public:
    AP_State_FlightAI_Initialize();

    void OnExit() override;

public:
    AbstractStateArdupilot* getClone() const override;

    void getClone(AbstractStateArdupilot** state) const override;

public:
    hsm::Transition GetTransition() override;

private:
    void setupGuidedMode();

public:
    bool handleCommand(const std::shared_ptr<command_item::AbstractCommandItem> command) override;

    void Update() override;

    void OnEnter() override;

    void OnEnter(const std::shared_ptr<command_item::AbstractCommandItem> command) override;

    void OnEnter(const command_item::Action_InitializeTestSetup &initialization);

public:

    bool notifyOfImpendingModeChange(const uint8_t &mode) override
    {
        bool acceptTransition = true;
        ardupilot::state::AbstractStateArdupilot* currentInnerState = static_cast<ardupilot::state::AbstractStateArdupilot*>(GetImmediateInnerState());
       
        if(currentInnerState != nullptr)
            acceptTransition = currentInnerState->notifyOfImpendingModeChange(mode);

        if(acceptTransition == false)
        {
            _impendingModeChange = static_cast<PLANE_MODE>(mode);
            _flagFlightModeChange = true;
        }
        
        return acceptTransition;
    }

private:
    command_item::Action_SetSurfaceDeflection m_SurfaceDeflection;
    command_item::Action_InitializeTestSetup m_InitializationConditions;

    /* The following member variables are to deal with the transitioning elements of the mode changes that may occur 
    while running a test but not yet commanded via Adept. For example, the remote or the GUI. */
    bool _flagFlightModeChange = false;
    PLANE_MODE _impendingModeChange = PLANE_MODE_RTL;

};

} //end of namespace state
} //end of namespace ardupilot

#endif // AP_STATE_FLIGHT_AI_INITIALIZE_H
