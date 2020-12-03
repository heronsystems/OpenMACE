#ifndef AP_STATE_FLIGHT_AI_INITIALIZE_H
#define AP_STATE_FLIGHT_AI_INITIALIZE_H

#include <iostream>

#include <mavlink.h>

#include "module_vehicle_ardupilot/flight_states/abstract_state_ardupilot.h"

#include "data_generic_command_item/command_item_components.h"

namespace ardupilot {
namespace state{

class AP_State_FlightAI_Abort;
class AP_State_FlightAI_Deflection;
class AP_State_FlightAI_EvalEnd;

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

public:
    bool handleCommand(const std::shared_ptr<AbstractCommandItem> command) override;

    void Update() override;

    void OnEnter() override;

    void OnEnter(const std::shared_ptr<AbstractCommandItem> command) override;
};

} //end of namespace state
} //end of namespace ardupilot

#endif // AP_STATE_FLIGHT_AI_INITIALIZE_H
