#ifndef AP_STATE_FLIGHT_AI_EXECUTE_ABORT_H
#define AP_STATE_FLIGHT_AI_EXECUTE_ABORT_H

#include <iostream>

#include <mavlink.h>

#include "module_vehicle_ardupilot/flight_states/abstract_state_ardupilot.h"

#include "data_generic_command_item/command_item_components.h"

namespace ardupilot {
namespace state{


class AP_State_FlightAI_ExecuteAbort : public AbstractStateArdupilot
{
public:
    AP_State_FlightAI_ExecuteAbort();

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

#endif // AP_STATE_FLIGHT_AI_EXECUTE_ABORT_H
