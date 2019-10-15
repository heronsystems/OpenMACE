#ifndef STATE_FLIGHT_GUIDED_IDLE_H
#define STATE_FLIGHT_GUIDED_IDLE_H

#include <mavlink.h>

#include <iostream>

#include "abstract_state_ardupilot.h"

#include "data_generic_command_item/command_item_components.h"

namespace ardupilot{
namespace state{

class State_FlightGuided_Idle : public AbstractStateArdupilot
{
public:
    State_FlightGuided_Idle();

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
} //end of namespace arudpilot

#endif // STATE_FLIGHT_GUIDED_IDLE_H
