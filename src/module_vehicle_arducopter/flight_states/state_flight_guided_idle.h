#ifndef STATE_FLIGHT_GUIDED_IDLE_H
#define STATE_FLIGHT_GUIDED_IDLE_H

#include <mavlink.h>

#include <iostream>

#include "abstract_state_arducopter.h"

#include "data_generic_command_item/command_item_components.h"

namespace arducopter{
namespace state{

class State_FlightGuided_Idle : public AbstractStateArducopter
{
public:
    State_FlightGuided_Idle();

    void OnExit() override;

public:
    AbstractStateArducopter* getClone() const override;

    void getClone(AbstractStateArducopter** state) const override;

public:
    hsm::Transition GetTransition() override;

public:
    bool handleCommand(const std::shared_ptr<AbstractCommandItem> command) override;

    void Update() override;

    void OnEnter() override;

    void OnEnter(const std::shared_ptr<AbstractCommandItem> command) override;
};

} //end of namespace state
} //end of namespace arudcopter

#endif // STATE_FLIGHT_GUIDED_IDLE_H
