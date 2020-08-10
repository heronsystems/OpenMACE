#ifndef STATE_FLIGHT_BRAKE_H
#define STATE_FLIGHT_BRAKE_H

#include "abstract_state_arducopter.h"

namespace arducopter{
namespace state{

class State_FlightBrake : public AbstractStateArducopter
{
public:
    State_FlightBrake();

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

} //end of namespace arducopter
} //end of namespace state

#endif // STATE_FLIGHT_BRAKE_H
