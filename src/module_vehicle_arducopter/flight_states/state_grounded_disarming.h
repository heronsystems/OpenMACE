#ifndef STATE_GROUNDED_DISARMING_H
#define STATE_GROUNDED_DISARMING_H


#include "module_vehicle_ardupilot/flight_states/abstract_state_ardupilot.h"

namespace ardupilot {
namespace state{

class State_GroundedArmed;
class State_GroundedDisarmed;

class State_GroundedDisarming : public AbstractStateArdupilot
{
public:
    State_GroundedDisarming();

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

} //end of namespace ardupilot
} //end of namespace state

#endif // STATE_GROUNDED_DISARMING_H
