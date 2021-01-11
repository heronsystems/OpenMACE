#ifndef STATE_GROUNDED_ARMED_H
#define STATE_GROUNDED_ARMED_H

#include "abstract_state_ardupilot.h"

namespace ardupilot{
namespace state{

class State_GroundedIdle;
class State_GroundedDisarming;

class State_GroundedArmed : public AbstractStateArdupilot
{
public:
    State_GroundedArmed();

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

#endif // STATE_GROUNDED_ARMED_H
