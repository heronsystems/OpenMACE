#ifndef STATE_GROUNDED_H
#define STATE_GROUNDED_H

#include "abstract_root_state.h"

namespace ardupilot{
namespace state{

class State_GroundedIdle;
class State_GroundedArming;
class State_GroundedArmed;
class State_GroundedDisarming;
class State_GroundedDisarmed;

class State_Takeoff;

class State_Grounded : public AbstractRootState
{
public:
    State_Grounded();

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

#endif // STATE_GROUNDED_H
