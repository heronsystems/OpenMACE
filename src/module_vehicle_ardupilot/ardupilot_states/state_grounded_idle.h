#ifndef STATE_GROUNDEDIDLE_H
#define STATE_GROUNDEDIDLE_H

#include "abstract_state_ardupilot.h"

namespace ardupilot{
namespace state{

class State_GroundedArming;
class State_GroundedArmed;

class State_GroundedIdle : public AbstractStateArdupilot
{
public:
    State_GroundedIdle();

public:
    AbstractStateArdupilot* getClone() const override;

    void getClone(AbstractStateArdupilot** state) const override;

public:
    hsm::Transition GetTransition() override;

public:
    bool handleCommand(const std::shared_ptr<AbstractCommandItem> command) override;

    void Update() override;

    void OnEnter() override;

    void OnEnter(const std::shared_ptr<AbstractCommandItem> command);

    void OnExit() override;
};

} //end of namespace ardupilot
} //end of namespace state

#endif // STATE_GROUNDEDIDLE_H
