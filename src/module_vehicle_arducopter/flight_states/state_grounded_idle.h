#ifndef STATE_GROUNDEDIDLE_H
#define STATE_GROUNDEDIDLE_H

#include "abstract_state_arducopter.h"

namespace arducopter{
namespace state{

class State_GroundedArming;
class State_GroundedArmed;

class State_GroundedIdle : public AbstractStateArducopter
{
public:
    State_GroundedIdle();

public:
    AbstractStateArducopter* getClone() const override;

    void getClone(AbstractStateArducopter** state) const override;

public:
    hsm::Transition GetTransition() override;

public:
    bool handleCommand(const std::shared_ptr<AbstractCommandItem> command) override;

    void Update() override;

    void OnEnter() override;

    void OnEnter(const std::shared_ptr<AbstractCommandItem> command);

    void OnExit() override;
};

} //end of namespace arducopter
} //end of namespace state

#endif // STATE_GROUNDEDIDLE_H
