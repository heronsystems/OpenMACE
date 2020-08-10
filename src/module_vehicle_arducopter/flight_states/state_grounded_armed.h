#ifndef STATE_GROUNDED_ARMED_H
#define STATE_GROUNDED_ARMED_H

#include "abstract_state_arducopter.h"

namespace arducopter{
namespace state{

class State_GroundedIdle;
class State_GroundedDisarming;

class State_GroundedArmed : public AbstractStateArducopter
{
public:
    State_GroundedArmed();

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

#endif // STATE_GROUNDED_ARMED_H
