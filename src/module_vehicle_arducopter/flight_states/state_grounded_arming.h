#ifndef STATE_GROUNDED_ARMING_H
#define STATE_GROUNDED_ARMING_H

#include "abstract_state_arducopter.h"

namespace arducopter{
namespace state{

class State_GroundedIdle;
class State_GroundedArmed;

class State_GroundedArming : public AbstractStateArducopter
{
public:
    State_GroundedArming();

    virtual ~State_GroundedArming() = default;

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

private:
    bool armingCheck;
};

} //end of namespace arducopter
} //end of namespace state

#endif // STATE_GROUNDED_ARMING_H
