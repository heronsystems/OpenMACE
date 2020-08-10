#ifndef STATE_GROUNDED_H
#define STATE_GROUNDED_H

#include "abstract_root_state.h"

namespace arducopter{
namespace state{

class State_GroundedIdle;
class State_GroundedArming;
class State_GroundedArmed;
class State_GroundedDisarming;
class State_GroundedDisarmed;

class State_Takeoff;

class State_Flight;

class State_Grounded : public AbstractRootState
{
public:
    State_Grounded();

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

#endif // STATE_GROUNDED_H
