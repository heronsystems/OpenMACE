#ifndef STATE_TAKEOFF_H
#define STATE_TAKEOFF_H

#include <mavlink.h>

#include "abstract_root_state.h"

#include "../arducopter_target_progess.h"


namespace arducopter{
namespace state{

class State_Grounded;
class State_TakeoffClimbing;
class State_TakeoffTransitioning;
class State_TakeoffComplete;
class State_Flight;

class State_Takeoff : public AbstractRootState
{
public:
    State_Takeoff();

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

private:

};

} //end of namespace arducopter
} //end of namespace state

#endif // STATE_TAKEOFF_H
