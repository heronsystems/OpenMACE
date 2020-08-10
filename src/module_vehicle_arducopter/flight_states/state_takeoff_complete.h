#ifndef STATE_TAKEOFF_COMPLETE_H
#define STATE_TAKEOFF_COMPLETE_H

#include "abstract_state_arducopter.h"

namespace arducopter{
namespace state{


class State_TakeoffComplete : public AbstractStateArducopter
{
public:
    State_TakeoffComplete();

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


#endif // STATE_TAKEOFF_COMPLETE_H
