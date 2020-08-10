#ifndef STATE_UNKNOWN_H
#define STATE_UNKNOWN_H

#include "abstract_state_arducopter.h"

namespace arducopter{
namespace state{

class State_Grounded;
class State_Takeoff;
class State_Flight;
class State_Landing;

class State_Unknown : public AbstractStateArducopter
{
public:
    State_Unknown();

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


#endif // STATE_UNKNOWN_H
