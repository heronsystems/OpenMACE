#ifndef START_STATE_H
#define START_STATE_H

#include "common/class_forward.h"
#include "generic_start.h"

namespace mace {
namespace state_space {

MACE_CLASS_FORWARD(StartState);

class StartState : public StartSampler
{
public:
    StartState(const StateSpacePtr &space, const double &value = 0.0);

    void setState(State *state);

    const State* getState() const;

    void sampleStart(State* sample) override;

private:
    State* startState;
};

} //end of namespace state_space
} //end of namespace mace

#endif // START_STATE_H
