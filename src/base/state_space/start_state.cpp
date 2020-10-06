#include "start_state.h"

namespace mace {
namespace state_space {

StartState::StartState(const StateSpacePtr &space, const double &value):
    StartSampler(space,value)
{
    this->setSampleFunction([this](State* sample)
    {
        stateSpace->getNewState();
        sample = startState;
        if(sample != nullptr){} // Warning suppression
    });
}

void StartState::setState(State* state)
{
    startState = state;
}

const State* StartState::getState() const
{
    return startState;
}

void StartState::sampleStart(State* sample)
{
    sampleFunction(sample);
}

} //end of namespace state_space
} //end of namespace mace
