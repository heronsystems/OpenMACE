#include "goal_state.h"

namespace mace {
namespace state_space {

GoalState::GoalState(const StateSpacePtr &space, const double &value):
    GoalSampler(space,value)
{
    this->setSampleFunction([this](State* sample)
    {
        sample = goalState;
    });
}

void GoalState::setState(State* state)
{
    goalState = state;
}

const State* GoalState::getState() const
{
    return goalState;
}

void GoalState::sampleGoal(State* sample)
{
    sampleFunction(sample);
}

bool GoalState::isGoalSatisfied(const State *current)
{
    double distance = stateSpace->distanceBetween(goalState,current);
    if(distance < radialSatisfy)
        return true;
    return false;
}

} //end of namespace state_space
} //end of namespace mace
