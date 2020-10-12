#include "goal_state.h"

namespace mace {
namespace state_space {

GoalState::GoalState(const double &value):
    GoalRegion (value), goalState(nullptr)
{

}

GoalState::GoalState(const GoalState &copy):
    GoalRegion(copy)
{
    if(copy.goalState != nullptr)
        this->goalState = copy.goalState->getStateClone();
}

GoalState::~GoalState()
{
    if(goalState != nullptr)
    {
        delete goalState;
        goalState = nullptr;
    }
}

void GoalState::setState(const State* state)
{
    goalState = state->getStateClone();
}

State* GoalState::getState() const
{
    return goalState;
}

bool GoalState::isGoalSatisfied(const State *current)
{
    double distance = std::numeric_limits<double>::max();

    if(m_DistanceFunction != nullptr)
        distance = m_DistanceFunction(current,goalState);

    if(distance <= radialSatisfy)
        return true;

    return false;
}

} //end of namespace state_space
} //end of namespace mace
