#ifndef GOAL_STATE_H
#define GOAL_STATE_H

#include "common/class_forward.h"

#include "generic_goal.h"

namespace mace {
namespace state_space {

MACE_CLASS_FORWARD(GoalState);

class GoalState : public GoalSampler
{
public:
    GoalState(const StateSpacePtr &space, const double &value = 0.0);

    void setState(State *state);

    const State* getState() const;

    void sampleGoal(State* sample) override;

    bool isGoalSatisfied(const State* current);

private:
    State* goalState;
};

} //end of namespace state_space
} //end of namespace mace

#endif // GOAL_STATE_H
