#ifndef GENERIC_GOAL_H
#define GENERIC_GOAL_H

#include <functional>

#include "state_space.h"

namespace mace {
namespace state_space {

MACE_CLASS_FORWARD(GoalSampler);
class GoalSampler
{
public:
    typedef std::function<void(State*)> SampleFunction;

    GoalSampler(const StateSpacePtr &space, const double &value = 0.0):
        m_StateSpace(space)
    {
        UNUSED(value);
    }

    virtual void sampleGoal(State* sample) = 0;

    virtual void setSampleFunction(const SampleFunction &distFun) {
        m_SamplerFunction = distFun;
    }

    const SampleFunction &getSampleFunction() const {
        return m_SamplerFunction;
    }

protected:
    SampleFunction m_SamplerFunction;

    const StateSpacePtr m_StateSpace;
};

} //end of namespace state_space
} //end of namespace mace

#endif // GENERIC_GOAL_H
