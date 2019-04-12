#ifndef GENERIC_GOAL_H
#define GENERIC_GOAL_H

#include <functional>

#include "state_space.h"

namespace mace {
namespace state_space {

MACE_CLASS_FORWARD(Goal);

class Goal{
public:
    Goal(const StateSpacePtr &space):
        stateSpace(space)
    {

    }

    ~Goal() = default;

    template <class T>
    T *as()
    {
        return static_cast<T*>(this);
    }

    template <class T>
    const T *as() const
    {
        return static_cast<const T*>(this);
    }

public:
    const StateSpacePtr getStateSpace() const
    {
        return stateSpace;
    }
protected:
    const StateSpacePtr stateSpace;
};

MACE_CLASS_FORWARD(GoalRegion);

class GoalRegion: public Goal
{
public:
    GoalRegion(const StateSpacePtr &space, const double &value = 0.0):
        Goal(space), radialSatisfy(value)
    {

    }

    ~GoalRegion() = default;

    void setRadialRegion(const double &value)
    {
        radialSatisfy = value;
    }

    double getRadialRegion() const
    {
        return radialSatisfy;
    }

protected:
    //!
    //! \brief metric allowing for a goal to be satisfied if this general criterion is met.
    //! This is a good metric as how sampling routines assess insertion into trees or roadmaps.
    //!
    double radialSatisfy = 0.0; //this is a metric not necessarily equating to distace
};

MACE_CLASS_FORWARD(GoalSampler);
class GoalSampler : public GoalRegion
{
public:
    typedef std::function<void(State*)> SampleFunction;

    GoalSampler(const StateSpacePtr &space, const double &value = 0.0):
        GoalRegion(space, value)
    {

    }

    virtual void sampleGoal(State* sample) = 0;

    virtual void setSampleFunction(const SampleFunction &distFun) {
        sampleFunction = distFun;
    }

    const SampleFunction &getSampleFunction() const {
        return sampleFunction;
    }

protected:
    SampleFunction sampleFunction;
};

} //end of namespace state_space
} //end of namespace mace

#endif // GENERIC_GOAL_H
