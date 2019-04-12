#ifndef GENERIC_GOAL_H
#define GENERIC_GOAL_H

#include <functional>

#include "state_space.h"

namespace mace {
namespace state_space {

MACE_CLASS_FORWARD(Start);

class Start{
public:
    Start(const StateSpacePtr &space):
        stateSpace(space)
    {

    }

    ~Start() = default;

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

MACE_CLASS_FORWARD(StartRegion);

class StartRegion: public Start
{
public:
    StartRegion(const StateSpacePtr &space, const double &value = 0.0):
        Start(space), radialSatisfy(value)
    {

    }

    ~StartRegion() = default;

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
    //! \brief metric allowing for a start to be satisfied if this general criterion is met.
    //! This is a good metric as how sampling routines assess insertion into trees or roadmaps.
    //!
    double radialSatisfy = 0.0; //this is a metric not necessarily equating to distace
};

MACE_CLASS_FORWARD(StartSampler);
class StartSampler : public StartRegion
{
public:
    typedef std::function<void(State*)> SampleFunction;

    StartSampler(const StateSpacePtr &space, const double &value = 0.0):
        StartRegion(space, value)
    {

    }

    virtual void sampleStart(State* sample) = 0;

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
