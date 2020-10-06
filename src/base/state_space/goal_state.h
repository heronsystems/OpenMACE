#ifndef GOAL_STATE_H
#define GOAL_STATE_H

#include "common/class_forward.h"

#include "generic_goal.h"

namespace mace {
namespace state_space {

MACE_CLASS_FORWARD(GoalRegion);
class GoalRegion
{
public:
    typedef std::function<double(const State*, const State*)> DistanceFunction;

public:
    GoalRegion(const double &value = 0.0):
        radialSatisfy(value), m_DistanceFunction(nullptr)
    {

    }

    GoalRegion(const GoalRegion &copy)
    {
        this->radialSatisfy = copy.radialSatisfy;
        this->m_DistanceFunction = copy.m_DistanceFunction;
    }

    virtual ~GoalRegion() = default;

    void setRadialRegion(const double &value)
    {
        radialSatisfy = value;
    }

    double getRadialRegion() const
    {
        return radialSatisfy;
    }

    virtual void setGoalSatisfiedFunction(const DistanceFunction &distFun) {
        m_DistanceFunction = distFun;
    }

    const DistanceFunction &getGoalSatisfiedFunction() const {
        return m_DistanceFunction;
    }

    virtual bool isGoalSatisfied(const State* current) = 0;

public:
    GoalRegion& operator = (const GoalRegion &rhs)
    {
        this->radialSatisfy = rhs.radialSatisfy;

        if(rhs.m_DistanceFunction != nullptr)
            this->m_DistanceFunction = rhs.m_DistanceFunction;

        return *this;
    }

protected:
    //!
    //! \brief metric allowing for a goal to be satisfied if this general criterion is met.
    //! This is a good metric as how sampling routines assess insertion into trees or roadmaps.
    //!
    double radialSatisfy = 0.0; //this is a metric not necessarily equating to distace

    //!
    //! \brief m_SatisfactoryFunction
    //!
    DistanceFunction m_DistanceFunction;

};

MACE_CLASS_FORWARD(GoalState);
class GoalState : public GoalRegion
{
public:

    GoalState(const double &value = 0.0);

    GoalState(const GoalState &copy);

    virtual ~GoalState() override;

public:
    void setState(const State *state);

    State* getState() const;

public:
    bool isGoalSatisfied(const State* current) override;


public:
    GoalState& operator = (const GoalState &rhs)
    {
        GoalRegion::operator=(rhs);

        if(rhs.goalState != nullptr)
            this->goalState = rhs.goalState->getStateClone();

        return *this;
    }

    /** Relational Operators */
public:
    //!
    //! \brief operator ==
    //! \param rhs
    //! \return
    //!
    bool operator == (const GoalState &rhs) const
    {
        UNUSED(rhs);
        return false;
    }

    //!
    //! \brief operator !=
    //! \param rhs
    //! \return
    //!
    bool operator != (const GoalState &rhs) {
        return !(*this == rhs);
    }

private:
    State* goalState;
};

} //end of namespace state_space
} //end of namespace mace

#endif // GOAL_STATE_H
