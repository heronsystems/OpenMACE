#ifndef PLANNERS_H
#define PLANNERS_H

#include <list>

#include "planners_global.h"

#include "base/state_space/goal_state.h"
#include "base/state_space/space_information.h"

namespace mace{
namespace planners {

class Planner_Interface
{
public:
    virtual ~Planner_Interface() = default;

public:
    virtual void cbiPlanner_SampledState(const state_space::State* sampleState) = 0;
    virtual void cbiPlanner_NewConnection(const state_space::State* beginState, const state_space::State* secondState) = 0;
};

class PLANNERSSHARED_EXPORT Planners
{

public:
    Planners(const state_space::SpaceInformationPtr &spaceInfo = nullptr);

    virtual ~Planners() = default;

    virtual std::vector<state_space::State*> solve() = 0;

    virtual void setPlanningSpaceInfo(const state_space::SpaceInformationPtr spaceInfo);

    virtual void setPlanningParameters(state_space::GoalState* begin, state_space::GoalState* end) = 0;

    void setCallbackFunction(Planner_Interface* callback)
    {
        m_CB = callback;
    }

protected:
    void createLog();

protected:
    state_space::SpaceInformationPtr m_spaceInfo;

    state_space::GoalState* m_stateBegin;

    state_space::GoalState* m_stateEnd;

    Planner_Interface* m_CB;
};

} //end of namespace planners
} //end of namespace mace

#endif // PLANNERS_H
