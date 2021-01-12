#ifndef AI_TESTCONDITIONAL_H
#define AI_TESTCONDITIONAL_H

#include <iostream>
#include "ai_test_boundary.h"
#include "ai_test_criteria.h"

namespace DataGenericItem {

class AI_TestConditional
{
public:
    AI_TestConditional();

    AI_TestConditional(const AI_TestBoundary &boundary, const AI_TestCriteria &criteria);

    AI_TestConditional(const AI_TestConditional &copy);

public:
    AI_PROCEDURAL_COMMANDS shouldStart(const mace::pose::BasicCarState3D &startState, const VehicleState_Cartesian3D &currentState, const mace::pose::GeodeticPosition_3D &currentPosition) const;

    //!
    //! \brief shouldTerminate
    //! \param elapsedTestDuration the elapsed test duration in milliseconds
    //! \param endState
    //! \param currentPosition
    //! \return
    //!
    AI_PROCEDURAL_COMMANDS shouldTerminate(const unsigned int &elapsedTestDuration, const VehicleState_Cartesian3D &endState, const mace::pose::GeodeticPosition_3D &currentPosition) const;

public:
    void operator = (const AI_TestConditional &rhs)
    {
        m_Boundary = rhs.m_Boundary;
        m_Criteria = rhs.m_Criteria;
    }

    bool operator == (const AI_TestConditional &rhs) const{
        if(this->m_Boundary != rhs.m_Boundary){
            return false;
        }
        if(this->m_Criteria != rhs.m_Criteria){
            return false;
        }
        return true;
    }

    bool operator != (const AI_TestConditional &rhs) const {
        return !(*this == rhs);
    }

public:

    friend std::ostream &operator<<(std::ostream &out, const AI_TestConditional &obj)
    {
        UNUSED(obj);
        return out;
    }

public:
    // Start criteria:
    AI_TestBoundary m_Boundary;

    // Abort criteria:
    AI_TestCriteria m_Criteria;

};


} //end of namespace DataGenericItem

#endif // AI_TESTCONDITIONAL_H
