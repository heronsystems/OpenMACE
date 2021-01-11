#include "ai_test_conditional.h"

namespace DataGenericItem {

AI_TestConditional::AI_TestConditional()
{

}

AI_TestConditional::AI_TestConditional(const AI_TestBoundary &boundary, const AI_TestCriteria &criteria)
{
    m_Boundary = boundary;
    m_Criteria = criteria;
}

AI_TestConditional::AI_TestConditional(const AI_TestConditional &copy)
{
    m_Boundary = copy.m_Boundary;
    m_Criteria = copy.m_Criteria;
}

AI_PROCEDURAL_COMMANDS AI_TestConditional::shouldStart(const mace::pose::BasicCarState3D &startState, const VehicleState_Cartesian3D &currentState, const mace::pose::GeodeticPosition_3D &currentPosition) const
{
    //This is where we should check the the criteria against the start state
    AI_PROCEDURAL_COMMANDS rtnCondition = AI_PROCEDURAL_COMMANDS::ABORT;
    if(!m_Boundary.validateState(currentPosition)) {
        rtnCondition = AI_PROCEDURAL_COMMANDS::ABORT;
    }

    mace::pose::CartesianPosition_3D startPosition = startState.getPosition();

    double dxCondition = fabs(currentState.m_Position.deltaX(startPosition));
    double dyCondition = fabs(currentState.m_Position.deltaY(startPosition));
    double dzCondition = fabs(currentState.m_Position.deltaZ(startPosition));

    bool isPositionalSatisfied = false, isRotationalSatisfied = false;

    if((dxCondition <= m_Criteria._startCriteria.get_dX()) && (dyCondition <= m_Criteria._startCriteria.get_dY()) &&
            (dzCondition <= m_Criteria._startCriteria.get_dZ()))
    {
        isPositionalSatisfied = true;
    }

    double dYaw = fabs(currentState.m_Rotation.getYaw() - startState.getRotation().getYaw());
    if(dYaw <= m_Criteria._startCriteria.get_dYaw())
    {
        isRotationalSatisfied = true;
    }

    double distance = currentState.m_Position.distanceBetween3D(&startPosition);
    if(distance < 10)
    {
        std::cout<<"The total distance from the target location is: "<<distance<<std::endl;
        std::cout<<"The X distance from the target location is: "<<dxCondition<<std::endl;
        std::cout<<"The Y distance from the target location is: "<<dyCondition<<std::endl;
        std::cout<<"The Z distance from the target location is: "<<dzCondition<<std::endl;
        std::cout<<"The Yaw from the target location is: "<<dYaw<<std::endl;
        std::cout<<"__________________________________________________________"<<std::endl;
    }


    if(isPositionalSatisfied && isRotationalSatisfied)
    {
        rtnCondition = AI_PROCEDURAL_COMMANDS::START;
    }
    else
    {
        rtnCondition = AI_PROCEDURAL_COMMANDS::RELEASE;
    }

    return rtnCondition;
}

AI_PROCEDURAL_COMMANDS AI_TestConditional::shouldTerminate(const unsigned int &elapsedTestDuration, const VehicleState_Cartesian3D &endState, const mace::pose::GeodeticPosition_3D &currentPosition) const
{
    if(!m_Boundary.validateState(currentPosition))
        return AI_PROCEDURAL_COMMANDS::ABORT;
    if(elapsedTestDuration > m_Criteria.getMaxTestDuration())
        return AI_PROCEDURAL_COMMANDS::STOP;

    return AI_PROCEDURAL_COMMANDS::RELEASE;
}

} // end of namespace DataGenericItem
