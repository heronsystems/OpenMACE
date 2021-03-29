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

AI_PROCEDURAL_COMMANDS AI_TestConditional::shouldTerminate(const double &elapsedTestDuration, const mace::pose::GeodeticPosition_3D &currentPosition, const mace::pose::Rotation_3D &currentRotation) const
{
    // **** Attitude:
    Eigen::Quaterniond orientationEstimate_NED = Eigen::Quaterniond(currentRotation.getQuaternion().w(), currentRotation.getQuaternion().x(), currentRotation.getQuaternion().y(), currentRotation.getQuaternion().z());
    orientationEstimate_NED.normalize();
    // Current rotation should be in NED:
    double roll_deg = mace::math::convertRadiansToDegrees(currentRotation.getRoll());
    double pitch_deg = mace::math::convertRadiansToDegrees(currentRotation.getPitch());


    // CHECK ABORT CONDITIONS:
    // TODO-PAT: Fix boundary load at test start. Long boundary waypoint lists don't load properly
//    if(!m_Boundary.validateState(currentPosition)) {
//        MaceLog::Emergency("ABORT DUE TO BOUNDARY VIOLATION: ");
//        MaceLog::Emergency("POSITON: " + currentPosition.printPositionalInfo());
//        return AI_PROCEDURAL_COMMANDS::ABORT;
//    }
    if(currentPosition.getAltitude() > this->m_Boundary._ceiling ||
       currentPosition.getAltitude() < this->m_Boundary._floor) {
        MaceLog::Emergency("ABORT DUE TO ALTITUDE VIOLATION: ");
        MaceLog::Emergency("Altitude: " + std::to_string(currentPosition.getAltitude()));
        return AI_PROCEDURAL_COMMANDS::ABORT;
    }

    if(std::abs(roll_deg) > this->m_Criteria._stopCriteria.get_dRoll() || // Abort due to excessive roll
       std::abs(pitch_deg) > this->m_Criteria._stopCriteria.get_dPitch() ) // Abort due to excessive pitch
    {
        MaceLog::Emergency("ABORT DUE TO EXCESSIVE MANEUVER: ");
        MaceLog::Emergency("Roll: " + std::to_string(roll_deg) + " / Pitch: " + std::to_string(pitch_deg));

        return AI_PROCEDURAL_COMMANDS::ABORT;
    }

    // TODO-PAT: Check yaw?? Is that cool?

    if(elapsedTestDuration > m_Criteria.getMaxTestDuration()) {
        MaceLog::Green("STOP DUE TO TIME LIMIT");
        return AI_PROCEDURAL_COMMANDS::STOP;
    }

    return AI_PROCEDURAL_COMMANDS::RELEASE;
}

} // end of namespace DataGenericItem
