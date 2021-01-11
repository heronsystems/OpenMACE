#include "tracking_simplified.h"
#include "base/math/frame_tf.h"

TrackingSimplified::TrackingSimplified(const std::string &agentParamsINIPath):
    TrajectoryManagement_Base(agentParamsINIPath)
{
    m_CompleteAgentState.AddNotifier(this, [this] {
        if(!m_TargetPosition.get().hasTranslationalComponentBeenSet())
            return;

        if(checkTrajectoryProgression(m_CompleteAgentState.get()))
            reassessForTrajectoryQueue();
    });

    m_TrajectoryQueue.addLambda_QueueUpdate(this,[this]()
    {
        TrajectoryPoint targetState;
        m_TrajectoryQueue.retrieveLeadingItem(targetState);
        if(targetState != m_TargetPoint)
        {
            m_TargetPoint = targetState;
            pose::CartesianPosition_3D currentTarget; currentTarget.updateFromDataVector(targetState._position);
            m_TargetPosition.set(currentTarget);
            m_TargetRotation.setQuaternion(targetState._orientation);
            reassessForTrajectoryQueue();
        }
    });
}

TrackingSimplified::TrackingSimplified():
    TrajectoryManagement_Base()
{
    m_CompleteAgentState.AddNotifier(this, [this] {
        if(!m_TargetPosition.get().hasTranslationalComponentBeenSet())
            return;

        if(checkTrajectoryProgression(m_CompleteAgentState.get()))
            reassessForTrajectoryQueue();
    });

    m_TrajectoryQueue.addLambda_QueueUpdate(this,[this]()
    {
        TrajectoryPoint targetState;
        m_TrajectoryQueue.retrieveLeadingItem(targetState);
        if(targetState != m_TargetPoint)
        {
            m_TargetPoint = targetState;
            pose::CartesianPosition_3D currentTarget; currentTarget.updateFromDataVector(targetState._position);
            m_TargetPosition.set(currentTarget);
            m_TargetRotation.setQuaternion(targetState._orientation);
            reassessForTrajectoryQueue();
        }
    });
}

TrackingSimplified::~TrackingSimplified()
{

}

void TrackingSimplified::reassessForTrajectoryQueue()
{
    if(m_TrajectoryQueue.isQueueEmpty())
        return;

    TrajectoryPoint targetState;
    pose::CartesianPosition_3D currentTarget;
    bool targetCriteriaSatisfied = true;

    while(targetCriteriaSatisfied)
    {
        if(!m_TrajectoryQueue.updateQueue(targetState))
        {
            std::cout<<"Either the queue is empty, or the target is not valid"<<std::endl;
            break;
        }
        currentTarget.updateFromDataVector(targetState._position);
        m_TargetPosition.set(currentTarget);
        m_TargetRotation.setQuaternion(targetState._orientation);

        targetCriteriaSatisfied = checkTrajectoryProgression(m_CompleteAgentState.get());
    } //end of while loop


    on_TargetUpdate(currentTarget);
}

bool TrackingSimplified::checkTrajectoryProgression(const VehicleState_Cartesian3D &currentState)
{
    bool targetSatisifed = false;
    pose::CartesianPosition_3D currentTarget = m_TargetPosition.get();

    if(!currentTarget.isAnyPositionValid()) //the reason for this is if the position isnt valid, there is nothing to validate against. Move on
        targetSatisifed = true;
    else
    {
        double targetDistance = currentState.m_Position.distanceBetween2D(&currentTarget);
        double polarTargetBearing = currentState.m_Position.polarBearingTo(&currentTarget);
        polarTargetBearing = math::wrapTo2Pi(polarTargetBearing);

        Eigen::Quaterniond NEDorientation = currentState.m_Rotation.getQuaternion();
        Eigen::Quaterniond ENUorientation = ftf::transform_orientation_aircraft_baselink(
            ftf::transform_orientation_ned_enu(NEDorientation));
        double yawOrientation_ENU = ftf::quaternion_get_yaw(ENUorientation);
        double yawWrapped = math::wrapTo2Pi(yawOrientation_ENU);

        double orientationDifference = math::angDistance(yawWrapped,polarTargetBearing);
        if(targetDistance <= m_AgentParams.getAcceptanceCriteria_Tight())
        {
            targetSatisifed = true;
        }
        else if((fabs(orientationDifference) >= math::convertDegreesToRadians(30)) && (targetDistance <= m_AgentParams.getTurningRadius()))
        {
            targetSatisifed = true;
        }

    } //end of the else statement

    return targetSatisifed;
}
