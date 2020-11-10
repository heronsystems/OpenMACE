#include "tracking_simplified.h"
#include "math/frame_tf.h"


TrackingSimplified::TrackingSimplified(const std::string &agentParamsINIPath, const std::string &environmentINIPath, const std::string &scenarioINIPath):
    TrajectoryManagement_Base(agentParamsINIPath, environmentINIPath, scenarioINIPath)
{
    m_CompleteAgentState.AddNotifier(this, [this] {
        checkTrajectoryProgression(m_CompleteAgentState.get());
    });

    m_TrajectoryQueue.addLambda_QueueUpdate(this,[this]()
    {
        reassessForTrajectoryQueue();
    });
}

TrackingSimplified::TrackingSimplified():
    TrajectoryManagement_Base()
{
    m_CompleteAgentState.AddNotifier(this, [this] {
        checkTrajectoryProgression(m_CompleteAgentState.get());
    });

    m_TrajectoryQueue.addLambda_QueueUpdate(this,[this]()
    {
        reassessForTrajectoryQueue();
    });
}

TrackingSimplified::~TrackingSimplified()
{

}

void TrackingSimplified::reassessForTrajectoryQueue()
{
    TrajectoryPoint targetState;
    
    if(!m_TrajectoryQueue.retrieveLeadingItem(targetState)) //there is nothing valid to do
    {
        m_TargetPosition.invalidatePositionObject();    
        return;
    }

    m_TargetPosition.updateFromDataVector(targetState._position);
    m_TargetRotation.setQuaternion(targetState._orientation);

    checkTrajectoryProgression(m_CompleteAgentState.get());
}

void TrackingSimplified::checkTrajectoryProgression(const VehicleState_Cartesian3D &currentState)
{
    if (m_TargetPosition.isAnyPositionValid()) //there is something valid to do
    {
        double targetDistance = currentState.m_Position.distanceBetween2D(&m_TargetPosition);
        double polarTargetBearing = currentState.m_Position.polarBearingTo(&m_TargetPosition);
        Eigen::Quaterniond NEDorientation = currentState.m_Rotation.getQuaternion();
        Eigen::Quaterniond ENUorientation = ftf::transform_orientation_aircraft_baselink(
            ftf::transform_orientation_ned_enu(NEDorientation));
        double yawOrientation_ENU = ftf::quaternion_get_yaw(ENUorientation);
        double yawWrapped = math::wrapToPi(yawOrientation_ENU);
        double orientationDifference = fabs(yawWrapped - polarTargetBearing);
        if((orientationDifference >= M_PI_2) && (targetDistance <= 2 * m_AgentParams.getTurningRadius()))
        {
            std::cout<<"!!!!!!!!!!!The target was considered behind us!!!!!!!!!!!"<<std::endl;
            std::cout<<"Distance: "<<targetDistance<<" Bearing: "<<polarTargetBearing<<" Aircraft Orientation: "<<yawOrientation_ENU<<" Wrapped Orientation: "<<yawWrapped<<" Diffeerence: "<<orientationDifference<<std::endl;
        }
        
        if ((targetDistance <= m_AgentParams.getAcceptanceCriteria_Tight()) || ((orientationDifference > M_PI_2) && (targetDistance <= 2 * m_AgentParams.getTurningRadius()))) //we are within the acceptance criteria
        {
            m_TrajectoryQueue.updateQueue();
            reassessForTrajectoryQueue();
        }
    }
    on_TargetUpdate(m_TargetPosition);    
}
