#include "virtual_target_controller.h"

VirtualTargetController::VirtualTargetController(const std::string &agentID, const unsigned int &timeout)
{
    m_VehicleState.AddNotifier(this,[this]{
        updateTargetProjection();
    });
}

void VirtualTargetController::updateAgentParams(const AgentParams &params)
{
    m_AgentParams = params;
}

void VirtualTargetController::updateAgentState(const VehicleState_Cartesian3D &state)
{
    m_VehicleState.set(state);
}

void VirtualTargetController::updateOriginReference(const pose::GeodeticPosition_3D &position)
{
    m_Origin.set(position);
}

void VirtualTargetController::registerCurrentTarget(const pose::CartesianPosition_3D &commandTarget, const TimeoutMode &mode)
{
    UNUSED(mode);

    m_CurrentTarget.set(commandTarget);
    
    updateTargetProjection();
}

void VirtualTargetController::updateTargetProjection()
{
    pose::CartesianPosition_3D actualTarget_Car = m_CurrentTarget.get();

    if (!actualTarget_Car.areAllPositionsValid())
    {
        return;
    }

    pose::GeodeticPosition_3D actualTarget_Geo, virtualTarget_Geo, origin;
    origin = m_Origin.get();
    pose::DynamicsAid::LocalPositionToGlobal(&origin, &actualTarget_Car, &actualTarget_Geo);

    pose::CartesianPosition_3D current = m_VehicleState.get().m_Position;
    //Find the polar bearing to the "ideal" target location
    double polarBearing = current.polarBearingTo(&actualTarget_Car);

    //Project a virtual target point at some great distance beyond the target
    pose::CartesianPosition_3D virtualTarget_Car;
    current.newPositionFromPolar(&virtualTarget_Car, 1000.0, polarBearing);
    virtualTarget_Car.setZPosition(actualTarget_Car.getZPosition());

    pose::DynamicsAid::LocalPositionToGlobal(&origin, &virtualTarget_Car, &virtualTarget_Geo);
    
    //Push the data to the vehicle core process
    callGeodeticTargetCallback(virtualTarget_Geo, actualTarget_Geo);
}