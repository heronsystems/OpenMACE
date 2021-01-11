#include "virtual_target_controller.h"

VirtualTargetController::VirtualTargetController(const std::string &agentID, const unsigned int &timeout)
{
    UNUSED(agentID);
    UNUSED(timeout);
}

void VirtualTargetController::updateAgentState(const VehicleState_Cartesian3D &state)
{
    if(m_VehicleState.set(state))
    {
        updateTargetProjection();
    }
}

void VirtualTargetController::updateOriginReference(const pose::GeodeticPosition_3D &position)
{
    pose::GeodeticPosition_3D modOrigin = position;
    modOrigin.setAltitude(0.0);
    m_Origin.set(modOrigin);
}

void VirtualTargetController::registerCurrentTarget(const pose::CartesianPosition_3D &commandTarget, const TimeoutMode &mode)
{
    UNUSED(mode);

    //std::cout<<"The target that had been sent is: "<<commandTarget<<std::endl;

    if(m_CurrentTarget.set(commandTarget))
    {
        updateTargetProjection();
    }

}

void VirtualTargetController::invalidateCurrentTarget()
{
    pose::CartesianPosition_3D newTarget;
    newTarget.invalidatePositionObject();
    m_CurrentTarget.set(newTarget);
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

    pose::CartesianPosition_3D current = m_VehicleState.get().m_Position;
    pose::DynamicsAid::LocalPositionToGlobal(&origin, &actualTarget_Car, &actualTarget_Geo);

    //Find the polar bearing to the "ideal" target location
    double polarBearing = current.polarBearingTo(&actualTarget_Car);
    polarBearing = math::wrapTo2Pi(polarBearing);

    //Project a virtual target point at some great distance beyond the target
    pose::CartesianPosition_3D virtualTarget_Car;
    current.newPositionFromPolar(&virtualTarget_Car, 100, polarBearing);
    virtualTarget_Car.setZPosition(actualTarget_Car.getZPosition());

    pose::DynamicsAid::LocalPositionToGlobal(&origin, &virtualTarget_Car, &virtualTarget_Geo);
    
    //Push the data to the vehicle core process
    if(m_VirtualTarget_Geo.set(virtualTarget_Geo))
    {
        callGeodeticTargetCallback(virtualTarget_Geo, actualTarget_Geo);
    }

}

