#include "state_data_mavlink.h"

StateData_MAVLINK::StateData_MAVLINK(const std::string &ID):
    _agentID(ID)
{
    vehicleGlobalPosition.AddNotifier(this, [this] {
        callbackUpdateVehicleState();
    });
    vehicleLocalPosition.AddNotifier(this, [this] {
        callbackUpdateVehicleState();
    });
    vehicleLocalVelocity.AddNotifier(this, [this] {
        callbackUpdateVehicleState();
    });
    vehicleAttitude.AddNotifier(this, [this] {
        callbackUpdateVehicleState();
    });
}

std::vector<std::shared_ptr<Data::ITopicComponentDataObject>> StateData_MAVLINK::GetTopicData()
{
    std::vector<std::shared_ptr<Data::ITopicComponentDataObject>> rtnVector;
    return rtnVector;
}

void StateData_MAVLINK::callbackUpdateVehicleState()
{
    VehicleState_Geodetic3D geoState; VehicleState_Cartesian3D localState;
    geoState.m_Position = vehicleGlobalPosition.get();
    geoState.m_Rotation = vehicleAttitude.get();
    geoState.m_Velocity = vehicleLocalVelocity.get();
    geoState.m_RotationalVelocity = vehicleRotationalVelocity.get();

    localState.m_Position = vehicleLocalPosition.get();
    localState.m_Rotation = vehicleAttitude.get();
    localState.m_Velocity = vehicleLocalVelocity.get();
    localState.m_RotationalVelocity = vehicleRotationalVelocity.get();

    m_CompleteVehicleGeoState.set(geoState);
    m_CompleteVehicleCarState.set(localState);
}

std::string StateData_MAVLINK::getAgentID() const
{
    return _agentID;
}

Data::EnvironmentTime StateData_MAVLINK::whenWasLastUpdate() const
{
    return m_CompleteVehicleGeoState.get().m_UpdateTime;
}
