#include "vehicle_state.h"

VehicleState::VehicleState()
{

}

const DataState::StateLocalPosition &VehicleState::getPosition() const
{
    return m_position;
}

void VehicleState::setPosition(const DataState::StateLocalPosition &position)
{
    m_position = position;
    m_positionValid = true;
}

bool VehicleState::hasPosition() const
{
    return m_positionValid;
}

const DataState::StateLocalVelocity &VehicleState::getVelocity() const
{
    return m_velocity;
}

void VehicleState::setVelocity(const DataState::StateLocalVelocity &velocity)
{
    m_velocity = velocity;
}

bool VehicleState::hasVelocity() const
{
    return m_velocityValid;
}

const DataState::StateAttitude &VehicleState::getAttitude() const
{
    return m_attitude;
}

void VehicleState::setAttitude(const DataState::StateAttitude &attitude)
{
    m_attitude = attitude;
    m_attitudeValid = true;
}

bool VehicleState::hasAttitude() const
{
    return m_attitudeValid;
}

const DataGenericItem::DataGenericItem_Battery &VehicleState::getBattery() const
{
    return m_battery;
}

void VehicleState::setBattery(const DataGenericItem::DataGenericItem_Battery &battery)
{
    m_battery = battery;
    m_batteryValid = true;
}

bool VehicleState::hasBattery() const
{
    return m_batteryValid;
}

const DataGenericItem::DataGenericItem_SystemArm &VehicleState::getSystemArm() const
{
    return m_systemArm;
}

void VehicleState::setSystemArm(const DataGenericItem::DataGenericItem_SystemArm &systemArm)
{
    m_systemArm = systemArm;
    m_systemArmValid = true;
}

bool VehicleState::hasSystemArm()
{
    return m_systemArmValid;
}

const Data::EnvironmentTime &VehicleState::getTimestamp() const
{
    return m_timestamp;
}

void VehicleState::setTimestamp(const Data::EnvironmentTime &timestamp)
{
    m_timestamp = timestamp;
    m_timestampValid = true;
}

bool VehicleState::hasTimestamp() const
{
    return m_timestampValid;
}

