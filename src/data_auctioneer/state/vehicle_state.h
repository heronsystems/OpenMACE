#ifndef STATEVEHICLE_STATE_H
#define STATEVEHICLE_STATE_H

#include "data/environment_time.h"
#include "data_generic_state_item/state_local_position.h"
#include "data_generic_state_item/state_local_velocity.h"
#include "data_generic_state_item/state_attitude.h"
#include "data_generic_item/data_generic_item_battery.h"
#include "data_generic_item/data_generic_item_system_arm.h"

#include "common/class_forward.h"

MACE_CLASS_FORWARD(VehicleState);

class VehicleState
{
public:
    VehicleState();

    const DataState::StateLocalPosition  &getPosition() const;
    void setPosition(const DataState::StateLocalPosition  &position);
    bool hasPosition() const;

    const DataState::StateLocalVelocity &getVelocity() const;
    void setVelocity(const DataState::StateLocalVelocity &velocity);
    bool hasVelocity() const;

    const DataState::StateAttitude &getAttitude() const;
    void setAttitude(const DataState::StateAttitude &attitude);
    bool hasAttitude() const;

    const DataGenericItem::DataGenericItem_Battery &getBattery() const;
    void setBattery(const DataGenericItem::DataGenericItem_Battery &battery);
    bool hasBattery() const;

    const DataGenericItem::DataGenericItem_SystemArm &getSystemArm() const;
    void setSystemArm(const DataGenericItem::DataGenericItem_SystemArm &systemArm);
    bool hasSystemArm();

    const Data::EnvironmentTime &getTimestamp() const;
    void setTimestamp(const Data::EnvironmentTime &timestamp);
    bool hasTimestamp() const;

private:
    DataState::StateLocalPosition               m_position;
    DataState::StateLocalVelocity               m_velocity;
    DataState::StateAttitude                    m_attitude;
    DataGenericItem::DataGenericItem_Battery    m_battery;
    DataGenericItem::DataGenericItem_SystemArm  m_systemArm;
    Data::EnvironmentTime                       m_timestamp;

    bool m_positionValid = false;
    bool m_velocityValid = false;
    bool m_attitudeValid = false;
    bool m_batteryValid = false;
    bool m_systemArmValid = false;
    bool m_timestampValid = false;
};

#endif // STATEVEHICLE_STATE_H
