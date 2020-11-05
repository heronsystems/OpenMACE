#include "mavlink_vehicle_object.h"


MavlinkVehicleObject::MavlinkVehicleObject(CommsMAVLINK *commsObj, const MaceCore::ModuleCharacteristic &module, const int &m_MavlinkID):
    mavlinkID(m_MavlinkID), m_module(module), m_CB(nullptr)
{
    this->commsLink = commsObj;

    controllerQueue = new TransmitQueue(2000, 3);
    state = new StateData_MAVLINK();
    environment = new EnvironmentData_MAVLINK();
    status = new StatusData_MAVLINK();
    mission = new MissionData_MAVLINK();

    Data::EnvironmentTime::CurrentTime(Data::Devices::SYSTEMCLOCK, prevAttitude);
    Data::EnvironmentTime::CurrentTime(Data::Devices::SYSTEMCLOCK, prevPosition);

}

MavlinkVehicleObject::~MavlinkVehicleObject()
{ 
    delete state;
    delete environment;
    delete status;
    delete mission;
}

int MavlinkVehicleObject::getMAVLINKID() const
{
    return this->mavlinkID;
}


MaceCore::ModuleCharacteristic MavlinkVehicleObject::getModule() const
{
    return m_module;
}

CommsMAVLINK* MavlinkVehicleObject::getCommsObject() const
{
    return this->commsLink;
}

bool MavlinkVehicleObject::handleMAVLINKMessage(const mavlink_message_t &msg)
{
    return m_ControllersCollection.Receive(msg, msg.sysid);
}
