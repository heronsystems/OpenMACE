#include "vehicle_task_status_topic.h"
const char g_vehicleTaskStatusName[] = "task_vehicle_status";
const MaceCore::TopicComponentStructure g_vehicleTaskStatusStructure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<int>("VehicleID");
    structure.AddTerminal<TaskKey>("TaskKey");
    structure.AddTerminal<TaskStatusType>("Status");
    structure.AddTerminal<bool>("Success");
    return structure;
}();

/*!
 * \brief Generates a datagram from underlying data
 * \return Generated datagram
 */
MaceCore::TopicDatagram VehicleTaskStatusTopic::GenerateDatagram() const
{
    MaceCore::TopicDatagram datagram;

    datagram.AddTerminal<int>("VehicleID", m_vehicleID);
    datagram.AddTerminal<TaskKey>("TaskKey", m_taskKey);
    datagram.AddTerminal<TaskStatusType>("Status", m_statusType);
    datagram.AddTerminal<bool>("Success", m_success);

    return datagram;
}

/*!
 * \brief Creates underyling data from a datagram
 * \param datagram Datagram to create from
 */
void VehicleTaskStatusTopic::CreateFromDatagram(const MaceCore::TopicDatagram &datagram)
{
    m_vehicleID = datagram.GetTerminal<int>("VehicleID");
    m_taskKey = datagram.GetTerminal<TaskKey>("TaskKey");
    m_statusType = datagram.GetTerminal<TaskStatusType>("Status");
    m_success = datagram.GetTerminal<bool>("Success");
}

int VehicleTaskStatusTopic::getVehicleID() const
{
    return m_vehicleID;
}

void VehicleTaskStatusTopic::setVehicleID(int vehicleID)
{
    m_vehicleID = vehicleID;
}

const TaskKey &VehicleTaskStatusTopic::getTaskKey() const
{
    return m_taskKey;
}

void VehicleTaskStatusTopic::setTaskKey(const TaskKey &taskKey)
{
    m_taskKey = taskKey;
}

TaskStatusType VehicleTaskStatusTopic::getStatusType() const
{
    return m_statusType;
}

void VehicleTaskStatusTopic::setStatusType(const TaskStatusType &statusType)
{
    m_statusType = statusType;
}

bool VehicleTaskStatusTopic::getSuccess() const
{
    return m_success;
}

void VehicleTaskStatusTopic::setSuccess(bool success)
{
    m_success = success;
}
