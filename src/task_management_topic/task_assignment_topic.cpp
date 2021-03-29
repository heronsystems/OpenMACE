#include "task_assignment_topic.h"

const char g_taskAssignmentName[] = "task_assignment";
const MaceCore::TopicComponentStructure g_taskAssignmentStructure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<std::vector<TaskDescriptorPtr>>("AssignedTasks");
    structure.AddTerminal<int>("VehicleID");
    return structure;
}();

/*!
 * \brief Generates a datagram from underlying data
 * \return Generated datagram
 */
MaceCore::TopicDatagram TaskAssignmentTopic::GenerateDatagram() const
{
    MaceCore::TopicDatagram datagram;

    datagram.AddTerminal<std::vector<TaskDescriptorPtr>>("AssignedTasks", m_assignedTasks);
    datagram.AddTerminal<int>("VehicleID", m_vehicleID);

    return datagram;
}

/*!
 * \brief Creates underyling data from a datagram
 * \param datagram Datagram to create from
 */
void TaskAssignmentTopic::CreateFromDatagram(const MaceCore::TopicDatagram &datagram)
{
    m_assignedTasks = datagram.GetTerminal<std::vector<TaskDescriptorPtr>>("AssignedTasks");
    m_vehicleID = datagram.GetTerminal<int>("VehicleID");
}

const std::vector<TaskDescriptorPtr> &TaskAssignmentTopic::getAssignedTasks() const
{
    return m_assignedTasks;
}

void TaskAssignmentTopic::setAssignedTasks(const std::vector<TaskDescriptorPtr> &assignedTasks)
{
    m_assignedTasks = assignedTasks;
}

int TaskAssignmentTopic::getVehicleID() const
{
    return m_vehicleID;
}

void TaskAssignmentTopic::setVehicleID(int vehicleID)
{
    m_vehicleID = vehicleID;
}
