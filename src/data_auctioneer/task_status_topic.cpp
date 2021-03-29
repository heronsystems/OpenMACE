#include "task_status_topic.h"

const char g_taskStatusName[] = "task_complete";
const MaceCore::TopicComponentStructure g_taskStatusStructure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<TaskKey>("Task Key");
    structure.AddTerminal<uint64_t>("Agent ID");
    structure.AddTerminal<TaskStatusType>("Type");
    structure.AddTerminal<TaskStatusTopic::DataArray>("Data");

    return structure;
}();

/*!
 * \brief Generates a datagram from underlying data
 * \return Generated datagram
 */
MaceCore::TopicDatagram TaskStatusTopic::GenerateDatagram() const
{
    MaceCore::TopicDatagram datagram;

    datagram.AddTerminal<TaskKey>("Task Key", m_taskKey);
    datagram.AddTerminal<uint64_t>("Agent ID", m_agentID);
    datagram.AddTerminal<TaskStatusType>("Type", m_statusType);
    datagram.AddTerminal<DataArray>("Data", m_data);
    return datagram;
}


/*!
 * \brief Creates underyling data from a datagram
 * \param datagram Datagram to create from
 */
void TaskStatusTopic::CreateFromDatagram(const MaceCore::TopicDatagram &datagram)
{
    m_taskKey = datagram.GetTerminal<TaskKey>("Task Key");
    m_agentID = datagram.GetTerminal<uint64_t>("Agent ID");
    m_statusType = datagram.GetTerminal<TaskStatusType>("Type");
    m_data = datagram.GetTerminal<DataArray>("Data");
}

/*!
 * \return Task key
 */
TaskKey TaskStatusTopic::getTaskKey() const
{
    return m_taskKey;
}

/*!
 * \param taskKey Task key
 */
void TaskStatusTopic::setTaskKey(const TaskKey &taskKey)
{
    m_taskKey = taskKey;
}

/*!
 * \return Agent ID
 */
uint64_t TaskStatusTopic::getAgentID() const
{
    return m_agentID;
}

/*!
 * \param agentID Agent ID
 */
void TaskStatusTopic::setAgentID(const uint64_t &agentID)
{
    m_agentID = agentID;
}

/*!
 * \return Type of status update
 */
TaskStatusType TaskStatusTopic::getStatusType() const
{
    return m_statusType;
}

/*!
 * \brief Sets the type of status update
 */
void TaskStatusTopic::setStatusType(const TaskStatusType &statusType)
{
    m_statusType = statusType;
}

/*!
 * \return Retrieves the data for a status update
 */
TaskStatusTopic::DataArray TaskStatusTopic::getData() const
{
    return m_data;
}

/*!
 * \brief Sets the data for a status update
 */
void TaskStatusTopic::setData(const DataArray &data)
{
    m_data = data;
}

int TaskStatusTopic::GetTypeDataSize() const
{
    switch (m_statusType)
    {
        case TaskStatusType::Started:
        case TaskStatusType::Aborted:
        case TaskStatusType::Completed:
        default:
            return 0;
    }
}


/*!
 * \brief Default constructor
 */
TaskStatusTopic::DataArray::DataArray()
{
    memset(&data, 0, sizeof(data));
}

/*!
 * \brief Constructor using input data array
 * \param dataSource Source data array
 */
TaskStatusTopic::DataArray::DataArray(const uint8_t (&dataSource)[64])
{
    memcpy(&data, &dataSource, sizeof(data));
}

/*!
 * \brief Copy constructor
 * \param other Other data array
 */
TaskStatusTopic::DataArray::DataArray(const TaskStatusTopic::DataArray &other)
{
    memcpy(&data, &other.data, sizeof(data));
}
