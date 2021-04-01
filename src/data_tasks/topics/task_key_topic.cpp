#include "task_key_topic.h"

const char g_taskKeyName[] = "task_key";
const MaceCore::TopicComponentStructure g_taskKeyStructure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<TaskKey>("Task Key");
    structure.AddTerminal<bool>("Requesting");
    return structure;
}();

/*!
 * \brief Generates a datagram from underlying data
 * \return Generated datagram
 */
MaceCore::TopicDatagram TaskKeyTopic::GenerateDatagram() const
{
    MaceCore::TopicDatagram datagram;

    datagram.AddTerminal<TaskKey>("Task Key", m_key);
    datagram.AddTerminal<bool>("Requesting", m_requestingDescriptor);

    return datagram;
}

/*!
 * \brief Creates underyling data from a datagram
 * \param datagram Datagram to create from
 */
void TaskKeyTopic::CreateFromDatagram(const MaceCore::TopicDatagram &datagram)
{
    m_key = datagram.GetTerminal<TaskKey>("Task Key");
    m_requestingDescriptor = datagram.GetTerminal<bool>("Requesting");
}

/*!
 * \brief Returns the task key
 * \return Task key
 */
const TaskKey &TaskKeyTopic::getKey() const
{
    return m_key;
}

/*!
 * \brief Sets the task key
 * \param key Task key
 */
void TaskKeyTopic::setKey(const TaskKey &key)
{
    m_key = key;
}

/*!
 * \brief Whether the descriptor is being requested
 * \return Whether the descriptor is being requested
 */
bool TaskKeyTopic::getRequestingDescriptor() const
{
    return m_requestingDescriptor;
}

/*!
 * \brief Sets whether the descriptor is being requested
 * \param requestingDescriptor Whether the descriptor is being requested
 */
void TaskKeyTopic::setRequestingDescriptor(bool requestingDescriptor)
{
    m_requestingDescriptor = requestingDescriptor;
}
