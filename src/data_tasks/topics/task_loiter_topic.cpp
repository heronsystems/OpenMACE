#include "task_loiter_topic.h"

const char g_taskLoiterName[] = "task_loiter";
const MaceCore::TopicComponentStructure g_taskLoiterStructure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<std::shared_ptr<AbstractTaskLoiterDescriptor>>("Loiter Task");
    structure.AddTerminal<bool>("Responding");

    return structure;
}();

/*!
 * \brief Generates a datagram from underlying data
 * \return Generated datagram
 */
MaceCore::TopicDatagram TaskLoiterTopic::GenerateDatagram() const
{
    MaceCore::TopicDatagram datagram;

    datagram.AddTerminal<std::shared_ptr<AbstractTaskLoiterDescriptor>>("Loiter Task", m_descriptor);
    datagram.AddTerminal<bool>("Responding", m_requesting);

    return datagram;
}

/*!
 * \brief Creates underyling data from a datagram
 * \param datagram Datagram to create from
 */
void TaskLoiterTopic::CreateFromDatagram(const MaceCore::TopicDatagram &datagram)
{
    m_descriptor = datagram.GetTerminal<std::shared_ptr<AbstractTaskLoiterDescriptor>>("Loiter Task");
    m_requesting = datagram.GetTerminal<bool>("Responding");
}

/*!
 * \brief Returns the descriptor
 * \return Task descriptor
 */
const std::shared_ptr<AbstractTaskLoiterDescriptor> &TaskLoiterTopic::getDescriptor() const
{
    return m_descriptor;
}

/*!
 * \brief Sets the descriptor
 * \param descriptor Task descriptor
 */
void TaskLoiterTopic::setDescriptor(const std::shared_ptr<AbstractTaskLoiterDescriptor> &descriptor)
{
    m_descriptor = descriptor;
}

/*!
 * \brief Whether the topic is a response
 * \return Whether the topic is a response
 */
bool TaskLoiterTopic::getResponding() const
{
    return m_requesting;
}

/*!
 * \brief Sets whether the topic is a response
 * \param responding Whether the topic is a response
 */
void TaskLoiterTopic::setResponding(bool requesting)
{
    m_requesting = requesting;
}
