#include "task_container.h"

/*!
 * \brief Constructor
 * \param taskKey Task key
 * \param addressable Whether the task is addressable
 * \param priority Priority
 */
TaskContainer::TaskContainer(uint64_t agentID, const TaskKey &taskKey, bool addressable, int priority) :
    m_taskKey(taskKey),
    m_bidContainer(agentID, taskKey),
    m_addressable(addressable),
    m_priority(priority)

{
}

/*!
 * \brief Gets the task key
 * \return Task key
 */
const TaskKey &TaskContainer::getTaskKey() const
{
    return m_taskKey;
}

/*!
 * \brief Gets the task descriptor
 * \return Task descriptor
 */
const std::shared_ptr<TaskDescriptor> &TaskContainer::getDescriptor() const
{
    return m_descriptor;
}

/*!
 * \brief Sets the task descriptor
 * \param descriptor Task descriptor
 */
void TaskContainer::setDescriptor(const std::shared_ptr<TaskDescriptor> &descriptor)
{
    m_descriptor = descriptor;
}

/*!
 * \brief Gets the bid container
 * \return Bid containerr
 */
BidContainer &TaskContainer::getBidContainer()
{
    return m_bidContainer;
}

/*!
 * \brief Gets the bid container
 * \overload
 * \return Bid container
 */
const BidContainer &TaskContainer::getBidContainer() const
{
    return m_bidContainer;
}

/*!
 * \brief Gets the task priority
 * \return Priority
 */
int TaskContainer::getPriority() const
{
    return m_priority;
}


/*!
 * \brief Sets the task priority
 * \param priority Priority
 */
void TaskContainer::setPriority(int priority)
{
    m_priority = priority;
}

/*!
 * \brief Gets whether consensus was reached on the task
 * \return Consensus
 */
bool TaskContainer::getConsensusReached() const
{
    return m_consensusReached;
}


/*!
 * \brief Sets whether consensus was reached on the task
 * \param consensusReached Consensus
 */
void TaskContainer::setConsensusReached(bool consensusReached)
{
    m_consensusReached = consensusReached;
}

/*!
 * \brief Gets whether the task is addressable
 * \return Addressability of the task
 */
bool TaskContainer::getAddressable() const
{
    return m_addressable;
}

