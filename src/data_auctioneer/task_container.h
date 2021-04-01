#ifndef TASK_CONTAINER_H
#define TASK_CONTAINER_H

#include <memory>

#include "data_tasks/task_key.h"
#include "data_tasks/task_descriptor.h"
#include "bids/bid_container.h"

/*!
 * \brief The TaskContainer class contains general information available about a task.
 * \details All task containers have a task key uniquely identifying the task.
 *
 * The container has a TaskDescriptor, if available, along with a BidContainer,
 * and TaskDynamicState.
 */
class TaskContainer
{
public:
    TaskContainer(uint64_t agentID, const TaskKey &taskKey, bool addressable, int priority = 0);

    const TaskKey &getTaskKey() const;

    const std::shared_ptr<TaskDescriptor> &getDescriptor() const;
    void setDescriptor(const std::shared_ptr<TaskDescriptor> &descriptor);

    BidContainer &getBidContainer();
    const BidContainer &getBidContainer() const;

    int getPriority() const;
    void setPriority(int priority);

    bool getConsensusReached() const;
    void setConsensusReached(bool consensusReached);

    bool getAddressable() const;

private:
    TaskKey                         m_taskKey;
    std::shared_ptr<TaskDescriptor> m_descriptor;
    BidContainer                    m_bidContainer;
    int                             m_priority;
    bool                            m_consensusReached = false;
    bool                            m_addressable;
};

#endif // TASK_CONTAINER_H
