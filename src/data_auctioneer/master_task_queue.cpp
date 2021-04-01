#include "master_task_queue.h"

MasterTaskQueue::MasterTaskQueue()
{

}

void MasterTaskQueue::setAgentID(uint64_t agentID)
{
    m_agentID = agentID;
}

/*!
 * \brief Adds a task key.
 *
 * This function must be called before any other task data can be added.
 *
 * \param key The task key
 * \param addressable Whether the agent this object belongs to can address the task
 * \param priority Priority of the task
 * \return
 */
bool MasterTaskQueue::AddTaskKey(const TaskKey &key, bool addressable, int priority)
{
    if (m_taskData.find(key) != m_taskData.end())
        return false;

    if (addressable)
        m_availableTasks.AddTask(key, priority);

    m_taskData.insert({key, TaskContainer(m_agentID, key, addressable, priority)});

    return true;
}

/*!
 * \brief Stores a task descriptor
 * \param descriptor Task descriptor
 */
void MasterTaskQueue::AddTaskDescriptor(const std::shared_ptr<TaskDescriptor> &descriptor)
{
    const TaskKey &key = descriptor->getTaskKey();
    TaskContainer &taskContainer = m_taskData.at(key);
    taskContainer.setDescriptor(descriptor);
}

/*!
 * \brief Retrieives the task descriptor corresponding to a task
 * \param key Task key
 * \return The task descriptor, or a nullptr if the task descriptor is not available
 */
const std::shared_ptr<TaskDescriptor> &MasterTaskQueue::getTaskDescriptor(const TaskKey &key)
{
    const TaskContainer &taskContainer = m_taskData.at(key);
    return taskContainer.getDescriptor();
}

/*!
 * \brief Retrieves the tasks assigned to this agent
 * \return Assigned task queue
 */
const AssignmentTaskQueue &MasterTaskQueue::getAssignedTasks() const
{
    return m_assignedTasks;
}

/*!
 * \brief Retrieves the tasks awarded to any agent, including assignments without consensus
 * \return Awarded task queue
 */
const AwardedTaskQueue &MasterTaskQueue::getAwardedTasks() const
{
    return m_awardedTasks;
}

/*!
 * \brief Retrieves the tasks available to bid on, which are those addressable by the agent
 * and without consensus on any existing assignment
 *
 * \return Available task queue
 */
const AvailableTaskQueue &MasterTaskQueue::getAvailableTasks() const
{
    return m_availableTasks;
}

/*!
 * \brief Receives a bid bundle, and updates the agent's knowledge of who is winning a task accordingly.
 *
 * This function will save information about received bundles in order to keep track of which winning bids
 * become invalid due to another agent outbidding another for particular tasks.
 *
 * \param bundle Received bundle
 * \return Whether the bundle won at least 1 task
 */
bool MasterTaskQueue::AddBidBundle(const std::shared_ptr<BidBundle> &bundle)
{
    bool newLeaders = false;
    ScrubMessage scrub;
    uint64_t agentID;

    AwardedTaskParams awardedParams;

    auto bundleIterator = bundle->Begin();
    agentID = bundle->getAgentID();

    auto knownAgentBundlesIterator = m_knownAgentBids.find(agentID);
    if (knownAgentBundlesIterator == m_knownAgentBids.end())
        m_knownAgentBids.insert({agentID, std::multimap<Data::EnvironmentTime, BidDescriptor>()});

    while (bundleIterator != bundle->End())
    {
        const BidDescriptor &bidDescriptor = bundleIterator->descriptor;
        if (!ReceiveBid(agentID, bidDescriptor.getGenerationTime(), bidDescriptor, bundleIterator->finalState))
            break; // Was not a new leader

        newLeaders = true;

        ++bundleIterator;
    }
    return newLeaders;
}

bool MasterTaskQueue::ReceiveBid(uint64_t sender,
                                 const Data::EnvironmentTime &senderTimestamp,
                                 const BidDescriptor &bid,
                                 const VehicleStatePtr &finalState)
{
    bool newLeader;
    const TaskKey &key = bid.getTaskKey();
    TaskContainer &taskContainer = m_taskData.at(key);
    BidContainer &bidContainer = taskContainer.getBidContainer();

    ScrubMessage scrub;

    if (taskContainer.getConsensusReached())
        return false; // Consensus was previously reached on this task, so we consider it invalid

    bool hadPrevBid = bidContainer.getValid();

    BidDescriptor prevLeadingBid = bidContainer.getLeadingBid();

    BidContainer::ReceiveAction receiveAction = bidContainer.ReceiveBid(sender, senderTimestamp, bid, newLeader);

    if (!hadPrevBid && receiveAction == BidContainer::ReceiveAction::BroadcastLocal)
        receiveAction = BidContainer::ReceiveAction::BroadcastScrub;

    Data::EnvironmentTime timestamp;
    bidContainer.getAgentTime(m_agentID, timestamp);

    switch (receiveAction)
    {
        case BidContainer::ReceiveAction::BroadcastLocal:
            m_bidsToBroadcast.insert({prevLeadingBid, timestamp});
            break;
        case BidContainer::ReceiveAction::BroadcastReceived:
            m_bidsToBroadcast.insert({bid, timestamp});
            break;
        case BidContainer::ReceiveAction::BroadcastScrub:
            scrub.agentID = m_agentID;
            scrub.taskKey = key;
            scrub.timestamp = timestamp;
            m_scrubMessagesToBroadcast.insert(scrub);
            break;
        case BidContainer::ReceiveAction::None:
            break;
        default:
            throw std::runtime_error("Unknown receive action on bid receipt");
            break;
    }

    if (hadPrevBid && (newLeader || receiveAction == BidContainer::ReceiveAction::BroadcastScrub))
        InvalidateBids(prevLeadingBid);

    if (newLeader)
    {
        AwardedTaskParams awardedParams;
        awardedParams.taskKey = key;
        awardedParams.awardedAgentID = sender;
        awardedParams.consensusReached = false;

        // TODO Other awarded task paramaters are currently not set
        m_awardedTasks.erase(key);
        m_awardedTasks.insert({key, awardedParams});

        if (sender == m_agentID)
        {
            m_assignedTasks.AddTask(key, finalState);
            m_availableTasks.RemoveTask(key);
        }

        if (m_knownAgentBids.find(bid.getAgentID()) == m_knownAgentBids.end())
            m_knownAgentBids.insert({bid.getAgentID(), std::multimap<Data::EnvironmentTime, BidDescriptor>()});

        auto &agentBidsMap = m_knownAgentBids.at(bid.getAgentID());
        agentBidsMap.insert({bid.getGenerationTime(), bid});

        if (m_auctionStats)
            m_auctionStats->TrackAssignment(key, sender, bid.getUtility());
    }
    return newLeader;
}

void MasterTaskQueue::ReceiveScrubMessage(const ScrubMessage &scrub)
{
    const TaskKey &key = scrub.taskKey;
    TaskContainer &taskContainer = m_taskData.at(key);
    BidContainer &bidContainer = taskContainer.getBidContainer();

    bool hadPrevBid = bidContainer.getValid();
    BidDescriptor prevBid = bidContainer.getLeadingBid();



    BidContainer::ReceiveAction action = bidContainer.ReceiveScrub(scrub);

    Data::EnvironmentTime timestamp;
    bidContainer.getAgentTime(m_agentID, timestamp);

    switch (bidContainer.ReceiveScrub(scrub))
    {
        case BidContainer::ReceiveAction::None:
            break;
        case BidContainer::ReceiveAction::BroadcastLocal:
            m_bidsToBroadcast.insert({bidContainer.getLeadingBid(), timestamp});
            break;
        case BidContainer::ReceiveAction::BroadcastScrub:
        {
            ScrubMessage newScrub;
            newScrub.agentID = m_agentID;
            newScrub.taskKey = scrub.taskKey;
            newScrub.timestamp = timestamp;
            m_scrubMessagesToBroadcast.insert(newScrub);
            break;
        }
        case BidContainer::ReceiveAction::BroadcastReceived:
            throw std::runtime_error("Rebroadcast received action requested when receiving a scrub message");
            break;
        default:
            throw std::runtime_error("Unknown receive action on scrub message receipt");
            break;
    }

    if (hadPrevBid && !bidContainer.getValid())
        InvalidateBids(prevBid);
}

/*!
 * \brief Returns whether consensus on a task has been reached
 * \param key Task key
 * \return Consensus
 */
bool MasterTaskQueue::getConsensusReached(const TaskKey &key)
{
    TaskContainer &taskContainer = m_taskData.at(key);
    return taskContainer.getConsensusReached();
}

///*!
// * \brief Sets consensus as reached for a particular task
// * \param key Task key
// * \param winningBid Winning bid
// */
//void MasterTaskQueue::setConsensusReached(const TaskKey &key, const BidDescriptor &winningBid)
//{
//    TaskContainer &taskContainer = m_taskData.at(key);
//    BidContainer &bidContainer = taskContainer.getBidContainer();

//    if (bidContainer.getValid() && bidContainer.getLeadingBid() != winningBid)
//    {
//        std::shared_ptr<BidBundle> bundleFrom = bidContainer.getBundleFrom();
//        int bundleIndex = bidContainer.getBundleIndex();
//        InvalidateBids(bundleFrom, bundleIndex);
//        bidContainer.setLeadingBid(winningBid);
//    }

//    uint64_t agentID = winningBid.getAgentID();

//    auto awardedParamsIterator = m_awardedTasks.find(key);
//    if (awardedParamsIterator == m_awardedTasks.end())
//    {
//        AwardedTaskParams awardedParams;
//        awardedParams.awardedAgentID = agentID;
//        awardedParams.consensusReached = true;
//        m_awardedTasks.insert({key, awardedParams});
//    }
//    else
//    {
//        AwardedTaskParams awardedParams = m_awardedTasks.at(key);
//        awardedParams.awardedAgentID = agentID;
//        awardedParams.consensusReached = true;
//    }
//    taskContainer.setConsensusReached(true);

//    m_availableTasks.RemoveTask(key);

//    if (m_agentID == agentID)
//    {
//        int nonConsensusIndex = m_assignedTasks.getFirstNonConsensusIndex();
//        auto assignedTaskIterator = m_assignedTasks.Begin_NonConsensus();
//        while (assignedTaskIterator != m_assignedTasks.End())
//        {
//            if (getConsensusReached(*assignedTaskIterator))
//                ++nonConsensusIndex;

//            ++assignedTaskIterator;
//        }
//        m_assignedTasks.setFirstNonConsensusIndex(nonConsensusIndex);
//    }
//}

/*!
 * \brief Retrieves the priority of a task
 * \param key Task key
 * \return Priority
 */
int MasterTaskQueue::getPriority(const TaskKey &key)
{
    const TaskContainer &taskContainer = m_taskData.at(key);
    return taskContainer.getPriority();
}

/*!
 * \brief Sets the priority of a task
 * \param key Task key
 * \param priority Priority
 */
void MasterTaskQueue::setPriority(const TaskKey &key, int priority)
{
    TaskContainer &taskContainer = m_taskData.at(key);
    taskContainer.setPriority(priority);
}

/*!
 * \brief Marks a task as completed
 * \param key Task key
 * \param agentCompletingID Agent which completed the task
 * \return Whether the task complete message should be rebroadcast. This will be the case if an
 * agent's knowledge of the leader on a task does not match which agent completed it.
 */
bool MasterTaskQueue::setTaskCompleted(const TaskKey &key, uint64_t completingAgentID)
{
    bool rebroadcast = false;
    if (m_completedTasks.find(key) != m_completedTasks.end())
        return rebroadcast;
    if (m_abortedTasks.find(key) != m_abortedTasks.end())
        return rebroadcast;

    if (m_taskData.find(key) == m_taskData.end())
    {
        m_taskData.insert({key, TaskContainer(m_agentID, key, false)});
        m_completedTasks.insert({key,completingAgentID});
        return rebroadcast;
    }

    TaskContainer &taskContainer = m_taskData.at(key);

    taskContainer.setConsensusReached(true);

    if (HasLeadingBid(key) && m_startedTasks.find(key) == m_startedTasks.end())
    {
        AwardedTaskParams &awardedParams = m_awardedTasks.at(key);
        BidContainer &bidContainer = taskContainer.getBidContainer();

        const BidDescriptor &leadingBid = bidContainer.getLeadingBid();

        if (awardedParams.awardedAgentID == completingAgentID)
        {
            // Since the task is completed, we remove its corresponding bid from our belief of the completing agent's assignment queue
            auto &agentBidsMap = m_knownAgentBids.at(completingAgentID);
            auto it = m_knownAgentBids.at(completingAgentID).find(leadingBid.getGenerationTime());
            while (it != agentBidsMap.end() && it->second != leadingBid)
                ++it;

            if (it != agentBidsMap.end())
                agentBidsMap.erase(it);
        }
        else
        {
            // An agent we did not know had the leading bid completed the task.
            // The subsequent bids from the believed leader must be invalid.
            InvalidateBids(leadingBid);
            rebroadcast = true;
        }
    }
    else
    {
        // We never had a leader for this task.
        rebroadcast = true;
    }

    m_availableTasks.RemoveTask(key);
    m_awardedTasks.erase(key);

    taskContainer.setConsensusReached(true);

    m_completedTasks.insert({key, completingAgentID});
    m_startedTasks.erase(key);
    return rebroadcast;
}

/*!
 * \brief Returns whether the task was completed, and sets the agentID to the agent which completed the task
 * \param key Task key
 * \param agentID Set to the agent which completed the task. Valid if true is returned
 * \return Whether the task is known to have been completed
 */
bool MasterTaskQueue::getTaskCompleted(const TaskKey &key, uint64_t &agentID)
{
    auto it = m_completedTasks.find(key);
    bool completed = it != m_completedTasks.end();
    if (completed)
        agentID = it->second;

    return completed;
}

/*!
 * \brief Returns whether the task was completed
 * \overload
 * \param key Task key
 * \return Whether the task is known to have been completed
 */
bool MasterTaskQueue::getTaskCompleted(const TaskKey &key)
{
    return m_completedTasks.find(key) != m_completedTasks.end();
}

/*!
 * \brief Sets a task as having been started
 * \details This function assumes that the task was not previously completed. Users should call
 * getTaskCompleted() before calling this function.
 * \param key Task key
 * \param startingAgentID ID of agent starting the task
 * \return Whether the task started message should be re-broadcast. This occurs if the agent which started
 * the task is not the agent believed to have been the leader
 */
bool MasterTaskQueue::setTaskStarted(const TaskKey &key, uint64_t startingAgentID)
{
    bool rebroadcast = false;
    if (m_startedTasks.find(key) != m_startedTasks.end())
        return rebroadcast;
    if (m_abortedTasks.find(key) != m_abortedTasks.end())
        return rebroadcast;
    if (m_completedTasks.find(key) != m_completedTasks.end())
        return rebroadcast;

    if (m_taskData.find(key) == m_taskData.end())
    {
        m_taskData.insert({key, TaskContainer(m_agentID, key, false)});
        m_startedTasks.insert({key, startingAgentID});
        return rebroadcast; // First time learning about the task
    }

    TaskContainer &taskContainer = m_taskData.at(key);

    taskContainer.setConsensusReached(true);

    if (HasLeadingBid(key))
    {
        AwardedTaskParams &awardedParams = m_awardedTasks.at(key);
        BidContainer &bidContainer = taskContainer.getBidContainer();

        const BidDescriptor &leadingBid = bidContainer.getLeadingBid();

        if (awardedParams.awardedAgentID == startingAgentID)
        {
            // Since the task is started, we remove its corresponding bid from our belief of the completing agent's assignment queue
            auto &agentBidsMap = m_knownAgentBids.at(startingAgentID);
            auto it = m_knownAgentBids.at(startingAgentID).find(leadingBid.getGenerationTime());
            while (it != agentBidsMap.end() && it->second != leadingBid)
                ++it;

            if (it != agentBidsMap.end())
                agentBidsMap.erase(it);
        }
        else
        {
            // An agent we did not know had the leading bid started the task.
            // The subsequent bids from the believed leader must be invalid.
            InvalidateBids(leadingBid);
            rebroadcast = true;
        }
    }
    else
    {   // We never had a leader for this task.
        rebroadcast = true;
    }

    m_availableTasks.RemoveTask(key);
    m_awardedTasks.erase(key);

    taskContainer.setConsensusReached(true);

    m_startedTasks.insert({key, startingAgentID});
    return rebroadcast;
}


/*!
 * \brief Returns whether the task was started, and sets the agentID to the agent which started the task
 * \param key Task key
 * \param agentID Set to the agent which started the task. Valid if true is returned.
 * \return Whether the task is known to have been started, but not completed
 */
bool MasterTaskQueue::getTaskStarted(const TaskKey &key, uint64_t &agentID)
{
    auto it = m_startedTasks.find(key);
    bool started = it != m_startedTasks.end();
    if (started)
        agentID = it->second;

    return started;
}


/*!
 * \param key Task key
 * \return Whether the task has been started
 */
bool MasterTaskQueue::getTaskStarted(const TaskKey &key)
{
    return m_startedTasks.find(key) != m_startedTasks.cend();
}

/*!
 * \brief Sets a task as having been aborted by the given agent
 * \param key
 * \param abortingAgentID
 * \return Whether the message should be re-broadcast. This occurs if the aborting agent does
 * not match the agent who started the task, or if the task was not believed to have been started.
 */
bool MasterTaskQueue::setTaskAborted(const TaskKey &key, uint64_t abortingAgentID)
{
    bool rebroadcast = false;
    if (m_abortedTasks.find(key) != m_abortedTasks.end())
        return rebroadcast;
    if (m_completedTasks.find(key) != m_completedTasks.end())
        return rebroadcast;

    if (m_taskData.find(key) == m_taskData.end())
    {
        m_taskData.insert({key, TaskContainer(m_agentID, key, false)});
        m_abortedTasks.insert({key, abortingAgentID});
        return rebroadcast; // First time learning about the task
    }

    TaskContainer &taskContainer = m_taskData.at(key);

    taskContainer.setConsensusReached(true);

    if (HasLeadingBid(key) && m_startedTasks.find(key) == m_startedTasks.end())
    {
        AwardedTaskParams &awardedParams = m_awardedTasks.at(key);
        BidContainer &bidContainer = taskContainer.getBidContainer();

        const BidDescriptor &leadingBid = bidContainer.getLeadingBid();

        if (awardedParams.awardedAgentID == abortingAgentID)
        {
            // Since the task is started, we remove its corresponding bid from our belief of the completing agent's assignment queue
            auto &agentBidsMap = m_knownAgentBids.at(abortingAgentID);
            auto it = m_knownAgentBids.at(abortingAgentID).find(leadingBid.getGenerationTime());
            while (it != agentBidsMap.end() && it->second != leadingBid)
                ++it;

            if (it != agentBidsMap.end())
                agentBidsMap.erase(it);
        }
        else
        {
            // An agent we did not know had the leading bid started the task.
            // The subsequent bids from the believed leader must be invalid.
            InvalidateBids(leadingBid);
            rebroadcast = true;
        }
    }
    else
    {   // We never had a leader for this task.
        rebroadcast = true;
    }

    m_startedTasks.erase(key);

    // These occurred when the task was started, but only if we had actually received the start message
    m_availableTasks.RemoveTask(key);
    m_awardedTasks.erase(key);

    taskContainer.setConsensusReached(true);

    m_startedTasks.insert({key, abortingAgentID});
    return rebroadcast;
}

/*!
 * \brief Returns whether the task was aborted, and sets the agentID to the agent which aborted the task
 * \param key Task key
 * \param agentID Set to the agent which aborted the task. Valid if true is returned.
 * \return Whether the task is known to have been aborted, but not completed
 */
bool MasterTaskQueue::getTaskAborted(const TaskKey &key, uint64_t &agentID)
{
    auto it = m_abortedTasks.find(key);
    bool aborted = it != m_abortedTasks.end();
    if (aborted)
        agentID = it->second;

    return aborted;
}

/*!
 * \param key Task key
 * \return Whether the task has been aborted
 */
bool MasterTaskQueue::getTaskAborted(const TaskKey &key)
{
    return m_abortedTasks.find(key) != m_abortedTasks.cend();
}


/*!
 * \brief Called when the current task is completed
 * \param valid Whether there was a currently assigned task
 * \return Task completed, if valid
 */
TaskKey MasterTaskQueue::CurrentTaskCompleted(bool &valid)
{
    TaskKey key;
    valid = m_assignedTasks.getCurrentTaskKey(key);
    if (valid)
    {
        m_assignedTasks.RemoveFront();
        setTaskCompleted(key, m_agentID);
    } // Else no task was currently being performed
    return key;
}

/*!
 * \brief Called when some assigned task queue is completed, and not the currently assigned task
 * \details This function is used if the robotic agent completes tasks out of order of the assignment queue generated
 * by the auction. Otherwise, use CurrentTaskCompleted().
 * \param key Task key
 * \return Whether the task was assigned to this agent.
 */
bool MasterTaskQueue::AssignedTaskCompleted(const TaskKey &key)
{
    bool valid = m_assignedTasks.RemoveTask(key);
    if (valid)
        setTaskCompleted(key, m_agentID);
    return valid;
}

/*!
 * \brief Called when the current task is aborted
 * \param valid Whether there was a currently assigned task
 * \return Task completed, if valid
 */
TaskKey MasterTaskQueue::CurrentTaskAborted(bool &valid)
{
    TaskKey key;
    valid = m_assignedTasks.getCurrentTaskKey(key);
    if (valid)
    {
        m_assignedTasks.RemoveFront();
        setTaskAborted(key, m_agentID);
    } // Else no task was currently being performed
    return key;
}

/*!
 * \brief Called when some assigned task queue is aborted, and not the currently assigned task
 * \details This function is used if the robotic agent aborts tasks out of order of the assignment queue generated
 * by the auction. Otherwise, use CurrentTaskAborted().
 * \param key Task key
 * \return Whether the task was assigned to this agent.
 */
bool MasterTaskQueue::AssignedTaskAborted(const TaskKey &key)
{
    bool valid = m_assignedTasks.RemoveTask(key);
    if (valid)
        setTaskCompleted(key, m_agentID);
    return valid;
}

/*!
 * \brief Called when the current task is started
 * \param valid
 * \return Whether there was a currently assigned task
 */
TaskKey MasterTaskQueue::CurrentTaskStarted(bool &valid)
{
    TaskKey key;
    valid = m_assignedTasks.getCurrentTaskKey(key);
    if (valid)
    {
        int index = m_assignedTasks.getFirstNonConsensusIndex();
        if (index <= 0)
            m_assignedTasks.setFirstNonConsensusIndex(1);
        setTaskStarted(key, m_agentID);
    }
    return key;
}

/*!
 * \brief Called when some assigned task queue is started, and not the currently assigned task
 * \details This function is used if the robotic agent aborts tasks out of order of the assignment queue generated
 * by the auction. Otherwise, use CurrentTaskStarted().
 * \param key Task key
 * \return Whether the task was assigned to this agent.
 */
bool MasterTaskQueue::AssignedTaskStarted(const TaskKey &key)
{
    auto it = m_assignedTasks.Begin_NonConsensus();
    int index = m_assignedTasks.getFirstNonConsensusIndex();
    while (it != m_assignedTasks.End())
    {
        if (it->first == key)
        {
            m_assignedTasks.setFirstNonConsensusIndex(index);
            setTaskStarted(key, m_agentID);
            return true;
        }
        ++index;
        ++it;
    }
    return false;
}



/*!
 * \brief Returns whether anything about the task is known
 * \param key Task key
 * \return Whether anything about the task is known
 */
bool MasterTaskQueue::HasTask(const TaskKey &key)
{
    return m_taskData.find(key) != m_taskData.end();
}

/*!
 * \brief Returns whether the descriptor is available
 * \param key Task key
 * \return Whether the descriptor is available
 */
bool MasterTaskQueue::HasTaskDescriptor(const TaskKey &key)
{
    const TaskContainer &taskContainer = m_taskData.at(key);
    return (taskContainer.getDescriptor().get() != nullptr);
}

/*!
 * \brief Returns whether a leading bid is available
 * \param key Task key
 * \return Whether a leading bid is available
 */
bool MasterTaskQueue::HasLeadingBid(const TaskKey &key)
{
    if (m_taskData.find(key) == m_taskData.cend())
        return false;

    const TaskContainer &taskContainer = m_taskData.at(key);
    return (taskContainer.getBidContainer().getValid());
}

/*!
 * \brief Returns whether there are tasks available to bid on
 * \param key Task key
 * \return Whether there are tasks available to bid on
 */
bool MasterTaskQueue::HasAvailableTasks()
{
    return !m_availableTasks.Empty();
}

/*!
 * \brief Returns whether consensus has been reached on all tasks assigned to the agent
 * \return Whether there is consensus on all assigned tasks
 */
bool MasterTaskQueue::ConsensusOnAllAssignedTasks()
{
    return m_assignedTasks.ConsensusOnAllTasks();
}

/*!
 * \brief Retrieves the leading bid for a task
 *
 * Assumes that a valid leading bid for the task exists
 *
 * \sa HasLeadingBid()
 * \param key Task key
 * \return Leading bid for the task
 */
const BidDescriptor &MasterTaskQueue::getLeadingBid(const TaskKey &key)
{
    const TaskContainer &taskContainer = m_taskData.at(key);
    return taskContainer.getBidContainer().getLeadingBid();
}

/*!
 * \brief Retrieves the ID of the agent with the leading bid for a task
 *
 * Assumes that a valid leading bid for the task exists
 *
 * \sa HasLeadingBid()
 * \param key Task key
 * \return Leading agent ID for the task
 */
uint64_t MasterTaskQueue::getLeadingAgentID(const TaskKey &key)
{
    const TaskContainer &taskContainer = m_taskData.at(key);
    return taskContainer.getBidContainer().getLeadingAgent();
}

Data::EnvironmentTime MasterTaskQueue::getBidTimestamp(const TaskKey &key)
{
    const TaskContainer &taskContainer = m_taskData.at(key);
    Data::EnvironmentTime timestamp;
    taskContainer.getBidContainer().getAgentTime(m_agentID, timestamp);
    return timestamp;
}

const std::unordered_map<BidDescriptor, Data::EnvironmentTime> &MasterTaskQueue::getBidsToBroadcast() const
{
    return m_bidsToBroadcast;
}

void MasterTaskQueue::ClearBidsToBroadcast()
{
    m_bidsToBroadcast.clear();
}

const std::unordered_set<ScrubMessage> &MasterTaskQueue::getScrubMessagesToBroadcast() const
{
    return m_scrubMessagesToBroadcast;
}

void MasterTaskQueue::ClearScrubMessagesToBroadcast()
{
    m_scrubMessagesToBroadcast.clear();
}

std::vector<TaskKey> MasterTaskQueue::getUnassignedTasks() const
{
    std::vector<TaskKey> unassignedVector;
    for (const auto &keyContainerPair : m_taskData)
    {
        const auto &key = keyContainerPair.first;
        if (m_awardedTasks.find(key) == m_awardedTasks.cend())
        {
            if (!(m_startedTasks.find(key) == m_startedTasks.cend()
                  || m_abortedTasks.find(key) == m_abortedTasks.cend()
                  || m_completedTasks.find(key) == m_completedTasks.cend()))
            {
                unassignedVector.push_back(keyContainerPair.first);
            }
        }
    }
    return unassignedVector;
}

void MasterTaskQueue::setAuctionStats(const std::shared_ptr<AuctionStatistics_Agent> &auctionStats)
{
    m_auctionStats = auctionStats;
}

AssignmentTaskQueue MasterTaskQueue::getBelievedAssignedTasks(uint64_t agentID, bool &valid)
{
    AssignmentTaskQueue belief;
    if (agentID == m_agentID)
    {
        belief = m_assignedTasks;
        valid = true;
    }
    else
    {
        auto agentIterator = m_knownAgentBids.find(agentID);
        if (agentIterator == m_knownAgentBids.end())
        {
            valid = false;
        }
        else
        {
            valid = true;
            auto bidIt = agentIterator->second.begin();
            while (bidIt != agentIterator->second.end())
            {
                belief.AddTask(bidIt->second.getTaskKey());
                ++bidIt;
            }
        }
    }
    return belief;
}

/*!
 * \brief Invalidates bids from some agent.
 * \details Called when a previously leading bid gets outbid. Bids based on winning the bid are now invalid.
 * \param startBid Bid to start invalidation from
 */
void MasterTaskQueue::InvalidateBids(const BidDescriptor &startBid)
{
    auto agentIterator = m_knownAgentBids.find(startBid.getAgentID());
    if (agentIterator == m_knownAgentBids.end())
        return; // Nothing is known about the agent this bid was for. Should not occur

    auto &agentBidsMap = agentIterator->second;
    auto bidIterator = agentBidsMap.lower_bound(startBid.getGenerationTime());
    auto upperBound = agentBidsMap.upper_bound(startBid.getGenerationTime());
    while (bidIterator != upperBound)
    {
        if (bidIterator->second == startBid)
            break;
        ++bidIterator;
    }

    if (bidIterator == upperBound)
        return; // Bid was invalid/unknown


    if (startBid.getAgentID() == m_agentID)
        m_assignedTasks.RemoveTasksAfter(startBid.getTaskKey());

    while (bidIterator != agentBidsMap.end())
    {
        const BidDescriptor &bid = bidIterator->second;
        const TaskKey &key = bid.getTaskKey();
        TaskContainer &taskContainer = m_taskData.at(key);
        BidContainer &bidContainer = taskContainer.getBidContainer();

        if (bidContainer.getValid() && bidContainer.getLeadingBid() == bid)
        {
            bidContainer.InvalidateBid();
            m_awardedTasks.erase(key);
        }

        m_awardedTasks.erase(key);

        if (taskContainer.getAddressable())
            m_availableTasks.AddTask(key, taskContainer.getPriority());

        if (m_auctionStats)
            m_auctionStats->TrackInvalidation(key);

        bidIterator = agentBidsMap.erase(bidIterator);
    }
}


