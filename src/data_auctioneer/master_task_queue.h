#ifndef MASTER_TASK_QUEUE_H
#define MASTER_TASK_QUEUE_H

#include <memory>
#include <vector>
#include <unordered_map>
#include <unordered_set>

#include "state/vehicle_state.h"
#include "bids/bid_bundle.h"
#include "data_tasks/task_key.h"
#include "data_tasks/task_descriptor.h"
#include "data_auctioneer/bids/bid_descriptor.h"

#include "available_task_queue.h"
#include "awarded_task_queue.h"
#include "assignment_task_queue.h"

#include "task_container.h"

#include "auction_statistics_agent.h"


/*!
 * \brief The MasterTaskQueue class stores all available information about tasks.
 */
class MasterTaskQueue
{
public:
    MasterTaskQueue();

    void setAgentID(uint64_t agentID);

    bool AddTaskKey(const TaskKey &key,
                    bool addressable,
                    int priority = 0);

    void AddTaskDescriptor(const std::shared_ptr<TaskDescriptor> &descriptor);

    const std::shared_ptr<TaskDescriptor> &getTaskDescriptor(const TaskKey &key);

    const AssignmentTaskQueue &getAssignedTasks() const;

    const AwardedTaskQueue &getAwardedTasks() const;

    const AvailableTaskQueue &getAvailableTasks() const;

    bool AddBidBundle(const std::shared_ptr<BidBundle> &bundle);
    bool ReceiveBid(uint64_t sender,
                    const Data::EnvironmentTime &senderTimestamp,
                    const BidDescriptor &bid,
                    const VehicleStatePtr &finalState = nullptr);

    void ReceiveScrubMessage(const ScrubMessage &scrub);

    bool getConsensusReached(const TaskKey &key);
//    void setConsensusReached(const TaskKey &key, const BidDescriptor &winningBid);

    int getPriority(const TaskKey &key);
    void setPriority(const TaskKey &key, int priority);

    bool setTaskCompleted(const TaskKey &key, uint64_t completingAgentID);
    bool getTaskCompleted(const TaskKey &key, uint64_t &agentID);
    bool getTaskCompleted(const TaskKey &key);

    bool setTaskStarted(const TaskKey &key, uint64_t startingAgentID);
    bool getTaskStarted(const TaskKey &key, uint64_t &agentID);
    bool getTaskStarted(const TaskKey &key);

    bool setTaskAborted(const TaskKey &key, uint64_t abortingAgentID);
    bool getTaskAborted(const TaskKey &key, uint64_t &agentID);
    bool getTaskAborted(const TaskKey &key);

    TaskKey CurrentTaskCompleted(bool &valid);

    bool AssignedTaskCompleted(const TaskKey &key);

    TaskKey CurrentTaskAborted(bool &valid);

    bool AssignedTaskAborted(const TaskKey &key);

    TaskKey CurrentTaskStarted(bool &valid);

    bool AssignedTaskStarted(const TaskKey &key);

    bool HasTask(const TaskKey &key);

    bool HasTaskDescriptor(const TaskKey &key);

    bool HasLeadingBid(const TaskKey &key);

    bool HasAvailableTasks();

    bool ConsensusOnAllAssignedTasks();

    const BidDescriptor &getLeadingBid(const TaskKey &key);
    uint64_t getLeadingAgentID(const TaskKey &key);
    Data::EnvironmentTime getBidTimestamp(const TaskKey &key);

    const std::unordered_map<BidDescriptor, Data::EnvironmentTime> &getBidsToBroadcast() const;
    void ClearBidsToBroadcast();

    const std::unordered_set<ScrubMessage> &getScrubMessagesToBroadcast() const;
    void ClearScrubMessagesToBroadcast();

    std::vector<TaskKey> getUnassignedTasks() const;

    void setAuctionStats(const std::shared_ptr<AuctionStatistics_Agent> &auctionStats);

    AssignmentTaskQueue getBelievedAssignedTasks(uint64_t agentID, bool &valid);

private:
    std::unordered_map<TaskKey, TaskContainer> m_taskData;

    uint64_t            m_agentID;          // ID of agent owning this object
    AvailableTaskQueue  m_availableTasks;   // Addressable by the agent owning this object, and without consensus reached
    AwardedTaskQueue    m_awardedTasks;     // Awarded to any agent, with parameters about the winner
    AssignmentTaskQueue m_assignedTasks;    // Tasks assigned to the agent owning this object

    std::unordered_map<uint64_t, std::multimap<Data::EnvironmentTime, BidDescriptor>> m_knownAgentBids;
    // Agent ID -> (known bids sorted by generation time)
    // If bids for different tasks have the same timestamp, we assume that order of arrival dictates the order of assignment.

    // These are temporary data stores consumed by the RTA module to re-broadcast bids or broadcast scrub messages.
    std::unordered_map<BidDescriptor, Data::EnvironmentTime> m_bidsToBroadcast;
    std::unordered_set<ScrubMessage>  m_scrubMessagesToBroadcast;

    // Tasks which are known to have been completed, mapped to the agent who completed the task.
    std::unordered_map<TaskKey, uint64_t> m_completedTasks;

    // Tasks which are known to have been aborted, mapped to the agent who completed the task.
    std::unordered_map<TaskKey, uint64_t> m_abortedTasks;

    // Tasks which are known to have been started (but not completed/aborted), mapped to the agent who started the task.
    std::unordered_map<TaskKey, uint64_t> m_startedTasks;

    // Pointer to a statistics object owned by an RTA module. This allows statistics to be tracked for task assignments and invalidation,
    // which is not directly accessible to the RTA module. Other statistics are being tracked by the RTA module directly.
    std::shared_ptr<AuctionStatistics_Agent> m_auctionStats = nullptr;

    void InvalidateBids(const BidDescriptor &startBid);

};

#endif // MASTER_TASK_QUEUE_H
