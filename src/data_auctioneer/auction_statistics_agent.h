#ifndef AUCTION_RTA_STATISTICS_H
#define AUCTION_RTA_STATISTICS_H

#include <stdint.h>
#include <unordered_map>
#include <iostream>
#include <mutex>

#include "data_tasks/task_key.h"
#include "bids/bid_bundle.h"
#include "bids/bid_descriptor.h"
#include "scrub_message.h"

#include "common/watchdog.h"

/*!
 * \brief The AuctionStatistics_Common struct stores statistics common to AuctionStatistics_Task and AuctionStatistics_Global
 */
typedef struct AuctionStatistics_Common
{
    int taskKeyBroadcastsSent = 0;
    int taskKeyBroadcastsRcv = 0;

    int taskDescriptorRequestsSent = 0;
    int taskDescriptorRequestsRcv = 0;

    int taskDescriptorsSent = 0;
    int taskDescriptorsRcv = 0;

    int bundlesSent = 0;
    int bundlesRcv = 0;

    int bidsRebroadcastSent = 0;
    int bidsRebroadcastRcv = 0;

    int scrubMessagesSent = 0;
    int scrubMessagesRcv = 0;

    int taskCompletionBroadcastsSent = 0;
    int taskCompletionBroadcastsRcv = 0;

    int taskAbortionBroadcastsSent = 0;
    int taskAbortionBroadcastsRcv = 0;

    int taskStartedBroadcastsSent = 0;
    int taskStartedBroadcastsRcv = 0;

    int bidsGeneratedForSelf = 0;

    int timesAssignedToSelf = 0;
    int timesAssignedToOther = 0;
    int timesInvalidated = 0;

    double highestUtility = 0;
    double currentUtility = 0;

    // The stats are considered modified under the following conditions: a task key is received; a task descriptor is received;
    // an assigmnent change occurs; a bundle is sent or received; an invalidation occurs; or the task is completed. This allows
    // logging can be stopped once an agent believes that consensus was reached.
    bool modified = false;

    static void PrintLayout(std::ostream &out,
                            const std::string &separator = ",");

    virtual void Print(std::ostream &out,
               bool displayValueNames = false,
               const std::string &valueSeparator = ",",
               const std::string &namesSeparator = ":") const;
} AuctionStatistics_Common;

/*!
 * \brief The AuctionStatistics_Task struct stores statistics about a task for AuctionStatistics_Agent
 */
typedef struct AuctionStatistics_Task : AuctionStatistics_Common
{
    Data::EnvironmentTime firstAssigned;
    Data::EnvironmentTime lastAssignmentChange;
    bool completed = false;
    bool aborted = false;
    bool started = false;
    bool assigned = false;

    static void PrintLayout(std::ostream &out,
                            const std::string &separator = ",");

    virtual void Print(std::ostream &out,
               bool displayValueNames = false,
               const std::string &valueSeparator = ",",
               const std::string &namesSeparator = ":") const;
} AuctionStatistics_Task;

/*!
 * \brief The AuctionStatistics_Global struct stores global statistics for AuctionStatistics_Agent
 */
typedef struct AuctionStatistics_Global : AuctionStatistics_Common
{
    int totalTasks = 0;
    int completedTasks = 0;
    int abortedTasks = 0;
    int startedTasks = 0;
    int assignedTasks = 0;
    double taskGenerationFrequency = 0;
    double completionFrequency = 0;
    double abortionFrequency = 0;
    double startFrequency = 0;
    double messageSendFrequency = 0;
    double messageRcvFrequency = 0;

    static void PrintLayout(std::ostream &out,
                            const std::string &separator = ",");

    virtual void Print(std::ostream &out,
               bool displayValueNames = false,
               const std::string &valueSeparator = ",",
               const std::string &namesSeparator = ":") const;
} AuctionStatistics_Global;

/*!
 * \brief The AuctionStatistics_Agent class is used to track auction statistics for an agent
 * \details Statistics are tracked both per task, and globally.
 */
class AuctionStatistics_Agent
{
public:
    typedef enum MessageType
    {
        Send,
        Rcv
    } MessageType;

    AuctionStatistics_Agent();

    void setAgentID(uint64_t agentID);

    void TrackTaskKey(MessageType type, const TaskKey &key);

    void TrackTaskDescriptorRequest(MessageType type, const TaskKey &key);
    void TrackTaskDescriptor(MessageType type, const TaskKey &key);
    void TrackBidBundle(MessageType type, const BidBundle &bundle);
    void TrackBid(MessageType type, const BidDescriptor &bid);
    void TrackScrubMessage(MessageType type, const ScrubMessage &scrub);

    void TrackCompletion(MessageType type, const TaskKey &key);
    void TrackAbortion(MessageType type, const TaskKey &key);
    void TrackStart(MessageType type, const TaskKey &key);


    void TrackAssignment(const TaskKey &key, uint64_t agentID, double utility);
    void TrackInvalidation(const TaskKey &key);

    void ResetGlobalChangedFlag();
    void ResetChangedFlag(const TaskKey &key);

    const AuctionStatistics_Global &getGlobalStats() const;
    const std::unordered_map<TaskKey, AuctionStatistics_Task> &getTaskStats() const;

private:
    uint64_t m_agentID;

    AuctionStatistics_Global m_globalStats;
    std::unordered_map<TaskKey, AuctionStatistics_Task> m_taskStats;

    AuctionStatistics_Task &getTaskStats(const TaskKey &key);

    std::mutex m_mutex;

    std::shared_ptr<ContinuousWatchdog> m_dynamicStatsWatchdog;

    int m_numRecentNewTasks = 0;
    int m_numRecentCompletedTasks = 0;
    int m_numRecentAbortedTasks = 0;
    int m_numRecentStartedTasks = 0;

    int m_numRecentMessagesSent = 0;
    int m_numRecentMessagesRcv = 0;

    void CalculateDynamicStats(double duration);
};


// Generic stream output
inline std::ostream &operator<<(std::ostream &out, const AuctionStatistics_Common& stats)
{
    stats.Print(out);
    return out;
}

inline std::ostream &operator<<(std::ostream &out, const AuctionStatistics_Task& stats)
{
    stats.Print(out);
    return out;
}

inline std::ostream &operator<<(std::ostream &out, const AuctionStatistics_Global& stats)
{
    stats.Print(out);
    return out;
}


#endif // AUCTION_RTA_STATISTICS_H
