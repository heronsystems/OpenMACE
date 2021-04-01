#include "auction_statistics_agent.h"

AuctionStatistics_Agent::AuctionStatistics_Agent()
{
    auto lambda = [&]()
    {
        CalculateDynamicStats(0.1);

        return false;
    };
    m_dynamicStatsWatchdog.reset(new ContinuousWatchdog(std::chrono::milliseconds(100), lambda));
}

/*!
 * \param agentID Agent ID
 */
void AuctionStatistics_Agent::setAgentID(uint64_t agentID)
{
    m_agentID = agentID;
}

/*!
 * \brief Tracks statistics for receiving/sending a broadcasted task key
 * \param type Send or Receive
 * \param key Task key
 */
void AuctionStatistics_Agent::TrackTaskKey(AuctionStatistics_Agent::MessageType type, const TaskKey &key)
{
    std::lock_guard<std::mutex> guard(m_mutex);

    auto &taskStats = getTaskStats(key);
    taskStats.modified = true;
    m_globalStats.modified = true;

    switch (type)
    {
        case MessageType::Send:
            ++m_globalStats.taskKeyBroadcastsSent;
            ++taskStats.taskKeyBroadcastsSent;
            ++m_numRecentMessagesSent;
            break;
        case MessageType::Rcv:
            ++m_globalStats.taskKeyBroadcastsRcv;
            ++taskStats.taskKeyBroadcastsRcv;
            ++m_numRecentMessagesRcv;
            break;
        default:
            std::cout << "Unknown message type to track for auction statistics" << std::endl;
            break;
    };
}

/*!
 * \brief Tracks statistics for receiving/sending a task descriptor request
 * \param type Send or Receive
 * \param key Task key
 */
void AuctionStatistics_Agent::TrackTaskDescriptorRequest(AuctionStatistics_Agent::MessageType type, const TaskKey &key)
{
    std::lock_guard<std::mutex> guard(m_mutex);

    auto &taskStats = getTaskStats(key);
    taskStats.modified = true;
    m_globalStats.modified = true;

    switch (type)
    {
        case MessageType::Send:
            ++m_globalStats.taskDescriptorRequestsSent;
            ++taskStats.taskDescriptorRequestsSent;
            ++m_numRecentMessagesSent;
            break;
        case MessageType::Rcv:
            ++m_globalStats.taskDescriptorRequestsRcv;
            ++taskStats.taskDescriptorRequestsRcv;
            ++m_numRecentMessagesRcv;
            break;
        default:
            std::cout << "Unknown message type to track for auction statistics" << std::endl;
            break;
    };
}

/*!
 * \brief Tracks statistics for receiving/sending a response to a track descriptor request
 * \param type Send or Receive
 * \param key Task key
 */
void AuctionStatistics_Agent::TrackTaskDescriptor(AuctionStatistics_Agent::MessageType type, const TaskKey &key)
{
    auto &taskStats = getTaskStats(key);
    taskStats.modified = true;
    m_globalStats.modified = true;

    switch (type)
    {
        case MessageType::Send:
            ++m_globalStats.taskDescriptorsSent;
            ++taskStats.taskDescriptorsSent;
            ++m_numRecentMessagesSent;
            break;
        case MessageType::Rcv:
            ++m_globalStats.taskDescriptorsRcv;
            ++taskStats.taskDescriptorsRcv;
            ++m_numRecentMessagesRcv;
            break;
        default:
            std::cout << "Unknown message type to track for auction statistics" << std::endl;
            break;
    };
}

/*!
 * \brief Tracks statistics for receiving/sending a bid bundle
 * \param type Send or Receive
 * \param bundle Bid bundle
 */
void AuctionStatistics_Agent::TrackBidBundle(AuctionStatistics_Agent::MessageType type, const BidBundle &bundle)
{
    std::lock_guard<std::mutex> guard(m_mutex);

    m_globalStats.modified = true;
    uint64_t agentID = bundle.getAgentID();
    switch (type)
    {
        case MessageType::Send:
            ++m_globalStats.bundlesSent;
            for (auto it = bundle.Begin(); it != bundle.End(); ++it)
            {
                auto &taskStats = getTaskStats(it->descriptor.getTaskKey());

                taskStats.modified = true;
                ++taskStats.bundlesSent;
                ++taskStats.bidsGeneratedForSelf;
                ++m_globalStats.bidsGeneratedForSelf;
            }
            ++m_numRecentMessagesSent;
            break;
        case MessageType::Rcv:
            ++m_globalStats.bundlesRcv;
            for (auto it = bundle.Begin(); it != bundle.End(); ++it)
            {
                auto &taskStats = getTaskStats(it->descriptor.getTaskKey());

                taskStats.modified = true;
                ++taskStats.bundlesRcv;
            }
            ++m_numRecentMessagesRcv;
            break;
        default:
            std::cout << "Unknown message type to track for auction statistics" << std::endl;
            break;
    };
}

/*!
 * \brief Tracks statistics for receiving/sending a re-broadcast bid
 * \param type Send or Receive
 * \param bid Bid
 */
void AuctionStatistics_Agent::TrackBid(AuctionStatistics_Agent::MessageType type, const BidDescriptor &bid)
{
    auto &taskStats = getTaskStats(bid.getTaskKey());

    switch (type)
    {
        case MessageType::Send:
            ++m_globalStats.bidsRebroadcastSent;
            ++taskStats.bidsRebroadcastSent;
            ++m_numRecentMessagesSent;
            break;
        case MessageType::Rcv:
            ++m_globalStats.bidsRebroadcastRcv;
            ++taskStats.bidsRebroadcastRcv;
            ++m_numRecentMessagesRcv;
            break;
        default:
            std::cout << "Unknown message type to track for auction statistics" << std::endl;
            break;
    };
}

/*!
 * \brief Tracks statistics for receiving/sending a scrub message
 * \param type Send or Receive
 * \param scrub Scrub message
 */
void AuctionStatistics_Agent::TrackScrubMessage(AuctionStatistics_Agent::MessageType type, const ScrubMessage &scrub)
{
    std::lock_guard<std::mutex> guard(m_mutex);

    auto &taskStats = getTaskStats(scrub.taskKey);

    switch (type)
    {
        case MessageType::Send:
            ++m_globalStats.scrubMessagesSent;
            ++taskStats.scrubMessagesSent;
            ++m_numRecentMessagesSent;
            break;
        case MessageType::Rcv:
            ++m_globalStats.scrubMessagesRcv;
            ++taskStats.scrubMessagesRcv;
            ++m_numRecentMessagesRcv;
            break;
        default:
            std::cout << "Unknown message type to track for auction statistics" << std::endl;
            break;
    };
}

void AuctionStatistics_Agent::TrackCompletion(AuctionStatistics_Agent::MessageType type, const TaskKey &key)
{
    std::lock_guard<std::mutex> guard(m_mutex);
    auto &taskStats = getTaskStats(key);
    m_globalStats.modified = true;
    taskStats.modified = true;
    if (!taskStats.completed)
    {
        taskStats.completed = true;
        ++m_numRecentCompletedTasks;
        ++m_globalStats.completedTasks;
        if (!taskStats.assigned)
        {

            taskStats.assigned = true;
            Data::EnvironmentTime now;
            Data::EnvironmentTime::CurrentTime(Data::SYSTEMCLOCK, now);
            if (taskStats.timesAssignedToOther == 0 && taskStats.timesAssignedToSelf == 0)
            {
                taskStats.firstAssigned = now;
            }
            taskStats.lastAssignmentChange = now;
            ++m_globalStats.assignedTasks;
        }
    }

    switch (type)
    {
        case MessageType::Send:
            ++taskStats.taskCompletionBroadcastsSent;
            ++m_globalStats.taskCompletionBroadcastsSent;
            ++m_numRecentMessagesSent;
            break;
        case MessageType::Rcv:
            ++taskStats.taskCompletionBroadcastsRcv;
            ++m_globalStats.taskCompletionBroadcastsRcv;
            ++m_numRecentMessagesRcv;
            break;
        default:
            std::cout << "Unknown message type to track for auction statistics" << std::endl;
            break;
    };

}

void AuctionStatistics_Agent::TrackAbortion(AuctionStatistics_Agent::MessageType type, const TaskKey &key)
{
    std::lock_guard<std::mutex> guard(m_mutex);
    auto &taskStats = getTaskStats(key);
    m_globalStats.modified = true;
    taskStats.modified = true;
    if (!taskStats.aborted)
    {
        taskStats.aborted = true;
        ++m_numRecentAbortedTasks;
        ++m_globalStats.abortedTasks;
        if (!taskStats.assigned)
        {

            taskStats.assigned = true;
            Data::EnvironmentTime now;
            Data::EnvironmentTime::CurrentTime(Data::SYSTEMCLOCK, now);
            if (taskStats.timesAssignedToOther == 0 && taskStats.timesAssignedToSelf == 0)
            {
                taskStats.firstAssigned = now;
            }
            taskStats.lastAssignmentChange = now;
            ++m_globalStats.assignedTasks;
        }
    }

    switch (type)
    {
        case MessageType::Send:
            ++taskStats.taskAbortionBroadcastsSent;
            ++m_globalStats.taskAbortionBroadcastsSent;
            ++m_numRecentMessagesSent;
            break;
        case MessageType::Rcv:
            ++taskStats.taskAbortionBroadcastsRcv;
            ++m_globalStats.taskAbortionBroadcastsRcv;
            ++m_numRecentMessagesRcv;
            break;
        default:
            std::cout << "Unknown message type to track for auction statistics" << std::endl;
            break;
    };
}

void AuctionStatistics_Agent::TrackStart(AuctionStatistics_Agent::MessageType type, const TaskKey &key)
{
    std::lock_guard<std::mutex> guard(m_mutex);
    auto &taskStats = getTaskStats(key);
    m_globalStats.modified = true;
    taskStats.modified = true;
    if (!taskStats.started)
    {
        taskStats.started = true;
        ++m_numRecentStartedTasks;
        ++m_globalStats.startedTasks;
        if (!taskStats.assigned)
        {

            taskStats.assigned = true;
            Data::EnvironmentTime now;
            Data::EnvironmentTime::CurrentTime(Data::SYSTEMCLOCK, now);
            if (taskStats.timesAssignedToOther == 0 && taskStats.timesAssignedToSelf == 0)
            {
                taskStats.firstAssigned = now;
            }
            taskStats.lastAssignmentChange = now;
            ++m_globalStats.assignedTasks;
        }
    }

    switch (type)
    {
        case MessageType::Send:
            ++taskStats.taskStartedBroadcastsSent;
            ++m_globalStats.taskStartedBroadcastsSent;
            ++m_numRecentMessagesSent;
            break;
        case MessageType::Rcv:
            ++taskStats.taskStartedBroadcastsRcv;
            ++m_globalStats.taskStartedBroadcastsRcv;
            ++m_numRecentMessagesRcv;
            break;
        default:
            std::cout << "Unknown message type to track for auction statistics" << std::endl;
            break;
    };
}

/*!
 * \brief Tracks statistics for assigning a task
 * \param key Task key
 * \param agentID Assigned agent ID
 * \param utility Utility of winning bid
 */
void AuctionStatistics_Agent::TrackAssignment(const TaskKey &key, uint64_t agentID, double utility)
{
    auto &taskStats = getTaskStats(key);
    taskStats.modified = true;
    m_globalStats.modified = true;

    if (!taskStats.completed && !taskStats.aborted && !taskStats.started)
    {
        if (!taskStats.assigned)
            ++m_globalStats.assignedTasks;

        taskStats.assigned = true;
        Data::EnvironmentTime now;
        Data::EnvironmentTime::CurrentTime(Data::SYSTEMCLOCK, now);
        if (taskStats.timesAssignedToOther == 0 && taskStats.timesAssignedToSelf == 0)
            taskStats.firstAssigned = now;

        taskStats.lastAssignmentChange = now;
    }

    if (agentID == m_agentID)
    {
        ++m_globalStats.timesAssignedToSelf;
        ++taskStats.timesAssignedToSelf;
    }
    else
    {
        ++m_globalStats.timesAssignedToOther;
        ++taskStats.timesAssignedToOther;
    }
    m_globalStats.currentUtility -= taskStats.currentUtility;
    m_globalStats.currentUtility += utility;
    taskStats.currentUtility = utility;

    if (taskStats.currentUtility > taskStats.highestUtility)
        taskStats.highestUtility = utility;

    if (m_globalStats.currentUtility > m_globalStats.highestUtility)
        m_globalStats.highestUtility = m_globalStats.currentUtility;
}

/*!
 * \brief Tracks statistics for invalidating a task assignment
 * \param key Task key
 */
void AuctionStatistics_Agent::TrackInvalidation(const TaskKey &key)
{
    std::lock_guard<std::mutex> guard(m_mutex);

    auto &taskStats = getTaskStats(key);
    taskStats.modified = true;
    m_globalStats.modified = true;

    ++m_globalStats.timesInvalidated;
    ++taskStats.timesInvalidated;

    m_globalStats.currentUtility -= taskStats.currentUtility;
    taskStats.currentUtility = 0;

    if (!taskStats.completed && !taskStats.aborted && !taskStats.started)
    {
        if (taskStats.assigned)
            --m_globalStats.assignedTasks;

        taskStats.assigned = false;
        Data::EnvironmentTime now;
        Data::EnvironmentTime::CurrentTime(Data::SYSTEMCLOCK, now);
        if (taskStats.timesAssignedToOther == 0 && taskStats.timesAssignedToSelf == 0) // should not occur
            taskStats.firstAssigned = now;

        taskStats.lastAssignmentChange = now;
    }
}

/*!
 * \brief Resets the global change flag
 */
void AuctionStatistics_Agent::ResetGlobalChangedFlag()
{
    std::lock_guard<std::mutex> guard(m_mutex);

    m_globalStats.modified = false;
}

/*!
 * \brief Resets the change flag for a particular task
 * \param key Task key
 */
void AuctionStatistics_Agent::ResetChangedFlag(const TaskKey &key)
{
    std::lock_guard<std::mutex> guard(m_mutex);

    auto it = m_taskStats.find(key);
    auto &taskStats = it->second;
    taskStats.modified = false;
}

/*!
 * \return Global stats
 */
const AuctionStatistics_Global &AuctionStatistics_Agent::getGlobalStats() const
{
    return m_globalStats;
}

/*!
 * \return Stats for all tasks
 */
const std::unordered_map<TaskKey, AuctionStatistics_Task> &AuctionStatistics_Agent::getTaskStats() const
{
    return m_taskStats;
}

/*!
 * \return Stats for a particular task
 */
AuctionStatistics_Task &AuctionStatistics_Agent::getTaskStats(const TaskKey &key)
{
    auto taskStatsIt = m_taskStats.find(key);
    if (taskStatsIt == m_taskStats.end())
    {
        m_taskStats.insert({key, AuctionStatistics_Task()});
        taskStatsIt = m_taskStats.find(key);
        ++m_numRecentNewTasks;
        ++m_globalStats.totalTasks;
    }
    auto &taskStats = taskStatsIt->second;
    return taskStats;
}

void AuctionStatistics_Agent::CalculateDynamicStats(double duration)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_globalStats.taskGenerationFrequency = 0.95 * m_globalStats.taskGenerationFrequency
                                      + 0.05 * ((double) m_numRecentNewTasks) / duration;
    m_globalStats.completionFrequency = 0.95 * m_globalStats.completionFrequency
                                      + 0.05 * ((double) m_numRecentCompletedTasks) / duration;
    m_globalStats.abortionFrequency = 0.95 * m_globalStats.abortionFrequency
                                      + 0.05 * ((double) m_numRecentAbortedTasks) / duration;
    m_globalStats.startFrequency = 0.95 * m_globalStats.startFrequency
                                      + 0.05 * ((double) m_numRecentStartedTasks) / duration;

    m_numRecentNewTasks = 0;
    m_numRecentCompletedTasks = 0;
    m_numRecentAbortedTasks = 0;
    m_numRecentStartedTasks = 0;

    m_globalStats.messageSendFrequency = 0.95 * m_globalStats.messageSendFrequency
                                       + 0.05 * ((double) m_numRecentMessagesSent);
    m_globalStats.messageRcvFrequency = 0.95 * m_globalStats.messageRcvFrequency
                                      + 0.05 * ((double) m_numRecentMessagesRcv);

    m_numRecentMessagesSent = 0;
    m_numRecentMessagesRcv = 0;
}

/******* AuctionStatistics print functions *******/

void AuctionStatistics_Common::PrintLayout(std::ostream &out, const std::string &separator)
{
    out << "taskKeyBroadcastsSent" << separator
        << "taskKeyBroadcastsRcv" << separator
        << "taskDescriptorRequestsSent" << separator
        << "taskDescriptorRequestsRcv" << separator
        << "taskDescriptorsSent" << separator
        << "taskDescriptorsRcv" << separator
        << "bundlesSent" << separator
        << "bundlesRcv" << separator
        << "bidsRebroadcastSent" << separator
        << "bidsRebroadcastRcv" << separator
        << "scrubMessagesSent" << separator
        << "scrubMessagesRcv" << separator
        << "bidsGeneratedForSelf" << separator
        << "timesAssignedToSelf" << separator
        << "timesAssignedToOther" << separator
        << "timesInvalidated" << separator
        << "completionMessagesSent" << separator
        << "completionMessagesRcv" << separator
        << "abortionMessagesSent" << separator
        << "abortionMessagesRcv" << separator
        << "startMessagesSent" << separator
        << "startMessagesRcv" << separator
        << "highestUtility" << separator
        << "currentUtility" << separator;
}

void AuctionStatistics_Common::Print(std::ostream &out, bool displayValueNames, const std::string &valueSeparator, const std::string &namesSeparator) const
{
    if (displayValueNames)
        out << "taskKeyBroadcastsSent" << namesSeparator;
    out << taskKeyBroadcastsSent << valueSeparator;

    if (displayValueNames)
        out << "taskKeyBroadcastsRcv" << namesSeparator;
    out << taskKeyBroadcastsRcv << valueSeparator;

    if (displayValueNames)
        out << "taskDescriptorRequestsSent" << namesSeparator;
    out << taskDescriptorRequestsSent << valueSeparator;

    if (displayValueNames)
        out << "taskDescriptorRequestsRcv" << namesSeparator;
    out << taskDescriptorRequestsRcv << valueSeparator;

    if (displayValueNames)
        out << "taskDescriptorsSent" << namesSeparator;
    out << taskDescriptorsSent << valueSeparator;

    if (displayValueNames)
        out << "taskDescriptorsRcv" << namesSeparator;
    out << taskDescriptorsRcv << valueSeparator;

    if (displayValueNames)
        out << "bundlesSent" << namesSeparator;
    out << bundlesSent << valueSeparator;

    if (displayValueNames)
        out << "bundlesRcv" << namesSeparator;
    out << bundlesRcv << valueSeparator;

    if (displayValueNames)
        out << "bidsRebroadcastSent" << namesSeparator;
    out << bidsRebroadcastSent << valueSeparator;

    if (displayValueNames)
        out << "bidsRebroadcastRcv" << namesSeparator;
    out << bidsRebroadcastRcv << valueSeparator;

    if (displayValueNames)
        out << "scrubMessagesSent" << namesSeparator;
    out << scrubMessagesSent << valueSeparator;

    if (displayValueNames)
        out << "scrubMessagesRcv" << namesSeparator;
    out << scrubMessagesRcv << valueSeparator;

    if (displayValueNames)
        out << "bidsGeneratedForSelf" << namesSeparator;
    out << bidsGeneratedForSelf << valueSeparator;

    if (displayValueNames)
        out << "timesAssignedToSelf" << namesSeparator;
    out << timesAssignedToSelf << valueSeparator;

    if (displayValueNames)
        out << "timesAssignedToOther" << namesSeparator;
    out << timesAssignedToOther << valueSeparator;

    if (displayValueNames)
        out << "timesInvalidated" << namesSeparator;
    out << timesInvalidated << valueSeparator;

    if (displayValueNames)
        out << "completionMessagesSent" << namesSeparator;
    out << taskCompletionBroadcastsSent << valueSeparator;

   if (displayValueNames)
       out << "completionMessagesRcv" << namesSeparator;
   out << taskCompletionBroadcastsRcv << valueSeparator;

   if (displayValueNames)
       out << "abortionMessagesSent" << namesSeparator;
   out << taskAbortionBroadcastsSent << valueSeparator;

   if (displayValueNames)
       out << "abortionMessagesRcv" << namesSeparator;
   out << taskAbortionBroadcastsRcv << valueSeparator;

   if (displayValueNames)
       out << "startMessagesSent" << namesSeparator;
   out << taskStartedBroadcastsSent << valueSeparator;

   if (displayValueNames)
       out << "startMessagesRcv" << namesSeparator;
   out << taskStartedBroadcastsRcv << valueSeparator;

    if (displayValueNames)
        out << "highestUtility" << namesSeparator;
    out << highestUtility << valueSeparator;

    if (displayValueNames)
        out << "currentUtility" << namesSeparator;
    out << currentUtility << valueSeparator;
}

void AuctionStatistics_Task::PrintLayout(std::ostream &out, const std::string &separator)
{
    out << "AuctionStatistics_Task" << separator;

    AuctionStatistics_Common::PrintLayout(out, separator);
    out << "firstAssigned" << separator
        << "lastAssignmentChange" << separator;
}

void AuctionStatistics_Task::Print(std::ostream &out, bool displayValueNames, const std::string &valueSeparator, const std::string &namesSeparator) const
{
    out << "AuctionStatistics_Task" << valueSeparator;

    AuctionStatistics_Common::Print(out, displayValueNames, valueSeparator, namesSeparator);

    bool hasBeenAssigned = assigned || timesAssignedToOther > 0 || timesAssignedToSelf > 0;
    if (displayValueNames)
        out << "firstAssigned" << namesSeparator;
    if (hasBeenAssigned)
        out << firstAssigned.ToString().toStdString() << valueSeparator;
    else
        out << "Never" << valueSeparator;

    if (displayValueNames)
        out << "lastAssignmentChange" << namesSeparator;
    if (hasBeenAssigned)
        out << lastAssignmentChange.ToString().toStdString() << valueSeparator;
    else
        out << "Never";
}

void AuctionStatistics_Global::PrintLayout(std::ostream &out, const std::string &separator)
{
    out << "AuctionStatistics_Global" << separator;

    AuctionStatistics_Common::PrintLayout(out, separator);
        out << "totalTasks" << separator
            << "completedTasks" << separator
            << "abortedTasks" << separator
            << "startedTasks" << separator
            << "assignedTasks" << separator
            << "unassignedTasks" << separator
            << "taskGenerationFrequency" << separator
            << "completionFrequency" << separator
            << "abortionFrequency" << separator
            << "startFrequency" << separator
            << "messageSendFrequency" << separator
            << "messageRcvFrequency";
}

void AuctionStatistics_Global::Print(std::ostream &out,
                                     bool displayValueNames,
                                     const std::string &valueSeparator,
                                     const std::string &namesSeparator) const
{
    out << "AuctionStatistics_Global" << valueSeparator;

    AuctionStatistics_Common::Print(out, displayValueNames, valueSeparator, namesSeparator);

    if (displayValueNames)
        out << "totalTasks" << namesSeparator;
    out << totalTasks << valueSeparator;

    if (displayValueNames)
        out << "completedTasks" << namesSeparator;
    out << completedTasks << valueSeparator;

    if (displayValueNames)
        out << "abortedTasks" << namesSeparator;
    out << abortedTasks << valueSeparator;

    if (displayValueNames)
        out << "startedTasks" << namesSeparator;
    out << startedTasks << valueSeparator;

    if (displayValueNames)
        out << "assignedTasks" << namesSeparator;
    out << assignedTasks << valueSeparator;

    if (displayValueNames)
        out << "unassignedTasks" << namesSeparator;
    out << (totalTasks - assignedTasks) << valueSeparator;

    if (displayValueNames)
        out << "taskGenerationFrequency" << namesSeparator;
    out << taskGenerationFrequency << valueSeparator;

    if (displayValueNames)
        out << "completionFrequency" << namesSeparator;
    out << completionFrequency << valueSeparator;

    if (displayValueNames)
        out << "abortionFrequency" << namesSeparator;
    out << abortionFrequency << valueSeparator;


    if (displayValueNames)
        out << "startFrequency" << namesSeparator;
    out << startFrequency << valueSeparator;


    if (displayValueNames)
        out << "messageSendFrequency" << namesSeparator;
    out << messageSendFrequency << valueSeparator;

    if (displayValueNames)
        out << "messageRcvFrequency" << namesSeparator;
    out << messageRcvFrequency;
}
