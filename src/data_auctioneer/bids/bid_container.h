#ifndef BID_CONTAINER_H
#define BID_CONTAINER_H

#include <queue>
#include <unordered_map>

#include "bid_descriptor.h"
#include "data_tasks/task_key.h"
#include "bid_bundle.h"
#include "../scrub_message.h"


/*!
 * \brief The BidContainer class contains information about the current leading
 * bid for a task.
 */
class BidContainer
{
public:
    /*!
     * \brief The ReceiveAction enum indicates the action the receiving agent is to take on receiving a bid.
     */
    typedef enum class ReceiveAction
    {
        None, /*!< No action */
        BroadcastLocal, /*!< Broadcast local leading bid, which may have just been updated */
        BroadcastReceived, /*!< Broadcast the received bid */
        BroadcastScrub /*!< Broadcast a message indicating no leader */
    } ReceiveAction;

    BidContainer(uint64_t agentID, const TaskKey &key);

    ReceiveAction ReceiveBid(uint64_t sender,
                             const Data::EnvironmentTime &senderTimestamp,
                             const BidDescriptor &bid,
                             bool &newLeadingBid);

    ReceiveAction ReceiveScrub(const ScrubMessage &scrub);

    void InvalidateBid();

    const BidDescriptor &getLeadingBid() const;

    uint64_t getLeadingAgent() const;

    bool getValid() const;

    bool getAgentTime(uint64_t agentID, Data::EnvironmentTime &timestamp) const;
    void setAgentTime(uint64_t agentID, const Data::EnvironmentTime &timestamp);

private:
    uint64_t      m_agentID;
    TaskKey       m_taskKey;
    BidDescriptor m_leadingBid;
    bool          m_valid = false;

    // Last time that we received information about a bid on the task this BidContainer corresponds to
    // for each agent

    std::unordered_map<uint64_t, Data::EnvironmentTime> m_agentUpdateTimes;
};

#endif // BID_CONTAINER_H
