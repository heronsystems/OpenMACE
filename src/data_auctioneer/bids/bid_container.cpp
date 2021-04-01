#include "bid_container.h"
#include <iostream>
/*!
 * \brief Constructor
 * \param key Task key
 */
BidContainer::BidContainer(uint64_t agentID, const TaskKey &key) :
    m_agentID(agentID),
    m_taskKey(key)
{
    Data::EnvironmentTime now;
    Data::EnvironmentTime::CurrentTime(Data::SYSTEMCLOCK, now);
    m_agentUpdateTimes.insert({agentID, now});
}

/*!
 * \brief Receives a new bid. If the bid exceeds the current leader, replaces it.
 * \param sender Sender ID
 * \param senderTimestamp Sender timestamp
 * \param bid Bid received
 * \param newLeadingBid Whether the bid is the new leading bid
 * \return The action that should be taken on receive
 */
BidContainer::ReceiveAction BidContainer::ReceiveBid(uint64_t sender,
                                                     const Data::EnvironmentTime &senderTimestamp,
                                                     const BidDescriptor &bid,
                                                     bool &newLeadingBid)
{
    // If the bid is received from ourself, we never rebroadcast since we will be sending a bundle out
    // which contains the bid
    bool selfReceive = sender == m_agentID;
    uint64_t bidAgentID = bid.getAgentID();
    newLeadingBid = false;
    Data::EnvironmentTime knownSenderTime, knownBidAgentTime, bidGenerationTime, now;
    if (selfReceive)
        now = senderTimestamp;
    else
        Data::EnvironmentTime::CurrentTime(Data::SYSTEMCLOCK, now);


    bidGenerationTime = bid.getGenerationTime();

    // Check if the information being received is outdated
    if (!selfReceive)
    {
        bool hasKnownSenderTime = getAgentTime(sender, knownSenderTime);
        setAgentTime(sender, senderTimestamp);

        if (sender != bidAgentID && bidAgentID != m_agentID)
        {
            bool hasKnownBidAgentTime = getAgentTime(bidAgentID, knownBidAgentTime);
            setAgentTime(bidAgentID, bidGenerationTime);
            if (hasKnownBidAgentTime)
            {
               if (knownBidAgentTime > bidGenerationTime
                       && ((knownBidAgentTime - bidGenerationTime) > BidDescriptor::s_bidTimeEpsilon))
                {
                   // Outdated information for the bid's agent
                    if (m_valid)
                        return ReceiveAction::BroadcastLocal;
                    else
                        return ReceiveAction::BroadcastScrub;
                }
            }
        }
        // else Sender had sent their own bid, which must be up to date
    }


    if (m_valid) // We have a current leader for this task
    {
        if (m_leadingBid == bid)
            return ReceiveAction::None; // Agreement

        uint64_t currentLeader = m_leadingBid.getAgentID();
        newLeadingBid = m_leadingBid < bid;

        if (selfReceive) // Self receive
        {
            if (newLeadingBid)
            {
                m_leadingBid = bid;
                setAgentTime(m_agentID, now);
            }


            return ReceiveAction::None;
        }

        if (bidAgentID == sender) // Sender believes they are the leader
        {
            setAgentTime(sender, senderTimestamp);
            newLeadingBid = (currentLeader == sender) || newLeadingBid;
            if (newLeadingBid)
            {
                m_leadingBid = bid;
                setAgentTime(m_agentID, now);

                return ReceiveAction::BroadcastReceived; // Sender was correct
            }

            return ReceiveAction::BroadcastLocal; // We were correct, and had a different leading agent
        }
        else if (bidAgentID == m_agentID) // Sender believes we are the leader
        {
            newLeadingBid = false;

            if (currentLeader == m_agentID)
            {
                if (m_leadingBid == bid)
                {
                    return ReceiveAction::None; // Agreement
                }
                else
                {
                    return ReceiveAction::BroadcastLocal; // Sender has an outdated bid from us
                }
            }
            else if (currentLeader == sender)
            {
                m_valid = false;
                setAgentTime(m_agentID, now);
                return ReceiveAction::BroadcastScrub; // We believed the sender was the leader, so the bid is invalid
            }
            else
            {
                return ReceiveAction::BroadcastLocal; // Another agent is the leader
            }
        }
        else // Sender believes some other agent is the leader
        {
            if (currentLeader == m_agentID || currentLeader == sender)
            {
                if (newLeadingBid)
                {
                    m_leadingBid = bid;
                    setAgentTime(m_agentID, now);
                    return ReceiveAction::BroadcastReceived; // Sender is correct
                }

                return ReceiveAction::BroadcastLocal; // We are correct
            }
            else if (currentLeader == bidAgentID) // We believe the leader is the same agent as the sender
            {
                if (m_leadingBid == bid)
                {
                    return ReceiveAction::None; // Agreement
                }
                else if (m_leadingBid.getGenerationTime() < bidGenerationTime)
                {
                    newLeadingBid = true;
                    m_leadingBid = bid;
                    setAgentTime(m_agentID, now);
                    return ReceiveAction::BroadcastReceived; // We had an outdated bid
                }
                else
                {
                    newLeadingBid = false;
                    return ReceiveAction::BroadcastLocal; // Sender had an outdated bid
                }
            }
            else // We believe the leader is some other agent
            {
                if (newLeadingBid)
                {
                    if (bid.getGenerationTime() >= m_leadingBid.getGenerationTime())
                    {
                        m_leadingBid = bid;
                        setAgentTime(m_agentID, now);
                        return ReceiveAction::BroadcastReceived; // Sender has a more recent bid, so the sender is correct
                    }
                    else
                    {
                        newLeadingBid = false;
                        return ReceiveAction::BroadcastLocal;
                        // Sender has a bid that was generated before our current leading bid, so we assume we are correct
                    }
                }
                else
                {
                    if (m_leadingBid.getGenerationTime() >= bidGenerationTime) // We have a more recent bid, so we are correct
                    {
                        return ReceiveAction::BroadcastLocal;
                    }
                    else // The sender has a more recent bid, so assume the sender is correct
                    {
                        newLeadingBid = true;
                        m_leadingBid = bid;
                        setAgentTime(m_agentID, now);
                        return ReceiveAction::BroadcastReceived;
                    }
                }
            }
        }
    }
    else // We don't have a current leader
    {
        newLeadingBid = true;

        if (selfReceive) // Self receive
        {
            m_valid = true;
            m_leadingBid = bid;
            setAgentTime(m_agentID, now);
            return ReceiveAction::None;
        }

        if (bidAgentID != m_agentID) // Sender believes they or another agent is the leader
        {
            m_valid = true;
            m_leadingBid = bid;
            setAgentTime(m_agentID, now);
            return ReceiveAction::BroadcastReceived;
        }
        else // Sender believes we are the leader
        {
            newLeadingBid = false;
            return ReceiveAction::BroadcastScrub;
        }
    }
}

/*!
 * \brief Receives a scrub message
 * \param sender Sender ID
 * \param senderTimestamp
 * \param scrub
 * \return
 */
BidContainer::ReceiveAction BidContainer::ReceiveScrub(const ScrubMessage &scrub)
{
    Data::EnvironmentTime knownSenderTime;
    getAgentTime(scrub.agentID, knownSenderTime);
    if (scrub.timestamp < knownSenderTime) // This was an outdated message from the sender
        return ReceiveAction::None;

    // Update timestamp on knowledge of
    setAgentTime(scrub.agentID, scrub.timestamp);

    if (m_valid)
    {
        uint64_t leader = m_leadingBid.getAgentID();
        if (leader == m_agentID)
            return ReceiveAction::BroadcastLocal;
        else
        {
            if (leader == scrub.agentID || scrub.timestamp > m_leadingBid.getGenerationTime())
            {
                m_valid = false;
                Data::EnvironmentTime now;
                Data::EnvironmentTime::CurrentTime(Data::SYSTEMCLOCK, now);
                setAgentTime(m_agentID, now);
                return ReceiveAction::BroadcastScrub;
            }
        }
    }
    m_valid = false;
    return ReceiveAction::None;
}

/*!
 * \brief Invalidates the leading bid
 */
void BidContainer::InvalidateBid()
{
    m_valid = false;
    //    m_bundleFrom = nullptr;
}

/*!
 * \return Leading bid. Check if valid using getValid()
 */
const BidDescriptor &BidContainer::getLeadingBid() const
{
    return m_leadingBid;
}

/*!
 * \return  Leading agent ID.  Check if valid using getValid()
 */
uint64_t BidContainer::getLeadingAgent() const
{
    return m_leadingBid.getAgentID();
}

/*!
 * \return Whether the referenced bid and related data are currently valid
 */
bool BidContainer::getValid() const
{
    return m_valid;
}

/*!
 * \brief Retrieves the timestamp for knowledge about bids on the task from an agent
 * \param agentID Agent ID
 * \param timestamp Timestamp
 * \return True if
 */
bool BidContainer::getAgentTime(uint64_t agentID, Data::EnvironmentTime &timestamp) const
{
    auto it = m_agentUpdateTimes.find(agentID);
    if (it == m_agentUpdateTimes.end())
        return false;

    timestamp.setTime(it->second);
    return true;
}

/*!
 * \brief Updates the timestamp for knowledge about bids on the task from an agent
 * \details This function will only update the timestamp if it is later than the
 * currently known timestamp.
 * \param agentID Agent ID
 * \param timestamp Timestamp
 */
void BidContainer::setAgentTime(uint64_t agentID, const Data::EnvironmentTime &timestamp)
{
    auto it = m_agentUpdateTimes.find(agentID);
    if (it == m_agentUpdateTimes.end())
        m_agentUpdateTimes.insert({agentID, timestamp});
    else if (it->second < timestamp)
        it->second.setTime(timestamp);
}




