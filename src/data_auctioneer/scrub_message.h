#ifndef SCRUB_MESSAGE_H
#define SCRUB_MESSAGE_H

#include "data/environment_time.h"
#include "data_tasks/task_key.h"

/*!
 * \brief The ScrubMessage struct is used to clear a bid for a particular task from a
 * particular agent.
 */
typedef struct ScrubMessage
{
    uint64_t agentID;
    TaskKey taskKey;
    Data::EnvironmentTime timestamp;
} ScrubMessage;

inline bool operator ==(const ScrubMessage &lhs, const ScrubMessage &rhs)
{
    return lhs.agentID == rhs.agentID
            && lhs.taskKey == rhs.taskKey
            && lhs.timestamp == rhs.timestamp;
}

inline bool operator !=(const ScrubMessage &lhs, const ScrubMessage &rhs)
{
    return !(lhs == rhs);
}


// std::hash specialization for ScrubMessage
namespace std
{
template <>
struct hash<ScrubMessage>
{
    size_t operator() (const ScrubMessage &scrub) const
    {
        size_t h0, h1, h2;
        h0 = hash<TaskKey>{}(scrub.taskKey);
        h1 = hash<uint64_t>{}(scrub.agentID);
        h2 = hash<double>{}(scrub.timestamp.ToSecSinceEpoch());
        return (h0 ^ (h1 << 1)) ^ (h2 << 1);
    }
};
}

#endif // SCRUB_MESSAGE_H
