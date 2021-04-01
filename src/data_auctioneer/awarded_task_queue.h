#ifndef AWARDED_TASK_QUEUE_H
#define AWARDED_TASK_QUEUE_H

#include <unordered_map>
#include <ostream>

#include "data_tasks/task_key.h"
#include "data_auctioneer/bids/bid_descriptor.h"


/*!
 * \brief The AwardedTaskParams struct contains parameters about a task the host agent
 * was awarded during an auction.
 */
typedef struct AwardedTaskParams
{
    // NOTE: Currently only the task key, agent ID, and consensus reached values are being used
    TaskKey       taskKey;          /*!< Task key */
    BidDescriptor worstCaseBid;     /*!< Worst case bid */
    uint64_t      awardedAgentID;   /*!< Awarded agent ID */
    uint64_t      bidValidTime;     /*!< Bid valid time */
    bool          consensusReached; /*!< Whether consensus was reached */

    static void PrintLayout(std::ostream &out,
                            const std::string &separator = ",",
                            const std::string &subtypeStartMarker = "{",
                            const std::string &subtypeEndMarker = "}");

    void Print(std::ostream &out,
               bool displayValueNames = false,
               const std::string &valueSeparator = ",",
               const std::string &namesSeparator = ":",
               const std::string &subtypeStartMarker = "{",
               const std::string &subtypeEndMarker = "}") const;
} AwardedTaskParams;

/*!
 * \brief The AwardedTaskQueue class stores information about tasks that have
 * been awarded to an agent
 */
class AwardedTaskQueue : public std::unordered_map<TaskKey, AwardedTaskParams>
{
public:
    static void PrintLayout(std::ostream &out,
                            const std::string &separator = ",",
                            const std::string &subtypeStartMarker = "{",
                            const std::string &subtypeEndMarker = "}",
                            const std::string &varSizeMarker = "...");

    AwardedTaskQueue();

    void Print(std::ostream &out,
               bool displayValueNames = false,
               bool paramsOnNewLine = false,
               const std::string &valueSeparator = ",",
               const std::string &namesSeparator = ":",
               const std::string &subtypeStartMarker = "{",
               const std::string &subtypeEndMarker = "}") const;
};

// Generic stream output
inline std::ostream &operator<<(std::ostream &out, const AwardedTaskQueue& queue)
{
    queue.Print(out);
    return out;
}

#endif // AWARDED_TASK_QUEUE_H
