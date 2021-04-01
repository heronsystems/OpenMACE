#ifndef BID_DESCRIPTOR_H
#define BID_DESCRIPTOR_H

#include <stdint.h>

#include "data_tasks/task_key.h"


/*!
 * \brief The BidDescriptor class describes a bid for a specified task.
 */
class BidDescriptor
{

public:
    // Note: Utilities will be considered equal if the difference is less than this value
    static constexpr double s_utilityEpsilon = 0.001;
    static const uint64_t s_bidTimeEpsilon = 5 * 1000; // in microseconds

    static void PrintLayout(std::ostream &out,
                            const std::string &separator = ",",
                            const std::string &taskKeyStartSeparator = "{",
                            const std::string &taskKeyEndSeparator = "}");

    BidDescriptor();
    BidDescriptor(uint64_t agentID, const TaskKey &taskKey);

    void Print(std::ostream &out,
               bool displayValueNames = false,
               const std::string &valueSeparator = ",",
               const std::string &namesSeparator = ":",
               const std::string &taskKeyStartMarker = "{",
               const std::string &taskKeyEndMarker = "}") const;


    void setBidProperties(double work,
                          double cost,
                          double reward,
                          const Data::EnvironmentTime &generationTime,
                          bool validity);

    void EstimateUtility();

    uint64_t getAgentID() const;
    void setAgentID(const uint64_t &agentID);

    const TaskKey &getTaskKey() const;
    void setTaskKey(const TaskKey &taskKey);

    double getWork() const;
    void setWork(double work);

    double getCost() const;
    void setCost(double cost);

    double getReward() const;
    void setReward(double reward);

    void setUtility(double utility);
    double getUtility() const;

    const Data::EnvironmentTime &getGenerationTime() const;
    void setGenerationTime(const Data::EnvironmentTime &generationTime);

    bool getValidity() const;
    void setValidity(bool getValidity);

    int8_t getPriority() const;
    void setPriority(const int8_t &priority);

private:
    uint64_t m_agentID;
    TaskKey  m_taskKey;
    double   m_work;
    double   m_cost;
    double   m_reward;
    double   m_utility;
    Data::EnvironmentTime m_generationTime;
    int8_t   m_priority; // Future use
    bool     m_validity;

    friend bool operator<(const BidDescriptor &bid1, const BidDescriptor &bid2);
    friend bool operator==(const BidDescriptor &bid1, const BidDescriptor &bid2);
};

inline bool operator<(const BidDescriptor &bid1, const BidDescriptor &bid2)
{
    if (operator==(bid1, bid2))
        return false;

    if (fabs(bid1.m_utility - bid2.m_utility) < BidDescriptor::s_utilityEpsilon)
    {
        if (bid1.m_agentID == bid2.m_agentID)
        {
            if (fabs(bid1.m_generationTime - bid2.m_generationTime) > BidDescriptor::s_bidTimeEpsilon)
                return bid1.m_generationTime < bid2.m_generationTime;
            else
                return false;
        }

        return bid1.m_agentID < bid2.m_agentID;
    }

    return bid1.m_utility < bid2.m_utility;
}
inline bool operator>(const BidDescriptor &bid1, const BidDescriptor &bid2)
{
    return operator<(bid2, bid1);
}
inline bool operator<=(const BidDescriptor &bid1, const BidDescriptor &bid2)
{
    return !operator<(bid2, bid1);
}
inline bool operator>=(const BidDescriptor &bid1, const BidDescriptor &bid2)
{
    return !operator<(bid1, bid2);
}

inline bool operator==(const BidDescriptor &bid1, const BidDescriptor &bid2)
{
    return (fabs(bid1.m_utility - bid2.m_utility) < BidDescriptor::s_utilityEpsilon)
            && bid1.m_agentID == bid2.m_agentID
            && bid1.m_taskKey == bid2.m_taskKey
            && fabs(bid1.m_generationTime - bid2.m_generationTime) < BidDescriptor::s_bidTimeEpsilon;
}
inline bool operator!=(const BidDescriptor &bid1, const BidDescriptor &bid2)
{
    return !operator==(bid1, bid2);
}

// std::hash specialization for BidDescriptor
namespace std
{
template <>
struct hash<BidDescriptor>
{
    size_t operator() (const BidDescriptor &bid) const
    {
        size_t h0, h1, h2;
        h0 = hash<TaskKey>{}(bid.getTaskKey());
        h1 = hash<uint64_t>{}(bid.getAgentID());
        h2 = hash<double>{}(bid.getGenerationTime().ToSecSinceEpoch());
        return (h0 ^ (h1 << 1)) ^ (h2 << 1);
    }
};
}

// Generic stream output
inline std::ostream &operator<<(std::ostream &out, const BidDescriptor& descriptor)
{
    descriptor.Print(out);
    return out;
}



#endif // BID_DESCRIPTOR_H
