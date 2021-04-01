#include "bid_descriptor.h"

#include <algorithm>
#include <ostream>

/*!
 * \brief Layout of values from the Print() function
 * \param out Output stream
 * \param separator Value separator
 * \param subtypeStartMarker Denotes the start of an embedded type
 * \param subtypeEndMarker Denotes the end of an embedded type
 */
void BidDescriptor::PrintLayout(std::ostream &out,
                                const std::string &separator,
                                const std::string &taskKeyStartSeparator,
                                const std::string &taskKeyEndSeparator)
{
    out << "BidDescriptor" << separator
        << "agentID" << separator
        << "taskKey" << taskKeyStartSeparator;
    TaskKey::PrintLayout(out, separator);
    out << taskKeyEndSeparator << separator
        << "work" << separator
        << "cost" << separator
        << "reward" << separator
        << "utility" << separator
        << "generationTime" << separator
        << "priority" << separator
        << "validity" << separator;
}

/*!
 * \brief Default constructor
 */
BidDescriptor::BidDescriptor() :
    m_agentID(0),
    m_work(0.0),
    m_cost(std::numeric_limits<double>::infinity()),
    m_reward(-m_cost),
    m_utility(-m_cost),
    m_priority(0),
    m_validity(false)

{

}

/*!
 * \brief Constructor
 * \param agentID Agent ID
 * \param taskKey Task key
 */
BidDescriptor::BidDescriptor(uint64_t agentID, const TaskKey &taskKey) :
    m_agentID(agentID),
    m_taskKey(taskKey),
    m_work(0.0),
    m_cost(std::numeric_limits<double>::infinity()),
    m_reward(-m_cost),
    m_utility(-m_cost),
    m_priority(0),
    m_validity(true)
{
}


/*!
 * \brief Prints to stream
 * \param out Output stream
 * \param displayValueNames Whether variable names are printed
 * \param valueSeparator Value separator
 * \param namesSeparator Separator between variable name and value
 * \param description Object description
 * \param taskKeyStartMarker Denotes the start of an embedded type
 * \param taskKeyEndMarker Denotes the end of an embedded type
 */
void BidDescriptor::Print(std::ostream &out,
                          bool displayValueNames,
                          const std::string &valueSeparator,
                          const std::string &namesSeparator,
                          const std::string &taskKeyStartMarker,
                          const std::string &taskKeyEndMarker) const
{
    out << "BidDescriptor" << valueSeparator;

    if (displayValueNames)
        out << "agentID" << namesSeparator;
    out << m_agentID << valueSeparator;

    if (displayValueNames)
        out << "taskKey" << namesSeparator;
    out << taskKeyStartMarker;
    m_taskKey.Print(out, displayValueNames, valueSeparator, namesSeparator);
    out << taskKeyEndMarker << valueSeparator;

    if (displayValueNames)
        out << "work" << namesSeparator;
    out << m_work << valueSeparator;

    if (displayValueNames)
        out << "cost" << namesSeparator;
    out << m_cost << valueSeparator;

    if (displayValueNames)
        out << "reward" << namesSeparator;
    out << m_reward << valueSeparator;

    if (displayValueNames)
        out << "utility" << namesSeparator;
    out << m_utility << valueSeparator;

    if (displayValueNames)
        out << "generationTime" << namesSeparator;
    out << m_generationTime.ToString().toStdString() << valueSeparator;

    if (displayValueNames)
        out << "priority" << namesSeparator;
    out << (int) m_priority << valueSeparator;

    if (displayValueNames)
        out << "validity" << namesSeparator;
    out << m_validity;
}

/*!
 * \brief Sets properties for the bid.
 * \param work Work
 * \param cost Cost
 * \param reward reward
 * \param generationTime Bid generation time
 * \param validity validity
 */
void BidDescriptor::setBidProperties(double work,
                                     double cost,
                                     double reward,
                                     const Data::EnvironmentTime &generationTime,
                                     bool validity)
{
    m_work = work;
    m_cost = cost;
    m_reward = reward;
    m_generationTime = generationTime;
    m_validity = validity;
}

/*!
 * \brief Estimates the bid utility
 */
void BidDescriptor::EstimateUtility()
{
    m_utility = m_reward - m_cost;
    if (m_utility < 0)
        m_utility = 0.0;
}

/*!
 * \return Agent ID that created this bid
 */
uint64_t BidDescriptor::getAgentID() const
{
    return m_agentID;
}

/*!
 * \param agentID Agent ID that created this bid
 */
void BidDescriptor::setAgentID(const uint64_t &agentID)
{
    m_agentID = agentID;
}

/*!
 * \return Task key
 */
const TaskKey &BidDescriptor::getTaskKey() const
{
    return m_taskKey;
}

/*!
 * \param taskKey Task key
 */
void BidDescriptor::setTaskKey(const TaskKey &taskKey)
{
    m_taskKey = taskKey;
}

/*!
 * \return Work
 */
double BidDescriptor::getWork() const
{
    return m_work;
}

/*!
 * \param work Work
 */
void BidDescriptor::setWork(double work)
{
    m_work = work;
}

/*!
 * \return Cost
 */
double BidDescriptor::getCost() const
{
    return m_cost;
}

/*!
 * \param cost Cost
 */
void BidDescriptor::setCost(double cost)
{
    m_cost = cost;
}

/*!
 * \return Reward
 */
double BidDescriptor::getReward() const
{
    return m_reward;
}

/*!
 * \param reward Reward
 */
void BidDescriptor::setReward(double reward)
{
    m_reward = reward;
}

/*!
 * \param utility Utility
 */
void BidDescriptor::setUtility(double utility)
{
    m_utility = utility;
}

/*!
 * \return Utility
 */
double BidDescriptor::getUtility() const
{
    return m_utility;
}

/*!
 * \return Generation time
 */
const Data::EnvironmentTime &BidDescriptor::getGenerationTime() const
{
    return m_generationTime;
}

/*!
 * \param generationTime Generation time
 */
void BidDescriptor::setGenerationTime(const Data::EnvironmentTime &generationTime)
{
    m_generationTime = generationTime;
}

/*!
 * \return Validity
 */
bool BidDescriptor::getValidity() const
{
    return m_validity;
}

/*!
 * \param validity Validity
 */
void BidDescriptor::setValidity(bool validity)
{
    m_validity = validity;
}

/*!
 * \return Priority
 */
int8_t BidDescriptor::getPriority() const
{
    return m_priority;
}

/*!
 * \param priority Priority
 */
void BidDescriptor::setPriority(const int8_t &priority)
{
    m_priority = priority;
}

