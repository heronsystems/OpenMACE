#include "bid_descriptor_topic.h"

const char g_bidDescriptorName[] = "bid_descriptor";
const MaceCore::TopicComponentStructure g_bidDescriptorStructure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<BidDescriptor>("Bid Descriptor");
    structure.AddTerminal<Data::EnvironmentTime>("Timestamp");

    return structure;
}();

/*!
 * \brief Generates a datagram from underlying data
 * \return Generated datagram
 */
MaceCore::TopicDatagram BidDescriptorTopic::GenerateDatagram() const
{
    MaceCore::TopicDatagram datagram;

    datagram.AddTerminal<BidDescriptor>("Bid Descriptor", m_descriptor);
    datagram.AddTerminal<Data::EnvironmentTime>("Timestamp", m_timestamp);
    return datagram;
}

/*!
 * \brief Creates underyling data from a datagram
 * \param datagram Datagram to create from
 */
void BidDescriptorTopic::CreateFromDatagram(const MaceCore::TopicDatagram &datagram)
{
    m_descriptor = datagram.GetTerminal<BidDescriptor>("Bid Descriptor");
    m_timestamp = datagram.GetTerminal<Data::EnvironmentTime>("Timestamp");
}

BidDescriptor BidDescriptorTopic::getDescriptor() const
{
    return m_descriptor;
}

void BidDescriptorTopic::setDescriptor(const BidDescriptor &descriptor)
{
    m_descriptor = descriptor;
}

const Data::EnvironmentTime &BidDescriptorTopic::getTimestamp() const
{
    return m_timestamp;
}

void BidDescriptorTopic::setTimestamp(const Data::EnvironmentTime &timestamp)
{
    m_timestamp = timestamp;
}

