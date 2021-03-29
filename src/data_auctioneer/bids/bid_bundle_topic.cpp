#include "bid_bundle_topic.h"

const char g_bidBundleName[] = "bid_bundle";
const MaceCore::TopicComponentStructure g_bidBundleStructure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<BidBundle>("Bid Bundle");

    return structure;
}();

/*!
 * \brief Generates a datagram from underlying data
 * \return Generated datagram
 */
MaceCore::TopicDatagram BidBundleTopic::GenerateDatagram() const
{
    MaceCore::TopicDatagram datagram;

    datagram.AddTerminal<BidBundle>("Bid Bundle", m_bundle);
    return datagram;
}


/*!
 * \brief Creates underyling data from a datagram
 * \param datagram Datagram to create from
 */
void BidBundleTopic::CreateFromDatagram(const MaceCore::TopicDatagram &datagram)
{
    m_bundle = datagram.GetTerminal<BidBundle>("Bid Bundle");
}

/*!
 * \return Bid bundle
 */
const BidBundle &BidBundleTopic::getBundle() const
{
    return m_bundle;
}

/*!
 * \param bundle Bid bundle
 */
void BidBundleTopic::setBundle(const BidBundle &bundle)
{
    m_bundle = bundle;
}

