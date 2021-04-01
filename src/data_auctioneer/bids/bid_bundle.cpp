#include "bid_bundle.h"


/*!
 * \brief Layout of values from the Print() function
 * \param out Output stream
 * \param separator Value separator
 * \param subtypeStartMarker Denotes the start of an embedded type
 * \param subtypeEndMarker Denotes the end of an embedded type
 * \param varSizeMarker Denotes that the previous value is a variable sized list
 */
void BidBundle::PrintLayout(std::ostream &out,
                            const std::string &separator,
                            const std::string &subtypeStartMarker,
                            const std::string &subtypeEndMarker,
                            const std::string &varSizeMarker)
{
    out << "BidBundle" << separator
        << "generationTime" << separator
        << "size" << separator
        << "bundle" << subtypeStartMarker;
    BidDescriptor::PrintLayout(out, separator, subtypeStartMarker, subtypeEndMarker);
    out << subtypeEndMarker << separator << varSizeMarker;
}

/*!
 * \brief Constructor
 */
BidBundle::BidBundle() :
    m_bundle()
{

}

/*!
 * \brief Prints to stream
 * \param out Output stream
 * \param displayValueNames Whether variable names are printed
 * \param bidsOnNewLine Whether bids should be printed on a new line
 * \param valueSeparator Value separator
 * \param namesSeparator Separator between variable name and value
 * \param description Object description
 * \param subtypeStartMarker Denotes the start of an embedded type
 * \param subtypeEndMarker Denotes the end of an embedded type
 */
void BidBundle::Print(std::ostream &out,
                      bool displayValueNames,
                      bool bidsOnNewLine,
                      const std::string &valueSeparator,
                      const std::string &namesSeparator,
                      const std::string &subtypeStartMarker,
                      const std::string &subtypeEndMarker) const
{
    static const std::string baseBidDescription = "BidDescription_";

    int size = Size();

    out << "BidBundle" << valueSeparator;

    if (displayValueNames)
        out << "generationTime" << namesSeparator;
    out << m_genTime.ToString().toStdString() << valueSeparator;

    if (displayValueNames)
        out << "size" << namesSeparator;
    out << Size() << valueSeparator;

    int bidIndex = 0;
    for (const BundledBidDescriptor &bundledBid: m_bundle)
    {
        if (bidsOnNewLine)
            out << std::endl;

        if (displayValueNames)
            out << baseBidDescription << bidIndex << namesSeparator;

        out << subtypeStartMarker;
        bundledBid.descriptor.Print(out, displayValueNames, valueSeparator,
                                    namesSeparator, subtypeStartMarker, subtypeEndMarker);
        out << subtypeEndMarker;

        if (bidIndex < size - 1)
            out << valueSeparator;

        ++bidIndex;
    }
}

// TODO unimplemented. Currently does nothing in Matlab version
double BidBundle::Workload()
{
    return 0.0;
}

/*!
 * \return Bundle size
 */
int BidBundle::Size() const
{
    return m_bundle.size();
}

/*!
 * \brief Appends a bid
 * \param descriptor Bid descriptor
 * \param finalState Final State (when building a bundle)
 * \return true if the bid can be appended, false if the bundle size exceeds the size limit
 */
bool BidBundle::AppendBid(const BidDescriptor &descriptor,
                          const VehicleStatePtr &finalState)
{

    if (m_bundle.size() >= s_bundleSizeLimit)
        return false;
    else if (m_bundle.size() > 0)
    {
        if (descriptor.getAgentID() != m_agentID)
            return false;
    }
    else
    {
        m_agentID = descriptor.getAgentID();
    }

    BundledBidDescriptor bundledBid;
    bundledBid.descriptor = descriptor;
    bundledBid.finalState = finalState;
    m_bundle.push_back(bundledBid);
    return true;
}

/*!
 * \brief Appends another bid bundle
 * \details This function is used to append bundles together when
 * \param other Bid bundle being appended
 */
void BidBundle::AppendBundle(const BidBundle &other)
{
    if (other.Empty())
        return;
    else if (this->Empty())
        this->m_agentID = other.m_agentID;
    else if (this->m_agentID != other.m_agentID)
        throw std::runtime_error("Attempted to append bundles with differing agent IDs");

//    m_bundle.insert(this->m_bundle.end(), other.m_bundle.begin(), other.m_bundle.end());
    BundledBidDescriptor bundledBid;
    auto it = other.Begin();
    while (it != other.End())
    {
        bundledBid.descriptor = it->descriptor;
        m_bundle.push_back(bundledBid);
        ++it;
    }
}

/*!
 * \brief Retrieves the bid bundle info (descriptor, final State) at index
 * \param index Index
 * \return Bid bundle info at index
 */
BundledBidDescriptor &BidBundle::At(int index)
{
    return m_bundle.at(index);
}
/*!
 * \overload
 */
const BundledBidDescriptor &BidBundle::At(int index) const
{
    return m_bundle.at(index);
}

/*!
 * \return begin iterator
 */
BidBundle::iterator BidBundle::Begin()
{
    return m_bundle.begin();
}

/*!
 * \overload
 */
BidBundle::const_iterator BidBundle::Begin() const
{
    return m_bundle.cbegin();
}

/*!
 * \return end iterator
 */
BidBundle::iterator BidBundle::End()
{
    return m_bundle.end();
}

/*!
 * \overload
 */
BidBundle::const_iterator BidBundle::End() const
{
    return m_bundle.cend();
}

/*!
 * \brief Erase item in bundle at iterator position
 * \param pos Erase position
 * \return incremented iterator
 */
BidBundle::iterator BidBundle::Erase(BidBundle::const_iterator pos)
{
    return m_bundle.erase(pos);
}

/*!
 * \brief Erase items in bundle from first to last (non-inclusive) iterators
 * \param first Position to start erase at
 * \param last Position to end erase at
 * \return incremented iterator
 */
BidBundle::iterator BidBundle::Erase(BidBundle::const_iterator first, BidBundle::const_iterator last)
{
    return m_bundle.erase(first, last);
}

/*!
 * \return Bid bundle info at front (descriptor, final State)
 */
BundledBidDescriptor &BidBundle::Front()
{
    return m_bundle.front();
}

/*!
 * \overload
 */
const BundledBidDescriptor &BidBundle::Front() const
{
    return m_bundle.front();
}

/*!
 * \return Bid bundle info at back (descriptor, final State)
 */
BundledBidDescriptor &BidBundle::Back()
{
    return m_bundle.back();
}

/*!
 * \overload
 */
const BundledBidDescriptor &BidBundle::Back() const
{
    return m_bundle.back();
}

/*!
 * \return Whether the bundle is empty
 */
bool BidBundle::Empty() const
{
    return m_bundle.empty();
}

/*!
 * \return Bundle generation time
 */
const Data::EnvironmentTime &BidBundle::getGenTime() const
{
    return m_genTime;
}

/*!
 * \param genTime bundle generation time
 */
void BidBundle::setGenTime(const Data::EnvironmentTime &genTime)
{
    m_genTime = genTime;
}

/*!
 * \brief Clears the bundle
 */
void BidBundle::Clear()
{
    m_bundle.clear();
}

/*!
 * \brief Returns the agent ID associated to this bundle. Valid only if the bundle is not empty.
 * \return Agent ID
 */
uint64_t BidBundle::getAgentID() const
{
    return m_agentID;
}
