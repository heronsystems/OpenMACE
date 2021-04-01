#ifndef BID_BUNDLE_H
#define BID_BUNDLE_H

#include <vector>
#include <memory>
#include <ostream>

#include "data_auctioneer/state/vehicle_state.h"

#include "bid_descriptor.h"


/*!
 * \brief The BundledBidDescriptor struct contains a BidDescriptor and state info used
 * when constructing a bundle.
 */
typedef struct BundledBidDescriptor
{
    BidDescriptor   descriptor;  /*!< Bid descriptor */
    VehicleStatePtr finalState; /*!< Final State after task is performed. Used by bundle creator */
} BundledBidDescriptor;

/*!
 * \brief The BidBundle class defines a bid bundle representing tasks an agent will bid on
 * in an auction.
 */
class BidBundle
{
public:
    static const int        s_bundleSizeLimit = 3;
    static constexpr double s_minBidEpsilon = 0.1;

    typedef std::vector<BundledBidDescriptor>::iterator iterator; /*!< Bundle iterator. Random access. */
    typedef std::vector<BundledBidDescriptor>::const_iterator const_iterator; /*!< Constant bundle iterator. Random access. */

    static void PrintLayout(std::ostream &out,
                            const std::string &separator = ",",
                            const std::string &subtypeStartMarker = "{",
                            const std::string &subtypeEndMarker = "}",
                            const std::string &varSizeMarker = "...");

    BidBundle();

    void Print(std::ostream &out,
               bool displayValueNames = false,
               bool bidsOnNewLine = false,
               const std::string &valueSeparator = ",",
               const std::string &namesSeparator = ":",
               const std::string &subtypeStartMarker = "{",
               const std::string &subtypeEndMarker = "}") const;

    double Workload();
    int Size() const;

    bool AppendBid(const BidDescriptor &descriptor,
                   const VehicleStatePtr &finalState = nullptr);

    void AppendBundle(const BidBundle &other);

    BundledBidDescriptor &At(int index);
    const BundledBidDescriptor &At(int index) const;

    iterator Begin();
    const_iterator Begin() const;

    iterator End();
    const_iterator End() const;

    iterator Erase(const_iterator pos);

    iterator Erase(const_iterator first, const_iterator last);

    BundledBidDescriptor &Front();
    const BundledBidDescriptor &Front() const;

    BundledBidDescriptor &Back();
    const BundledBidDescriptor &Back() const;

    bool Empty() const;

    const Data::EnvironmentTime &getGenTime() const;
    void setGenTime(const Data::EnvironmentTime &genTime);

    void Clear();

    uint64_t getAgentID() const;

private:
    std::vector<BundledBidDescriptor> m_bundle;
    Data::EnvironmentTime m_genTime;
    uint64_t              m_agentID;
};

// Generic stream output
inline std::ostream &operator<<(std::ostream &out, const BidBundle& bundle)
{
    bundle.Print(out);
    return out;
}

#endif // BID_BUNDLE_H
