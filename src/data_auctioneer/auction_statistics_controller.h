#ifndef AUCTION_MESSAGE_STATISTICS_H
#define AUCTION_MESSAGE_STATISTICS_H

#include <ostream>
#include <mutex>

#include "common/watchdog.h"

#include "data_auctioneer/auction_logger.h"

/*!
 * \brief The AuctionExternalLinkStatistics struct is used to track basic statistics of auction messages sent/received
 * at an external link.
 * \details The statistics include the total number of Mavlink messages sent/received used to send/receive auction messages.
 */
class AuctionStatistics_Controller
{
public:
    static void PrintLayout(std::ostream &out, const std::string &separator = ",");

    AuctionStatistics_Controller();
    AuctionStatistics_Controller(const AuctionStatistics_Controller &other);



    void Print(std::ostream &out,
               bool displayValueNames = true,
               const std::string &valueSeparator = ",",
               const std::string &nameSeparator = ":") const;

    AuctionStatistics_Controller operator +(const AuctionStatistics_Controller &other);

    void operator +=(const AuctionStatistics_Controller &other);

    void TrackAuctionMessageSent();
    void TrackAuctionMessageRcv();
    void TrackMavlinkMessageSent(int bytes);
    void TrackMavlinkMessageRcv(int bytes);
    void TrackFailure();

    void ResetModifiedFlag();

    bool getModified() const;

    void StartWatchdog();

private:
    int m_numMessagesSentTotal_Auction = 0; // Number of messages sent/received by the Auctioneer algorithm
    int m_numMessagesRcvTotal_Auction = 0;

    // Number of messages sent/received due to Mavlink protocol. Does not count retransmission
    int m_numMessagesSentTotal_Mavlink = 0;
    int m_numMessagesRcvTotal_Mavlink = 0;

    int m_numFailuresTotal = 0;

    // Messages sent since the last modified reset
    int m_numMessagesSentPeriod_Auction = 0;
    int m_numMessagesRcvPeriod_Auction = 0;

    int m_numMessagesSentPeriod_Mavlink = 0;
    int m_numMessagesRcvPeriod_Mavlink = 0;

    int m_numFailuresPeriod = 0;

    // In current window

    int m_bytesSent = 0;
    int m_bytesRcv = 0;
    double m_bytesPerSecondSent = 0;
    double m_bytesPerSecondRcv = 0;
    double m_auctionSendFrequency = 0;
    double m_auctionRcvFrequency = 0;
    double m_mavlinkSendFrequency = 0;
    double m_mavlinkRcvFrequency = 0;
    double m_failureRatio = 0;

    bool m_modified = false;

    std::unique_ptr<ContinuousWatchdog> m_ratesWatchdog;

    void CalculateRates(double duration);
    mutable std::mutex m_mutex;
};



#endif // AUCTION_MESSAGE_STATISTICS_H
