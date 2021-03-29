#include "auction_statistics_controller.h"

#include <regex>


void AuctionStatistics_Controller::PrintLayout(std::ostream &out, const std::string &separator)
{
    out << "AuctionControllerStatistics" << separator
        << "MessagesSent_Auction" << separator
        << "MessagesRcv_Auction" << separator
        << "MessagesSent_Mavlink" << separator
        << "MessagesRcv_Mavlink" << separator
        << "MessagesFailed" << separator
        << "BandwidthSendEstimate" << separator
        << "BandwidthRcvEstimate" << separator
        << "CurrentAuctionSendFrequency" << separator
        << "CurrentAuctionRcvFrequency" << separator
        << "CurrentMavlinkSendFrequency" << separator
        << "CurrentMavlinkRcvFrequency" << separator
        << "CurrentFailurePercent";
}

AuctionStatistics_Controller::AuctionStatistics_Controller()
{
    StartWatchdog();

}

AuctionStatistics_Controller::AuctionStatistics_Controller(const AuctionStatistics_Controller &other)
{
    m_numMessagesSentTotal_Auction = other.m_numMessagesSentTotal_Auction;
    m_numMessagesRcvTotal_Auction = other.m_numMessagesRcvTotal_Auction;
    m_numMessagesSentTotal_Mavlink = other.m_numMessagesSentTotal_Mavlink;
    m_numMessagesRcvTotal_Mavlink = other.m_numMessagesRcvTotal_Mavlink;
    m_numFailuresTotal = other.m_numFailuresTotal;

    m_numMessagesSentPeriod_Auction = other.m_numMessagesSentPeriod_Auction;
    m_numMessagesRcvPeriod_Auction = other.m_numMessagesRcvPeriod_Auction;
    m_numMessagesSentPeriod_Mavlink = other.m_numMessagesSentPeriod_Mavlink;
    m_numMessagesRcvPeriod_Mavlink = other.m_numMessagesRcvPeriod_Mavlink;
    m_numFailuresPeriod = other.m_numFailuresPeriod;

    m_bytesSent = other.m_bytesSent;
    m_bytesRcv = other.m_bytesRcv;
    m_bytesPerSecondSent = other.m_bytesPerSecondSent;
    m_bytesPerSecondRcv = other.m_bytesPerSecondRcv;
    m_auctionSendFrequency = other.m_auctionSendFrequency;
    m_auctionRcvFrequency = other.m_auctionRcvFrequency;
    m_mavlinkSendFrequency = other.m_mavlinkSendFrequency;
    m_mavlinkRcvFrequency = other.m_mavlinkRcvFrequency;

    m_modified = other.m_modified;
}


void AuctionStatistics_Controller::Print(std::ostream &out, bool displayValueNames, const std::string &valueSeparator, const std::string &nameSeparator) const
{
    std::lock_guard<std::mutex> lock(m_mutex);

    out << "AuctionControllerStatistics" << valueSeparator;

    if (displayValueNames)
        out << "MessagesSent_Auction" << nameSeparator;
    out << m_numMessagesSentTotal_Auction << valueSeparator;

    if (displayValueNames)
        out << "MessagesRcv_Auction" << nameSeparator;
    out << m_numMessagesRcvTotal_Auction << valueSeparator;

    if (displayValueNames)
        out << "MessagesSent_Mavlink" << nameSeparator;
    out << m_numMessagesSentTotal_Mavlink << valueSeparator;

    if (displayValueNames)
        out << "MessagesRcv_Mavlink" << nameSeparator;
    out << m_numMessagesRcvTotal_Mavlink << valueSeparator;

    if (displayValueNames)
        out << "MessagesFailed" << nameSeparator;
    out << m_numFailuresTotal << valueSeparator;

    if (displayValueNames)
        out << "BandwidthSendEstimate" << nameSeparator;

    if (m_bytesPerSecondSent > 1000 * 1000)
        out << (m_bytesPerSecondSent / 1000 * 1000) << "MB/s" << valueSeparator;
    else if (m_bytesPerSecondSent > 1000)
        out << (m_bytesPerSecondSent / 1000) << "KB/s" << valueSeparator;
    else
        out << m_bytesPerSecondSent << "B/s" << valueSeparator;

    if (displayValueNames)
        out << "BandwidthRcvEstimate" << nameSeparator;

    if (m_bytesPerSecondRcv > 1000 * 1000)
        out << (m_bytesPerSecondRcv / 1000 * 1000) << "MB/s" << valueSeparator;
    else if (m_bytesPerSecondRcv > 1000)
        out << (m_bytesPerSecondRcv / 1000) << "KB/s" << valueSeparator;
    else
        out << m_bytesPerSecondRcv << "B/s" << valueSeparator;

    if (displayValueNames)
        out << "CurrentAuctionSendFrequency" << nameSeparator;
    out << m_auctionSendFrequency << valueSeparator;

    if (displayValueNames)
       out << "CurrentAuctionRcvFrequency" << nameSeparator;
    out << m_auctionRcvFrequency << valueSeparator;


    if (displayValueNames)
       out << "CurrentMavlinkSendFrequency" << nameSeparator;
    out << m_mavlinkSendFrequency << valueSeparator;

    if (displayValueNames)
       out << "CurrentMavlinkRcvFrequency" << nameSeparator;
    out << m_mavlinkRcvFrequency << valueSeparator;

    if (displayValueNames)
       out << "CurrentFailurePercent" << nameSeparator;
    out << (m_failureRatio * 100);

}

AuctionStatistics_Controller AuctionStatistics_Controller::operator +(const AuctionStatistics_Controller &other)
{
    AuctionStatistics_Controller result = *this;
    result += other;

    return result;
}

void AuctionStatistics_Controller::operator +=(const AuctionStatistics_Controller &other)
{
    m_numMessagesRcvTotal_Auction += other.m_numMessagesRcvTotal_Auction;
    m_numMessagesSentTotal_Auction += other.m_numMessagesSentTotal_Auction;
    m_numMessagesRcvTotal_Mavlink += other.m_numMessagesRcvTotal_Mavlink;
    m_numMessagesSentTotal_Mavlink += other.m_numMessagesSentTotal_Mavlink;
    m_numFailuresTotal += other.m_numFailuresTotal;

    m_numMessagesRcvPeriod_Auction += other.m_numMessagesRcvPeriod_Auction;
    m_numMessagesSentPeriod_Auction += other.m_numMessagesSentPeriod_Auction;
    m_numMessagesRcvPeriod_Mavlink += other.m_numMessagesRcvPeriod_Mavlink;
    m_numMessagesSentPeriod_Mavlink += other.m_numMessagesSentPeriod_Mavlink;
    m_numFailuresPeriod += other.m_numFailuresPeriod;

   m_bytesRcv += other.m_bytesRcv;
   m_bytesSent += other.m_bytesSent;

   m_bytesPerSecondRcv += other.m_bytesPerSecondRcv;
   m_bytesPerSecondSent += other.m_bytesPerSecondSent;


   double overallMavlinkFreq = m_mavlinkRcvFrequency + m_mavlinkSendFrequency;
   double otherOverallMavlinkFreq = other.m_mavlinkRcvFrequency + other.m_mavlinkSendFrequency;


   m_auctionSendFrequency += other.m_auctionSendFrequency;
   m_auctionRcvFrequency += other.m_auctionRcvFrequency;
   m_mavlinkSendFrequency += other.m_mavlinkSendFrequency;
   m_mavlinkRcvFrequency += other.m_mavlinkRcvFrequency;

   double failureFreq = m_failureRatio * overallMavlinkFreq;
   double otherFailureFreq = other.m_failureRatio * otherOverallMavlinkFreq;

   overallMavlinkFreq += otherFailureFreq;

   if (overallMavlinkFreq > 0)
       m_failureRatio = failureFreq / overallMavlinkFreq;
   else
       m_failureRatio = 0;
}

void AuctionStatistics_Controller::TrackAuctionMessageSent()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    ++m_numMessagesSentTotal_Auction;
    ++m_numMessagesSentPeriod_Auction;
    m_modified = true;
}

void AuctionStatistics_Controller::TrackAuctionMessageRcv()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    ++m_numMessagesRcvTotal_Auction;
    ++m_numMessagesRcvPeriod_Auction;
    m_modified = true;
}

void AuctionStatistics_Controller::TrackMavlinkMessageSent(int bytes)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    ++m_numMessagesSentTotal_Mavlink;
    ++m_numMessagesSentPeriod_Mavlink;
    m_bytesSent += bytes;
    m_modified = true;
}

void AuctionStatistics_Controller::TrackMavlinkMessageRcv(int bytes)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    ++m_numMessagesRcvTotal_Mavlink;
    ++m_numMessagesRcvPeriod_Mavlink;
    m_bytesRcv += bytes;
    m_modified = true;
}

void AuctionStatistics_Controller::TrackFailure()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    ++m_numFailuresTotal;
    ++m_numFailuresPeriod;
    m_modified = true;
}

void AuctionStatistics_Controller::ResetModifiedFlag()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_modified = false;
}

bool AuctionStatistics_Controller::getModified() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_modified;
}

void AuctionStatistics_Controller::StartWatchdog()
{
    if (m_ratesWatchdog)
        return;

    auto lambda = [&]()
    {
        CalculateRates(0.1);
        return false;
    };
    m_ratesWatchdog.reset(new ContinuousWatchdog(std::chrono::milliseconds(100), lambda));
}

void AuctionStatistics_Controller::CalculateRates(double duration)
{
    std::lock_guard<std::mutex> lock(m_mutex);

    // Moving average
    m_bytesPerSecondSent = m_bytesPerSecondSent * 0.95 + 0.05 * (((double) m_bytesSent) / duration);
    m_bytesPerSecondRcv = m_bytesPerSecondRcv * 0.95 + 0.05 * (((double) m_bytesRcv) / duration);
    m_auctionSendFrequency = m_auctionSendFrequency * 0.95 + 0.05 * (((double) m_numMessagesSentPeriod_Auction) / duration);
    m_auctionRcvFrequency = m_auctionRcvFrequency * 0.95 + 0.05 * (((double) m_numMessagesRcvPeriod_Auction) / duration);
    m_mavlinkSendFrequency = m_mavlinkSendFrequency * 0.95 + 0.05 * (((double) m_numMessagesSentPeriod_Mavlink) / duration);
    m_mavlinkRcvFrequency = m_mavlinkRcvFrequency * 0.95 + 0.05 * (((double) m_numMessagesRcvPeriod_Mavlink) / duration);

    int numMavlinkMessages = m_numMessagesRcvPeriod_Mavlink + m_numMessagesSentPeriod_Mavlink;
    if (numMavlinkMessages > 0)
    {
         double periodFailureRatio = m_numFailuresPeriod / (m_numMessagesRcvPeriod_Mavlink + m_numMessagesSentPeriod_Mavlink);
         m_failureRatio = m_failureRatio * 0.95 + 0.05 * periodFailureRatio;
    }



    m_numMessagesSentPeriod_Auction = 0;
    m_numMessagesRcvPeriod_Auction = 0;

    m_numMessagesSentPeriod_Mavlink = 0;
    m_numMessagesRcvPeriod_Mavlink = 0;

    m_numFailuresPeriod = 0;

    m_bytesRcv = 0;
    m_bytesSent = 0;
}
