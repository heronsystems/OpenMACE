#include <chrono>
#include <iomanip>
#include <sstream>

#include "auction_logger.h"

#include "awarded_task_queue.h"
#include "available_task_queue.h"
#include "assignment_task_queue.h"

#include "bids/bid_descriptor.h"
#include "bids/bid_bundle.h"

#include "data_tasks/task_key.h"
#include "data_tasks/task_descriptor.h"
#include "data_tasks/task_loiter_descriptor.h"
#include "data_tasks/task_survey_descriptor.h"

#include "auction_statistics_controller.h"
#include "auction_statistics_agent.h"

AuctionLogger::AuctionLogger(bool isAgentStats) :
    m_isAgentStats(isAgentStats)
{

}

/*!
 * \brief Adds a reference to an output stream to format output to.
 * \param formatedStream Object containing the stream reference and formatting options
 * \param printLayout Whether the layout of all known types should be output immediately.
 */
void AuctionLogger::addStream(const FormatedStream &formatedStream, bool printLayout)
{
    std::ostream &stream = formatedStream.stream;
    const std::string &valueSeparator = formatedStream.valueSeparator;
    const std::string &subtypeStartMarker = formatedStream.subtypeStartMarker;
    const std::string &subtypeEndMarker = formatedStream.subtypeEndMarker;
    const std::string &varSizeMarker = formatedStream.varSizeMarker;
    if (printLayout)
    {
        if (m_isAgentStats)
        {
            TaskKey::PrintLayout(stream, valueSeparator);
            stream << std::endl;

            TaskDescriptor::PrintLayout(stream, true, valueSeparator);
            stream << std::endl;

            AbstractTaskLoiterDescriptor::PrintLayout(stream, true, valueSeparator);
            stream << std::endl;

            AbstractTaskSurveyDescriptor::PrintLayout(stream, true, valueSeparator);
            stream << std::endl;

            BidDescriptor::PrintLayout(stream, valueSeparator, subtypeStartMarker, subtypeEndMarker);
            stream << std::endl;

            BidBundle::PrintLayout(stream, valueSeparator, subtypeStartMarker, subtypeEndMarker, varSizeMarker);
            stream << std::endl;

            AwardedTaskParams::PrintLayout(stream, valueSeparator, subtypeStartMarker, subtypeEndMarker);
            stream << std::endl;

            AwardedTaskQueue::PrintLayout(stream, valueSeparator, subtypeStartMarker, subtypeEndMarker, varSizeMarker);
            stream << std::endl;

            AssignmentTaskQueue::PrintLayout(stream, valueSeparator, subtypeStartMarker, subtypeEndMarker, varSizeMarker);
            stream << std::endl;

            AvailableTaskQueue::PrintLayout(stream, valueSeparator, subtypeStartMarker, subtypeEndMarker, varSizeMarker);
            stream << std::endl;

            AuctionStatistics_Controller::PrintLayout(stream, valueSeparator);
            stream << std::endl;
        }
        else // EL stats for Auctioneer
        {
            AuctionStatistics_Task::PrintLayout(stream, valueSeparator);
            stream << std::endl;
            AuctionStatistics_Global::PrintLayout(stream, valueSeparator);
            stream << std::endl;
        }
    }

    m_streams.push_back(formatedStream);
}

/***** Basic structures *****/
AuctionLogger &AuctionLogger::operator <<(const TaskKey &key)
{

    std::string timestr = GetCurrentTimeString();
    for (FormatedStream &formatedStream : m_streams)
    {
        PrependSeparator(formatedStream);
        PrintTimestamp(formatedStream, timestr);
        key.Print(formatedStream.stream,
                  formatedStream.displayValueNames,
                  formatedStream.valueSeparator,
                  formatedStream.nameSeparator);
    }

    return *this;
}


AuctionLogger &AuctionLogger::operator <<(const TaskDescriptor &descriptor)
{

    std::string timestr = GetCurrentTimeString();
    for (FormatedStream &formatedStream : m_streams)
    {
        PrependSeparator(formatedStream);
        PrintTimestamp(formatedStream, timestr);
        descriptor.Print(formatedStream.stream,
                         formatedStream.displayValueNames,
                         formatedStream.valueSeparator,
                         formatedStream.nameSeparator);
    }

    return *this;
}

AuctionLogger &AuctionLogger::operator <<(const BidDescriptor &bid)
{

    std::string timestr = GetCurrentTimeString();
    for (FormatedStream &formatedStream : m_streams)
    {
        PrependSeparator(formatedStream);
        PrintTimestamp(formatedStream, timestr);
        bid.Print(formatedStream.stream,
                  formatedStream.displayValueNames,
                  formatedStream.valueSeparator,
                  formatedStream.nameSeparator,
                  formatedStream.subtypeStartMarker,
                  formatedStream.subtypeEndMarker);
    }

    return *this;
}

AuctionLogger &AuctionLogger::operator <<(const BidBundle &bundle)
{

    std::string timestr = GetCurrentTimeString();
    for (FormatedStream &formatedStream : m_streams)
    {
        PrependSeparator(formatedStream);
        PrintTimestamp(formatedStream, timestr);
        bundle.Print(formatedStream.stream,
                     formatedStream.displayValueNames,
                     formatedStream.multiLineLists,
                     formatedStream.valueSeparator,
                     formatedStream.nameSeparator,
                     formatedStream.subtypeStartMarker,
                     formatedStream.subtypeEndMarker);
    }

    return *this;
}


/***** Algorithmic structures *****/
AuctionLogger &AuctionLogger::operator <<(const AwardedTaskParams &params)
{

    std::string timestr = GetCurrentTimeString();
    for (FormatedStream &formatedStream : m_streams)
    {
        PrependSeparator(formatedStream);
        PrintTimestamp(formatedStream, timestr);
        params.Print(formatedStream.stream,
                     formatedStream.displayValueNames,
                     formatedStream.valueSeparator,
                     formatedStream.nameSeparator,
                     formatedStream.subtypeStartMarker,
                     formatedStream.subtypeEndMarker);
    }

    return *this;
}

AuctionLogger &AuctionLogger::operator <<(const AwardedTaskQueue &queue)
{

    std::string timestr = GetCurrentTimeString();
    for (FormatedStream &formatedStream : m_streams)
    {
        PrependSeparator(formatedStream);
        PrintTimestamp(formatedStream, timestr);
        queue.Print(formatedStream.stream,
                    formatedStream.displayValueNames,
                    formatedStream.multiLineLists,
                    formatedStream.valueSeparator,
                    formatedStream.nameSeparator,
                    formatedStream.subtypeStartMarker,
                    formatedStream.subtypeEndMarker);
    }

    return *this;
}

AuctionLogger &AuctionLogger::operator <<(const AvailableTaskQueue &queue)
{

    std::string timestr = GetCurrentTimeString();
    for (FormatedStream &formatedStream : m_streams)
    {
        PrependSeparator(formatedStream);
        PrintTimestamp(formatedStream, timestr);
        queue.Print(formatedStream.stream,
                    formatedStream.displayValueNames,
                    formatedStream.multiLineLists,
                    formatedStream.valueSeparator,
                    formatedStream.nameSeparator,
                    formatedStream.subtypeStartMarker,
                    formatedStream.subtypeEndMarker);
    }

    return *this;
}

AuctionLogger &AuctionLogger::operator <<(const AssignmentTaskQueue &queue)
{

    std::string timestr = GetCurrentTimeString();
    for (FormatedStream &formatedStream : m_streams)
    {
        PrependSeparator(formatedStream);
        PrintTimestamp(formatedStream, timestr);
        queue.Print(formatedStream.stream,
                    formatedStream.displayValueNames,
                    formatedStream.multiLineLists,
                    formatedStream.valueSeparator,
                    formatedStream.nameSeparator,
                    formatedStream.subtypeStartMarker,
                    formatedStream.subtypeEndMarker);
    }

    return *this;
}

/***** Statistics *****/
AuctionLogger &AuctionLogger::operator <<(const AuctionStatistics_Controller &stats)
{
    std::string timestr = GetCurrentTimeString();
    for (FormatedStream &formatedStream : m_streams)
    {
        PrependSeparator(formatedStream);
        PrintTimestamp(formatedStream, timestr);
        stats.Print(formatedStream.stream,
                    formatedStream.displayValueNames,
                    formatedStream.valueSeparator,
                    formatedStream.nameSeparator);
    }

    return *this;
}

AuctionLogger &AuctionLogger::operator <<(const AuctionStatistics_Task &stats)
{
    std::string timestr = GetCurrentTimeString();
    for (FormatedStream &formatedStream : m_streams)
    {
        PrependSeparator(formatedStream);
        PrintTimestamp(formatedStream, timestr);
        stats.Print(formatedStream.stream,
                    formatedStream.displayValueNames,
                    formatedStream.valueSeparator,
                    formatedStream.nameSeparator);
    }

    return *this;
}

AuctionLogger &AuctionLogger::operator <<(const AuctionStatistics_Global &stats)
{
    std::string timestr = GetCurrentTimeString();
    for (FormatedStream &formatedStream : m_streams)
    {
        PrependSeparator(formatedStream);
        PrintTimestamp(formatedStream, timestr);
        stats.Print(formatedStream.stream,
                    formatedStream.displayValueNames,
                    formatedStream.valueSeparator,
                    formatedStream.nameSeparator);
    }

    return *this;
}

/***** Strings *****/
AuctionLogger &AuctionLogger::operator <<(const std::string &str)
{

    std::string timestr = GetCurrentTimeString();
    for (FormatedStream &formatedStream : m_streams)
    {
        PrependSeparator(formatedStream);
        PrintTimestamp(formatedStream, timestr);
        formatedStream.stream << formatedStream.stringStartMarker
                              << str
                              << formatedStream.stringEndMarker;
    }

    return *this;
}

/***** Standard stream manipulation functions *****/
AuctionLogger &AuctionLogger::operator <<(std::ostream &(*streamManip)(std::ostream &))
{
    for (FormatedStream &formatedStream : m_streams)
    {
        formatedStream.stream << (*streamManip);
        formatedStream.printTimestamp = true;
        formatedStream.prependSeparator = false;
    }

    return *this;
}

/***** Custom stream manipulation functions *****/
AuctionLogger &AuctionLogger::operator <<(const StringNotEnclosed &manip)
{
    std::string timestr = GetCurrentTimeString();
    for (FormatedStream &formatedStream : m_streams)
    {
        PrependSeparator(formatedStream);
        PrintTimestamp(formatedStream, timestr);
        formatedStream.stream << manip.string;
    }

    return *this;
}

/*!
 * \brief Returns a string containing the current timestamp
 * \return Current timestamp
 */
std::string AuctionLogger::GetCurrentTimeString()
{
    Data::EnvironmentTime now;
    Data::EnvironmentTime::CurrentTime(Data::SYSTEMCLOCK, now);

    return now.ToString().toStdString();
}

/*!
 * \brief Prints a timestamp.
 * \details Timestamps are printed when std::endl is passed to stream manipulation function operator.
 * \param formatedStream Stream and formatting information
 * \param timestamp Timestamp string
 */
void AuctionLogger::PrintTimestamp(FormatedStream &formatedStream, const std::string &timestamp)
{
    if (formatedStream.displayTimestamp && formatedStream.printTimestamp)
    {
        formatedStream.stream << formatedStream.stringStartMarker
                          << timestamp
                          << formatedStream.stringEndMarker
                          << formatedStream.valueSeparator;
    }
    formatedStream.printTimestamp = false;
}

void AuctionLogger::PrependSeparator(FormatedStream &formatedStream)
{
    if (formatedStream.prependSeparator)
        formatedStream.stream << formatedStream.valueSeparator;
    formatedStream.prependSeparator = true;

}
