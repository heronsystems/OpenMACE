#ifndef AUCTION_LOGGER_H
#define AUCTION_LOGGER_H

#include <iostream>

#include <vector>

typedef struct FormatedStream
{
    std::ostream &stream;
    std::string valueSeparator = ",";
    std::string nameSeparator = ":";
    std::string subtypeStartMarker = "{";
    std::string subtypeEndMarker = "}";
    std::string stringStartMarker = "[";
    std::string stringEndMarker = "]";
    std::string varSizeMarker = "...";
    bool displayTimestamp = false;
    bool displayValueNames = false;
    bool multiLineLists = false;

    FormatedStream(std::ostream &s) :
        stream(s)
    {
    }

private:
    friend class AuctionLogger;

    bool printTimestamp = true;
    bool prependSeparator = false;
} FormatedStream;

struct TaskKey;

class TaskDescriptor;
class BidBundle;
class BidDescriptor;

struct AwardedTaskParams;
class AwardedTaskQueue;

class AvailableTaskQueue;
class AssignmentTaskQueue;

class AuctionStatistics_Controller;
class AuctionStatistics_Task;
class AuctionStatistics_Global;

class AuctionLogger
{
public:
    typedef struct StringNotEnclosed
    {
        StringNotEnclosed(const std::string &str) :
            string(str)
        {
        }
    private:
        friend AuctionLogger;
        std::string string;
    } StringNotEnclosed;

//    static AuctionLogger &notEnclosed(AuctionLogger &out, const std::string str);

    AuctionLogger(bool isAgentStats = true);

    void addStream(const FormatedStream &formatedStream, bool printLayout = false);

    AuctionLogger &operator <<(const TaskKey &key);
    AuctionLogger &operator <<(const TaskDescriptor &descriptor);
    AuctionLogger &operator <<(const BidDescriptor &bid);
    AuctionLogger &operator <<(const BidBundle &bundle);

    AuctionLogger &operator <<(const AwardedTaskParams &params);
    AuctionLogger &operator <<(const AwardedTaskQueue &queue);

    AuctionLogger &operator <<(const AvailableTaskQueue &queue);
    AuctionLogger &operator <<(const AssignmentTaskQueue &queue);

    AuctionLogger &operator <<(const AuctionStatistics_Controller &stats);
    AuctionLogger &operator <<(const AuctionStatistics_Task &stats);
    AuctionLogger &operator <<(const AuctionStatistics_Global &stats);

    AuctionLogger &operator <<(const std::string &str);
    AuctionLogger &operator <<(std::ostream &(*streamManip)(std::ostream &));

    AuctionLogger &operator <<(const StringNotEnclosed &manip);



private:
    bool m_isAgentStats;
    std::vector<FormatedStream> m_streams;


    std::string GetCurrentTimeString();
    void PrintTimestamp(FormatedStream &formatedStream, const std::string &timestamp);
    void PrependSeparator(FormatedStream &formatedStream);
};



#endif // AUCTION_LOGGER_H
