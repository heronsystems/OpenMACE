#include "awarded_task_queue.h"

/*!
 * \brief Layout of values from the Print() function
 * \param out Output stream
 * \param separator Value separator
 * \param subtypeStartMarker Denotes the start of an embedded type
 * \param subtypeEndMarker Denotes the end of an embedded type
 */
void AwardedTaskParams::PrintLayout(std::ostream &out, const std::string &separator, const std::string &subtypeStartMarker, const std::string &subtypeEndMarker)
{
    out << "AwardedTaskParams" << separator
        << "TaskKey" << subtypeStartMarker;
    TaskKey::PrintLayout(out, separator);
    out << subtypeEndMarker << separator
        << "worstCaseBid" << subtypeStartMarker;
    BidDescriptor::PrintLayout(out, separator);
    out << subtypeEndMarker << separator
        << "awardedAgentID" << separator
        << "bidValidTime" << separator
        << "consensusReached";
}

/*!
 * \brief Prints to stream
 * \param out Output stream
 * \param displayValueNames Whether variable names are printed
 * \param valueSeparator Value separator
 * \param namesSeparator Separator between variable name and value
 * \param subtypeStartMarker Denotes the start of an embedded type
 * \param subtypeEndMarker Denotes the end of an embedded type
 */
void AwardedTaskParams::Print(std::ostream &out,
                              bool displayValueNames,
                              const std::string &valueSeparator,
                              const std::string &namesSeparator,
                              const std::string &subtypeStartMarker,
                              const std::string &subtypeEndMarker) const
{
    out << "AwardedTaskParams" << valueSeparator;

    if (displayValueNames)
        out << "taskKey" << namesSeparator;
    out << subtypeStartMarker;
    taskKey.Print(out, displayValueNames, valueSeparator, namesSeparator);
    out << subtypeEndMarker << valueSeparator;

    if (displayValueNames)
        out << "worstCaseBid" << namesSeparator;
    out << subtypeStartMarker;
    taskKey.Print(out, displayValueNames, valueSeparator, namesSeparator);
    out << subtypeEndMarker << valueSeparator;

    if (displayValueNames)
        out << "awardedAgentID" << namesSeparator;
    out << awardedAgentID << valueSeparator;

    if (displayValueNames)
        out << "bidValidTime" << namesSeparator;
    out << bidValidTime << valueSeparator;

    if (consensusReached)
        out << "consensusReached" << namesSeparator;
    out << consensusReached << valueSeparator;
}


/*!
 * \brief Layout of values from the Print() function
 * \param out Output stream
 * \param separator Value separator
 * \param subtypeStartMarker Denotes the start of an embedded type
 * \param subtypeEndMarker Denotes the end of an embedded type
 * \param varSizeMarker Denotes that the previous value is a variable sized list
 */
void AwardedTaskQueue::PrintLayout(std::ostream &out,
                                   const std::string &separator,
                                   const std::string &subtypeStartMarker,
                                   const std::string &subtypeEndMarker,
                                   const std::string &varSizeMarker)
{
    out << "AwardedTaskQueue" << separator
        << "size" << separator
        << "queue" <<subtypeStartMarker;
    AwardedTaskParams::PrintLayout(out, separator, subtypeStartMarker, subtypeEndMarker);
    out << subtypeEndMarker << separator << varSizeMarker;
}

AwardedTaskQueue::AwardedTaskQueue() :
    std::unordered_map<TaskKey, AwardedTaskParams>()
{


}

/*!
 * \brief Prints to stream
 * \param out Output stream
 * \param displayValueNames Whether variable names are printed
 * \param paramsOnNewLine Whether elements of the queue should be printed on a new line
 * \param valueSeparator Value separator
 * \param namesSeparator Separator between variable name and value
 * \param subtypeStartMarker Denotes the start of an embedded type
 * \param subtypeEndMarker Denotes the end of an embedded type
 */
void AwardedTaskQueue::Print(std::ostream &out,
                             bool displayValueNames,
                             bool paramsOnNewLine,
                             const std::string &valueSeparator,
                             const std::string &namesSeparator,
                             const std::string &subtypeStartMarker,
                             const std::string &subtypeEndMarker) const
{
    int size = this->size();
    out << "AwardedTaskQueue" << valueSeparator;

    if (displayValueNames)
        out << "size" << namesSeparator;
    out << size << valueSeparator;

    int paramIndex = 0;
    for (auto pair : *this)
    {
        const AwardedTaskParams &params = pair.second;

        if (displayValueNames)
            out << "AwardedTaskParams_" << paramIndex << namesSeparator;
        out << subtypeStartMarker;
        params.Print(out, displayValueNames, valueSeparator, namesSeparator, subtypeStartMarker, subtypeEndMarker);
        out << subtypeEndMarker;

        if (paramIndex < size - 1)
            out << valueSeparator;

        if (paramsOnNewLine)
            out << std::endl;

        ++paramIndex;
    }
}
