#include "scrub_message_topic.h"

const char g_scrubMessageName[] = "scrub_message";
const MaceCore::TopicComponentStructure g_scrubMessageStructure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<ScrubMessage>("Scrub Message");

    return structure;
}();

/*!
 * \brief Generates a datagram from underlying data
 * \return Generated datagram
 */
MaceCore::TopicDatagram ScrubMessageTopic::GenerateDatagram() const
{
    MaceCore::TopicDatagram datagram;

    datagram.AddTerminal<ScrubMessage>("Scrub Message", m_message);
    return datagram;
}

/*!
 * \brief Creates underyling data from a datagram
 * \param datagram Datagram to create from
 */
void ScrubMessageTopic::CreateFromDatagram(const MaceCore::TopicDatagram &datagram)
{
    m_message = datagram.GetTerminal<ScrubMessage>("Scrub Message");
}

ScrubMessage ScrubMessageTopic::getMessage() const
{
    return m_message;
}

void ScrubMessageTopic::setMessage(const ScrubMessage &message)
{
    m_message = message;
}
