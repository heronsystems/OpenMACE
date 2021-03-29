#ifndef SCRUB_MESSAGE_TOPIC_H
#define SCRUB_MESSAGE_TOPIC_H


#include "data/topic_data_object_collection.h"
#include "mace_core/topic.h"

#include "scrub_message.h"

extern const char g_scrubMessageName[];
extern const MaceCore::TopicComponentStructure g_scrubMessageStructure;

class ScrubMessageTopic : public Data::NamedTopicComponentDataObject<g_scrubMessageName, &g_scrubMessageStructure>
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;
    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);

    ScrubMessage getMessage() const;
    void setMessage(const ScrubMessage &message);

private:
    ScrubMessage m_message;
};

#endif // SCRUB_MESSAGE_TOPIC_H
