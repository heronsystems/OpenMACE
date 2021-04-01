#ifndef BID_DESCRIPTOR_TOPIC_H
#define BID_DESCRIPTOR_TOPIC_H

#include "data/topic_data_object_collection.h"
#include "mace_core/topic.h"


#include "bid_descriptor.h"

extern const char g_bidDescriptorName[];
extern const MaceCore::TopicComponentStructure g_bidDescriptorStructure;

class BidDescriptorTopic : public Data::NamedTopicComponentDataObject<g_bidDescriptorName, &g_bidDescriptorStructure>
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;
    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);

    BidDescriptor getDescriptor() const;
    void setDescriptor(const BidDescriptor &descriptor);

    const Data::EnvironmentTime &getTimestamp() const;
    void setTimestamp(const Data::EnvironmentTime &timestamp);

private:
    BidDescriptor m_descriptor;
    Data::EnvironmentTime m_timestamp;
};

#endif // BID_DESCRIPTOR_TOPIC_H
