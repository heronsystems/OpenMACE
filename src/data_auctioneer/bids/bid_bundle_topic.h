#ifndef BID_BUNDLE_TOPIC_H
#define BID_BUNDLE_TOPIC_H

#include "data/topic_data_object_collection.h"
#include "mace_core/topic.h"

#include "bid_bundle.h"

extern const char g_bidBundleName[];
extern const MaceCore::TopicComponentStructure g_bidBundleStructure;

class BidBundleTopic : public Data::NamedTopicComponentDataObject<g_bidBundleName, &g_bidBundleStructure>
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;
    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);

    const BidBundle &getBundle() const;
    void setBundle(const BidBundle &bundle);

private:
    BidBundle m_bundle;
};

#endif // BID_BUNDLE_TOPIC_H
