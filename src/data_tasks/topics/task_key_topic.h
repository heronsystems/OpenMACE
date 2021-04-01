#ifndef TASK_KEY_TOPIC_H
#define TASK_KEY_TOPIC_H

#include "data/topic_data_object_collection.h"
#include "mace_core/topic.h"

#include "../task_key.h"
#include "mace_core/mace_core.h"

extern const char g_taskKeyName[];
extern const MaceCore::TopicComponentStructure g_taskKeyStructure;

/*!
 * \brief The TaskKeyTopic class can be used to broadcast task keys or request a corresponding task descriptor.
 */
class TaskKeyTopic : public Data::NamedTopicComponentDataObject<g_taskKeyName, &g_taskKeyStructure>
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;
    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);

    const TaskKey &getKey() const;
    void setKey(const TaskKey &key);

    bool getRequestingDescriptor() const;
    void setRequestingDescriptor(bool requestingDescriptor);

private:
    TaskKey m_key;
    bool    m_requestingDescriptor;
};

#endif // TASK_KEY_TOPIC_H
