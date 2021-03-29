#ifndef TASK_LOITER_TOPIC_H
#define TASK_LOITER_TOPIC_H

#include "data/topic_data_object_collection.h"
#include "mace_core/topic.h"

#include "../task_loiter_descriptor.h"
#include "mace_core/module_characteristics.h"

extern const char g_taskLoiterName[];
extern const MaceCore::TopicComponentStructure g_taskLoiterStructure;

/*!
 * \brief The TaskLoiterTopic class can be used to send a loiter descriptor.
 */
class TaskLoiterTopic : public Data::NamedTopicComponentDataObject<g_taskLoiterName, &g_taskLoiterStructure>
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;
    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);

    const std::shared_ptr<AbstractTaskLoiterDescriptor> &getDescriptor() const;
    void setDescriptor(const std::shared_ptr<AbstractTaskLoiterDescriptor> &descriptor);

    bool getResponding() const;
    void setResponding(bool requesting);

private:
    std::shared_ptr<AbstractTaskLoiterDescriptor> m_descriptor;
    bool m_requesting;
};

#endif // TASK_LOITER_TOPIC_H
