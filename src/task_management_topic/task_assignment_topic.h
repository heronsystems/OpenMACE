#ifndef TASK_ASSIGNMENT_TOPIC_H
#define TASK_ASSIGNMENT_TOPIC_H

#include <vector>

#include "data_tasks/task_descriptor.h"
#include "data_tasks/task_key.h"

#include "data/topic_data_object_collection.h"
#include "mace_core/topic.h"

#include "mace_core/mace_core.h"

extern const char g_taskAssignmentName[];
extern const MaceCore::TopicComponentStructure g_taskAssignmentStructure;

/*!
 * \brief The TaskKeyTopic class can be used to broadcast task keys or request a corresponding task descriptor.
 */
class TaskAssignmentTopic : public Data::NamedTopicComponentDataObject<g_taskAssignmentName, &g_taskAssignmentStructure>
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;
    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);


    const std::vector<TaskDescriptorPtr> &getAssignedTasks() const;
    void setAssignedTasks(const std::vector<TaskDescriptorPtr> &assignedTasks);

    int getVehicleID() const;
    void setVehicleID(int vehicleID);

private:
    std::vector<TaskDescriptorPtr> m_assignedTasks;
    int m_vehicleID;
};
#endif // TASK_ASSIGNMENT_TOPIC_H
