#ifndef VEHICLE_TASK_STATUS_TOPIC_H
#define VEHICLE_TASK_STATUS_TOPIC_H

#include "data_tasks/task_key.h"
#include "data_auctioneer/task_status_topic.h"

#include "data/topic_data_object_collection.h"
#include "mace_core/topic.h"

#include "mace_core/mace_core.h"

extern const char g_vehicleTaskStatusName[];
extern const MaceCore::TopicComponentStructure g_vehicleTaskStatusStructure;

class VehicleTaskStatusTopic : public Data::NamedTopicComponentDataObject<g_vehicleTaskStatusName, &g_vehicleTaskStatusStructure>
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;
    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);


    int getVehicleID() const;
    void setVehicleID(int vehicleID);

    const TaskKey &getTaskKey() const;
    void setTaskKey(const TaskKey &taskKey);


    bool getSuccess() const;
    void setSuccess(bool success);

    TaskStatusType getStatusType() const;
    void setStatusType(const TaskStatusType &statusType);

private:
    int m_vehicleID;
    TaskKey m_taskKey;

    TaskStatusType m_statusType;

    bool m_success; // The auction module responds to the task management module on receipt of this message, indicating if
    // the status update was accepted. Relevant for start/complete/abort, to handle the unlikely case where an agent loses a task simultaneously
    // with the task management module wants to start, complete, or abort it.
};

#endif // VEHICLE_TASK_STATUS_TOPIC_H
