#ifndef TASK_STATUS_TOPIC_H
#define TASK_STATUS_TOPIC_H

#include "data/topic_data_object_collection.h"
#include "mace_core/topic.h"

#include "data_tasks/task_key.h"

extern const char g_taskStatusName[];
extern const MaceCore::TopicComponentStructure g_taskStatusStructure;

typedef enum class TaskStatusType
{
    Started,
    Aborted,
    Completed
} TaskStatusType;

class TaskStatusTopic : public Data::NamedTopicComponentDataObject<g_taskStatusName, &g_taskStatusStructure>
{
public:
    /*!
     * \brief The DataArray struct is used to store custom data for a task status type.
     * \details If a task status type uses custom data, the user of this topic is responsible for packing
     * the data into the data array. The user is also responsible for converting to and from network byte order,
     * as necessary.
     */
    typedef struct DataArray
    {
        uint8_t data[64];

        DataArray();
        DataArray(const uint8_t (&data)[64]);
        DataArray(const DataArray &other);
    } DataArray;

    virtual MaceCore::TopicDatagram GenerateDatagram() const;
    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);

    TaskKey getTaskKey() const;
    void setTaskKey(const TaskKey &taskKey);

    uint64_t getAgentID() const;
    void setAgentID(const uint64_t &agentID);

    TaskStatusType getStatusType() const;
    void setStatusType(const TaskStatusType &statusType);

    DataArray getData() const;
    void setData(const DataArray &data);

    int GetTypeDataSize() const;

private:
    TaskKey m_taskKey;
    uint64_t m_agentID;

    TaskStatusType m_statusType;

    DataArray m_data;
};

#endif // TASK_STATUS_TOPIC_H
