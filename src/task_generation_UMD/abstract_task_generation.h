#ifndef TASKGENERATION_H
#define TASKGENERATION_H

#include "maps/layered_map.h"
#include "data_tasks/task_key.h"
#include "list"
#include "data_tasks/task_loiter_descriptor.h"

class CallbackInterface_TaskGeneration
{
public:

    virtual void newlyAvailableTask(std::list<std::shared_ptr<TaskDescriptor>>) const = 0;
};

class Abstract_TaskGeneration
{
public:
    enum class AlgorithmTypes
    {
        RANDOM,
        FRONTIER,
        UNDEFINED
    };

public:
    Abstract_TaskGeneration():
        m_CBInterface(nullptr),
        m_currentMapObject(nullptr),
        taskCounter(0),
        m_creatorID(0)
    {

    }

    virtual ~Abstract_TaskGeneration() = default;

    virtual void setCreatorID(uint64_t cID)
    {
        m_creatorID = cID;
    }

    //this parameter should probably be const
    virtual void assignMapLayerObject(mace::maps::LayeredMap* layeredMap)
    {
        m_currentMapObject = layeredMap;
    }

    virtual void updateMemberCallback(const CallbackInterface_TaskGeneration* cb)
    {
        this->m_CBInterface = cb;
    }

    virtual void newlyUpdatedMapLayer(const std::string &layerName, const mace::pose::CartesianPosition_2D* pose = new mace::pose::CartesianPosition_2D(0,0)) = 0;

    virtual void newTaskAssignment(std::vector<TaskKey>) = 0;

    virtual AlgorithmTypes getAlgorithmType()
    {
        return m_algorithmType;
    }

    template <class T>
    T *as()
    {
        return static_cast<T*>(this);
    }

    template <class T>
    const T *as() const
    {
        return static_cast<const T*>(this);
    }

protected:

    //local layered map
    mace::maps::LayeredMap* m_currentMapObject;

    //callback interface for new tasks available
    const CallbackInterface_TaskGeneration* m_CBInterface;

    //enum of this type of task generation
    AlgorithmTypes m_algorithmType;

    //local counter for TaskKey identifier
    int taskCounter;

    //creatorID from caller - used for TaskKey
    uint64_t m_creatorID;

};


#endif // TASKGENERATION_H
