#include "random_generation.h"
#include "data/environment_time.h"
#include "data_tasks/task_descriptor.h"
#include "maps/base_grid_map.h"
#include "maps/layered_map.h"
#include "data_tasks/task_loiter_descriptor.h"

TaskGeneration_RandomSpatial::TaskGeneration_RandomSpatial()
{
    m_algorithmType = AlgorithmTypes::RANDOM;

    space = std::make_shared<mace::state_space::Cartesian2DSpace>();
    sampler = std::make_shared<mace::state_space::Cartesian2DSpace_Sampler>(space);
    spaceInfo = std::make_shared<mace::state_space::SpaceInformation>(space);
    spaceInfo->setStateSampler(sampler);
}


std::list<std::shared_ptr<TaskDescriptor>> TaskGeneration_RandomSpatial::generateRandomWaypoint()
{
    // local state_space is up to date with any new map information
    // so now just create random waypoints within the boundaries

    // std::list<TaskKey*> taskKeyList;
    std::list<std::shared_ptr<TaskDescriptor>> taskList;

    mace::state_space::State* sampleState = space->getNewState();
    spaceInfo->getStateSampler()->sampleUniform(sampleState);
    //Ken Comment: Be careful with the sample state pointer, once you create a task with it, I would sugguest calling destructor to avoid memory leaks


    // create a task key for the call back
    if(m_CBInterface != nullptr)
    {
        mace::pose::CartesianPosition_2D* cast = sampleState->as<pose::CartesianPosition_2D>();

        std::shared_ptr<TaskLoiterDescriptor<mace::pose::CartesianPosition_2D>> task = std::make_shared<TaskLoiterDescriptor<mace::pose::CartesianPosition_2D>>(m_creatorID, taskCounter);
        task->setLoiterPosition(*cast);

        taskCounter++;

        Data::EnvironmentTime now;
        Data::EnvironmentTime::CurrentTime(Data::SYSTEMCLOCK, now);
        task->setTimeCreated(now);

        TaskKey key = task->getTaskKey();

        taskList.push_back(task);

    }

    m_CBInterface->newlyAvailableTask(taskList);

    //return the state that has randomized waypoints
    return taskList;

}

void TaskGeneration_RandomSpatial::assignMapLayerObject(mace::maps::LayeredMap* layeredMap)
{
    //call the abstract base class functionality here
    Abstract_TaskGeneration::assignMapLayerObject(layeredMap);

    //since this is just random, grab one of the maps within the layered map and get its size
    std::unordered_map<std::string, mace::maps::BaseGridMap*>  map = layeredMap->getLayeredMap();

    double xMax = map.begin()->second->getXMax();
    double xMin = map.begin()->second->getXMin();
    double yMax = map.begin()->second->getYMax();
    double yMin = map.begin()->second->getYMin();

    space->setBounds(mace::state_space::Cartesian2DSpaceBounds(xMin, xMax, yMin, yMax));

}


void TaskGeneration_RandomSpatial::newlyUpdatedMapLayer(const std::string &layerName, const mace::pose::CartesianPosition_2D* pose = new mace::pose::CartesianPosition_2D(0,0))
{
    UNUSED(layerName);
}

void TaskGeneration_RandomSpatial::newTaskAssignment(std::vector<TaskKey>)
{

}



