#ifndef RANDOMGENERATION_H
#define RANDOMGENERATION_H

#include "abstract_task_generation.h"

#include "base/state_space/space_information.h"
#include "base/state_space/cartesian_2D_space.h"

#include "maps/data_2d_grid.h"

#include "maps/occupancy_definition.h"



MACE_CLASS_FORWARD(TaskGeneration_RandomSpatial);

class TaskGeneration_RandomSpatial: public Abstract_TaskGeneration
{
public:
    TaskGeneration_RandomSpatial();

    ~TaskGeneration_RandomSpatial(){}

     std::list<std::shared_ptr<TaskDescriptor>> generateRandomWaypoint();

    // virtual void updateLayeredMap(LayeredMap*);

    // this is a placeholder method using BaseGridMap until LayeredMap is implemented
    void assignMapLayerObject(mace::maps::LayeredMap* layeredMap) override;

    void newlyUpdatedMapLayer(const std::string &layerName, const mace::pose::CartesianPosition_2D* pose) override;

    virtual void newTaskAssignment(std::vector<TaskKey>) override;

private:

    //local space information to keep track of map changes
    std::shared_ptr<mace::state_space::SpaceInformation> spaceInfo;
    std::shared_ptr<mace::state_space::Cartesian2DSpace> space;
    std::shared_ptr<mace::state_space::Cartesian2DSpace_Sampler> sampler;


};

#endif // RANDOMGENERATION_H
