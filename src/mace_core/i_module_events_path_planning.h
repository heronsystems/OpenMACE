#ifndef I_MODULE_EVENTS_PATH_PLANNING_H
#define I_MODULE_EVENTS_PATH_PLANNING_H

#include "i_module_events_general.h"
#include "i_module_events_boundary_generator.h"

#include "maps/data_2d_grid.h"
#include "maps/octomap_wrapper.h"

#include "maps/octomap_sensor_definition.h"
#include "maps/octomap_2d_projection_definition.h"

namespace MaceCore
{

class IModuleEventsPathPlanning  : public IModuleEventsBoundaryGenerator, public IModuleEventsGeneral
{
public:

    //!
    //! \brief EventPP_LoadOccupancyEnvironment Load a new occupancy map
    //! \param sender Sender module
    //! \param filePath Occupancy map file path
    //!
    virtual void EventPP_LoadOccupancyEnvironment(const ModuleBase* sender, const std::string &filePath) = 0;

    //!
    //! \brief EventPP_LoadOctomapProperties Load octomap properties
    //! \param sender Sender module
    //! \param properties Octomap properties
    //!
    virtual void EventPP_LoadOctomapProperties(const ModuleBase* sender, const mace::maps::OctomapSensorDefinition &properties) = 0;

    //!
    //! \brief EventPP_LoadMappingProjectionProperties Load map projection properties
    //! \param sender Sender module
    //! \param properties Map projection properties
    //!
    virtual void EventPP_LoadMappingProjectionProperties(const ModuleBase* sender, const mace::maps::Octomap2DProjectionDefinition &properties) = 0;

    //!
    //! \brief EventPP_New2DOccupancyMap New compressed 2D occupancy map event
    //! \param sender Sender module
    //! \param map Occupancy map
    //!
    virtual void EventPP_New2DOccupancyMap(const void* sender, const mace::maps::Data2DGrid<mace::maps::OccupiedResult> &map) = 0;

    //!
    //! \brief EventPP_NewDynamicMissionQueue New dynamic mission queue event
    //! \param sender Sender module
    //! \param queue New mission queue
    //!
    virtual void EventPP_NewDynamicMissionQueue(const ModuleBase* sender, const TargetItem::DynamicMissionQueue &queue) = 0;

    //!
    //! \brief EventPP_NewPathFound New path found event
    //! \param sender Sender module
    //! \param path Path vector
    //!
    virtual void EventPP_NewPathFound(const void* sender, const std::vector<mace::state_space::StatePtr> &path) = 0;

};

} //End MaceCore Namespace

#endif // I_MODULE_EVENTS_PATH_PLANNING_H
