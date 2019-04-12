#ifndef I_MODULE_COMMAND_ROS_H
#define I_MODULE_COMMAND_ROS_H

#include <string>
#include <map>

#include "abstract_module_event_listeners.h"
#include "metadata_ROS.h"

#include "i_module_topic_events.h"
#include "i_module_events_ROS.h"

#include "maps/data_2d_grid.h"
#include "maps/octomap_wrapper.h"

#include "i_module_command_generic_boundaries.h"
#include "i_module_command_generic_vehicle_listener.h"

namespace MaceCore
{

enum class ROSCommands
{
    BASE_MODULE_LISTENER_ENUMS,
    GENERIC_MODULE_BOUNDARY_LISTENER_ENUMS,
    GENERIC_MODULE_VEHICLE_LISTENER_ENUMS,
    NEWLY_UPDATED_3D_OCCUPANCY_MAP,
    NEWLY_COMPRESSED_OCCUPANCY_MAP,
    NEWLY_FOUND_PATH,
    TEST_FIRE
};

class MACE_CORESHARED_EXPORT IModuleCommandROS  :
        public AbstractModule_EventListeners<MetadataROS, IModuleEventsROS, ROSCommands>,
        public IModuleGenericBoundaries,
        public IModuleGenericVehicleListener
{
    friend class MaceCore;
public:

    static ModuleClasses moduleClass;

    IModuleCommandROS():
        AbstractModule_EventListeners()
    {
        IModuleGenericBoundaries::SetUp<MetadataROS, IModuleEventsROS, ROSCommands>(this);
        IModuleGenericVehicleListener::SetUp<MetadataROS, IModuleEventsROS, ROSCommands>(this);


        AddCommandLogic<mace::maps::Data2DGrid<mace::maps::OccupiedResult>>(ROSCommands::NEWLY_COMPRESSED_OCCUPANCY_MAP, [this](const mace::maps::Data2DGrid<mace::maps::OccupiedResult> &map, const OptionalParameter<ModuleCharacteristic> &sender){
            UNUSED(sender);
            NewlyCompressedOccupancyMap(map);
        });
        AddCommandLogic<std::vector<mace::state_space::StatePtr>>(ROSCommands::NEWLY_FOUND_PATH, [this](const std::vector<mace::state_space::StatePtr> &path, const OptionalParameter<ModuleCharacteristic> &sender){
            UNUSED(sender);
            NewlyFoundPath(path);
        });
        AddCommandLogic<int>(ROSCommands::NEWLY_UPDATED_3D_OCCUPANCY_MAP, [this](const int &vehicleID, const OptionalParameter<ModuleCharacteristic> &sender){
            UNUSED(sender);
            UNUSED(vehicleID);
            NewlyUpdated3DOccupancyMap();
        });
    }

    virtual ModuleClasses ModuleClass() const
    {
        return moduleClass;
    }

public:


    //!
    //! \brief NewlyUpdated3DOccupancyMap New 3D occupancy map subscriber
    //!
    virtual void NewlyUpdated3DOccupancyMap() = 0;

    //!
    //! \brief NewlyCompressedOccupancyMap New compressed occupancy map subscriber
    //! \param map Compressed occupancy map
    //!
    virtual void NewlyCompressedOccupancyMap(const mace::maps::Data2DGrid<mace::maps::OccupiedResult> &map) = 0;

    //!
    //! \brief NewlyUpdatedOperationalFence New operational fence subscriber
    //! \param boundary New operationl fence
    //!
//    virtual void NewlyUpdatedOperationalFence(const BoundaryItem::BoundaryList &boundary) = 0;

    //!
    //! \brief NewlyFoundPath New path subscriber
    //! \param path New path
    //!
    virtual void NewlyFoundPath(const std::vector<mace::state_space::StatePtr> &path) = 0;

    //    //!
//    //! \brief New targets have been assigned to the given vehicle
//    //! \param vehicleID ID of vehicle
//    //!
//    virtual void NewVehicleTarget(const std::string &vehicleID) = 0;


};

} //End MaceCore Namespace

#endif // I_MODULE_COMMAND_ROS_H
