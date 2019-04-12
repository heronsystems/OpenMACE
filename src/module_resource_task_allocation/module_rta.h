#ifndef MODULE_RTA_H
#define MODULE_RTA_H

#include "module_resource_task_allocation_global.h"

#include "mace_core/i_module_topic_events.h"
#include "data/i_topic_component_data_object.h"
#include "data/topic_data_object_collection.h"
#include "mace_core/i_module_command_RTA.h"

#include "data_generic_state_item_topic/state_topic_components.h"
#include "data_generic_command_item/command_item_components.h"
#include "data_vehicle_sensors/components.h"
#include "data_generic_state_item/positional_aid.h"

#include <memory>
//#include "environment.h"
#include "environment_custom.h"

using namespace mace;
using namespace geometry;

class MODULE_RESOURCE_TASK_ALLOCATIONSHARED_EXPORT ModuleRTA : public MaceCore::IModuleCommandRTA
{

public:
    //!
    //! \brief ModuleRTA Default constructor
    //!
    ModuleRTA();

    ~ModuleRTA();

    //!
    //! \brief This module as been attached as a module
    //! \param ptr pointer to object that attached this instance to itself
    //!
    virtual void AttachedAsModule(MaceCore::IModuleTopicEvents* ptr);

    //!
    //! \brief Describes the strucure of the parameters for this module
    //! \return Strucure
    //!
    virtual std::shared_ptr<MaceCore::ModuleParameterStructure> ModuleConfigurationStructure() const;

    //!
    //! \brief Provides object contains parameters values to configure module with
    //! \param params Parameters to configure
    //!
    virtual void ConfigureModule(const std::shared_ptr<MaceCore::ModuleParameterValue> &params);

    //!
    //! \brief New non-spooled topic given
    //!
    //! NonSpooled topics send their data immediatly.
    //! \param topicName Name of stopic
    //! \param sender Module that sent topic
    //! \param data Data for topic
    //! \param target Target module (or broadcasted)
    //!
    virtual void NewTopicData(const std::string &topicName, const MaceCore::ModuleCharacteristic &sender, const MaceCore::TopicDatagram &data, const OptionalParameter<MaceCore::ModuleCharacteristic> &target);

    //!
    //! \brief New Spooled topic given
    //!
    //! Spooled topics are stored on the core's datafusion.
    //! This method is used to notify other modules that there exists new data for the given components on the given module.
    //! \param topicName Name of topic given
    //! \param sender Module that sent topic
    //! \param componentsUpdated Components in topic that where updated
    //! \param target Target moudle (or broadcast)
    //!
    virtual void NewTopicSpooled(const std::string &topicName, const MaceCore::ModuleCharacteristic &sender, const std::vector<std::string> &componentsUpdated, const OptionalParameter<MaceCore::ModuleCharacteristic> &target = OptionalParameter<MaceCore::ModuleCharacteristic>());

public:


    // ============================================================================= //
    // ======== Virtual functions as defined by IModuleCommandGenericBoundaries ==== //
    // ============================================================================= //

    //!
    //! \brief NewlyAvailableBoundary Subscriber to a new boundary
    //! \param key Key corresponding to the updated boundary in the core
    //!
    void NewlyAvailableBoundary(const uint8_t &key, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender = OptionalParameter<MaceCore::ModuleCharacteristic>()) override;


    //! Virtual functions as defined by IModuleCommandRTA
public:

    //!
    //! \brief NewlyAvailableVehicle Subscriber for a new vehicle topic
    //! \param vehicleID Vehicle ID of the new vehicle
    //!
    virtual void NewlyAvailableVehicle(const int &vehicleID, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender);


    //!
    //! \brief TestFunction
    //! \param vehicleID
    //!
    void TestFunction(const int &vehicleID) override;

    //!
    //! \brief NewlyUpdatedGlobalOrigin Subsciber for a new global origin position
    //! \param position Geodetic 3D position
    //!
    void NewlyUpdatedGlobalOrigin(const mace::pose::GeodeticPosition_3D &position) override;


    //!
    //! \brief NewlyUpdatedGridSpacing Grid spacing subscriber to update nodes within the environment
    //!
    void NewlyUpdatedGridSpacing() override;

private:
    //!
    //! \brief updateMACEMissions Sends new missions to MACE for each vehicle in the provided list
    //! \param updateCells Map of cells that contain node lists to send to MACE
    //! \param direction Grid direction for missions (NORTH_SOUTH, EAST_WEST, or CLOSEST_POINT)
    //!
    void updateMACEMissions(std::map<int, Cell_2DC> updateCells, GridDirection direction);

    /**
     * @brief updateEnvironment Given a new boundary, update the environment and Voronoi partitions
     * @param boundary New boundary to partition/generate targets for
     */
    void updateEnvironment(const BoundaryItem::BoundaryList &boundary);

private:
    //!
    //! \brief environment Container for the RTA environment (containing boundary, partitions, and targets)
    //!
    std::shared_ptr<Environment_Map> environment;

    //!
    //! \brief m_globalOrigin Global origin used for conversions to/from local coordinate frame
    //!
    std::shared_ptr<CommandItem::SpatialHome> m_globalOrigin;

    //!
    //! \brief m_gridSpacing Grid spacing/resolution used for generating targets/nodes in the boundary
    //!
    double m_gridSpacing;

    //!
    //! \brief m_vehicles Map of vehicle IDs and corresponding 2D cartesian position
    //!
    std::map<int, Position<CartesianPosition_2D> > m_vehicles;

    //!
    //! \brief m_vehicleCells Map of vehicle IDs and corresponding vehicle cells (boundary, and environment nodes)
    //!
    std::map<int, mace::geometry::Cell_2DC> m_vehicleCells;

    // Flags:
    //!
    //! \brief m_globalInstance Denotes a global or local instance, used to determine if targets should be generated
    //!
    bool m_globalInstance;

    //!
    //! \brief gridSpacingSent Deprecated
    //!
    bool gridSpacingSent;

    //!
    //! \brief environmentBoundarySent Deprecated
    //!
    bool environmentBoundarySent;


private:
    Data::TopicDataObjectCollection<DATA_STATE_GENERIC_TOPICS> m_VehicleDataTopic;
    Data::TopicDataObjectCollection<DATA_VEHICLE_SENSORS> m_SensorDataTopic;
    Data::TopicDataObjectCollection<DATA_VEHICLE_SENSOR_FOOTPRINT> m_SensorFootprintDataTopic;
};

#endif // MODULE_RTA_H

