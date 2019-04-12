#ifndef MODULE_VEHICLE_SENSORS_H
#define MODULE_VEHICLE_SENSORS_H

#include <iostream>
#include <cmath>

#include "module_vehicle_sensors_global.h"

#include "mace_core/i_module_topic_events.h"

#include "data/i_topic_component_data_object.h"
#include "data/topic_data_object_collection.h"

#include "data_vehicle_sensors/components.h"
#include "mace_core/i_module_command_sensors.h"

#include "data_generic_state_item/state_item_components.h"

#include "data_generic_state_item_topic/state_topic_components.h"

#include "data_generic_command_item/command_item_components.h"

#include "maps/iterators/grid_map_iterator.h"
#include "maps/iterators/circle_map_iterator.h"
#include "maps/iterators/polygon_map_iterator.h"
#include "maps/occupancy_definition.h"
#include "maps/map_cell.h"
#include "maps/data_2d_grid.h"
//#include "maps/octomap_wrapper.h"

#include "base/pose/dynamics_aid.h"

class MODULE_VEHICLE_SENSORSSHARED_EXPORT ModuleVehicleSensors : public MaceCore::IModuleCommandSensors
{

public:
    ModuleVehicleSensors();

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

    //!
    //! \brief computeVehicleFootprint Compute the vertices of the camera footprint and notify listeners of updated footprint
    //! \param systemID Generating system ID
    //! \param camera Camera properties
    //! \param globalPosition Position of the vehicle/sensor
    //! \param attitude Attitude of the vehicle/sensor
    //!
    void computeVehicleFootprint(const int &systemID, const DataVehicleSensors::SensorCamera &camera, const DataState::StateGlobalPositionEx &globalPosition, const DataState::StateAttitude &attitude);

    //!
    //! \brief loadTruthMap Load the truth map from a .bt (i.e. occupancy graph) file
    //! \param btFile Filename, relative to the root MACE path
    //!
    void loadBTFile(const string &btFilePath, const string &layerName);

    //!
    //! \brief loadRoadNetwork Load the truth map for a road network from a file
    //! \param filePath Filename, relative to the root MACE path
    //!
    void loadRoadNetwork(const string &filePath, const string &layerName);

    //!
    //! \brief updateDataInSensorFootprint_Circular Update the local map data from truth data in a circular footprint
    //! \param sensorOriginGlobal Sensor origin for footprint calculations
    //!
    void updateDataInSensorFootprint_Circular(const DataState::StateGlobalPositionEx &sensorOriginGlobal);

    //! Virtual functions as defined by IModuleCommandSensors
public:

    //!
    //! \brief NewlyAvailableVehicle Subscriber to a newly available vehicle topic
    //! \param vehicleID Vehilce ID of the newly available vehicle
    //!
    virtual void NewlyAvailableVehicle(const int &vehicleID, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender);

    //!
    //! \brief NewlyAvailableGlobalOrigin Subscriber to a new global origin
    //!
    void NewlyUpdatedGlobalOrigin(const mace::pose::GeodeticPosition_3D &globalOrigin) override;

    //!
    //! \brief OnModulesStarted Method that fires when all modules have started
    //!
    void OnModulesStarted() override;


private:

    //!
    //! \brief computeVehicleFootprint_Circular Compute the radius of the circular footprint created by a camera at a given cartesian position. (ignore skewing effects from attitude)
    //! \param camera Camera sensor
    //! \param sensorOrigin Current sensor origin
    //! \return Radius of the sensor footprint
    //!
    double computeVehicleFootprint_Circular(const DataVehicleSensors::SensorCircularCamera &camera, const CartesianPosition_3D &sensorOrigin);

private:
    //!
    //! \brief cameraSensor Container for camera parameters
    //!
//    DataVehicleSensors::SensorCamera* cameraSensor;

    //!
    //! \brief m_circularCameraSensor Container for a camera with circular footprint
    //!
    std::shared_ptr<DataVehicleSensors::SensorCircularCamera> m_circularCameraSensor;

    //!
    //! \brief m_compressedMapTruth Map/grid containing truth data. Typically loaded from a file
    //!
//    mace::maps::Data2DGrid<mace::maps::MapCell>* m_compressedMapTruth; // TODO: Move into mace_data into LayeredMap object

    //!
    //! \brief m_compressedMapLocal Map/grid containing local data measured by a vehicle
    //!
//    mace::maps::Data2DGrid<mace::maps::MapCell>* m_compressedMapLocal; // TODO: Move into mace_data into LayeredMap object


    std::shared_ptr<mace::maps::LayeredMap> m_LayeredMap_Truth;


    std::shared_ptr<mace::maps::LayeredMap> m_LayeredMap_Local;

    //!
    //! \brief m_truthBTFile Filename for truth data file
    //!
    std::string m_truthBTFile;

private:
    Data::TopicDataObjectCollection<DATA_STATE_GENERIC_TOPICS> m_VehicleDataTopic;
    Data::TopicDataObjectCollection<DATA_VEHICLE_SENSORS> m_SensorDataTopic;
    Data::TopicDataObjectCollection<DATA_VEHICLE_SENSOR_FOOTPRINT> m_SensorFootprintDataTopic;

};

#endif // MODULE_VEHICLE_SENSORS_H
