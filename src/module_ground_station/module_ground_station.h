#ifndef MODULE_GROUND_STATION_H
#define MODULE_GROUND_STATION_H

#include "module_ground_station_global.h"

#include <string>
#include <memory>

#include <QtNetwork/QTcpServer>
#include <QtNetwork/QTcpSocket>
#include <QJsonObject>
#include <QJsonArray>
#include <QJsonDocument>
#include <QThread>

#include "common/common.h"

#include "guitimer.h"
#include "macetogui.h"
#include "guitomace.h"

#include "mace_core/i_module_command_ground_station.h"

#include "base_topic/base_topic_components.h"

#include "spdlog/spdlog.h"
#include "spdlog/async.h" //support for async logging.
#include "spdlog/sinks/basic_file_sink.h"

using namespace std;

class MODULE_GROUND_STATIONSHARED_EXPORT ModuleGroundStation : public MaceCore::IModuleCommandGroundStation
{

public:
    ModuleGroundStation();

    ~ModuleGroundStation();

    virtual std::vector<MaceCore::TopicCharacteristic> GetEmittedTopics();


    //!
    //! \brief initiateLogs Start log files and logging for the Ground Station module
    //!
    void initiateLogs();

    //!
    //! \brief Starts the TCP server for the GCS to send requests to
    //! \return
    //!
    virtual bool StartTCPServer();

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
    //! \brief AssignLoggingDirectory
    //! \param path
    //!
    virtual void AssignLoggingDirectory(const std::string &path);

    //!
    //! \brief start Start event listener thread
    //!
    virtual void start();


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


    // ============================================================================= //
    // ======== Virtual functions as defined by IModuleCommandGenericBoundaries ==== //
    // ============================================================================= //

    //!
    //! \brief NewlyAvailableBoundary Subscriber to a new boundary
    //! \param key Key corresponding to the updated boundary in the core
    //!
    void NewlyAvailableBoundary(const uint8_t &key, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender = OptionalParameter<MaceCore::ModuleCharacteristic>()) override;


    // ============================================================================= //
    // ======== Virtual functions as defined by IModuleCommandGroundStation ======== //
    // ============================================================================= //
public:

    //!
    //! \brief NewlyAvailableVehicle Subscriber to a newly available vehilce topic
    //! \param vehicleID Vehilce ID of the newly available vehicle
    //!
    virtual void NewlyAvailableVehicle(const int &vehicleID, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender);

    //!
    //! \brief NewlyAvailableParameterList Explicitly being told about the newly available parameters
    //! \param params the params that are currently relevant to the vehicle
    //!
    void NewlyAvailableParameterList(const std::map<std::string, DataGenericItem::DataGenericItem_ParamValue> &params, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender) override;

    //!
    //! \brief NewlyAvailableCurrentMission Subscriber to a new vehicle mission topic
    //! \param missionKey Key denoting which mission is available
    //!
    void NewlyAvailableCurrentMission(const MissionItem::MissionKey &missionKey) override;

    //!
    //! \brief NewlyAvailableMissionExeState Subscriber to a new vehicle mission state topic
    //! \param key Key denoting which mission has a new exe state
    //!
    void NewlyAvailableMissionExeState(const MissionItem::MissionKey &key) override;

    //!
    //! \brief NewlyAvailableHomePosition Subscriber to a new home position
    //! \param home New home position
    //!
    void NewlyAvailableHomePosition(const command_item::SpatialHome &home, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender) override;

    //!
    //! \brief NewlyAvailableGlobalOrigin Subscriber to a new global origin
    //!
    void NewlyUpdatedGlobalOrigin(const mace::pose::GeodeticPosition_3D &position) override;


    // ============================================================================= //
    // =============================== Public slots ================================ //
    // ============================================================================= //
public slots:
    //!
    //! \brief on_newConnection Slot to fire when a new TCP connection is initiated
    //!
    void on_newConnection();


    // ============================================================================= //
    // ============================= Topic Collections ============================= //
    // ============================================================================= //
private:

    //!
    //! \brief m_SensorDataTopic Sensor data topic collection
    //!
    Data::TopicDataObjectCollection<DATA_VEHICLE_SENSORS> m_SensorDataTopic;

    //!
    //! \brief m_SensorFootprintDataTopic Sensor footprint data topic collection
    //!
    Data::TopicDataObjectCollection<DATA_VEHICLE_SENSOR_FOOTPRINT> m_SensorFootprintDataTopic;

    //!
    //! \brief m_VehicleDataTopic Vehicle data topic collection
    //!
    Data::TopicDataObjectCollection<DATA_GENERIC_VEHICLE_ITEM_TOPICS, BASE_POSE_TOPICS, VEHICLE_MEASUREMENT_TOPICS> m_VehicleDataTopic;

    //!
    //! \brief m_MissionDataTopic Mission data topic collection
    //!
    Data::TopicDataObjectCollection<DATA_MISSION_GENERIC_TOPICS> m_MissionDataTopic;


    //!
    //! \brief m_VehicleRoutingTopic Routing and trajectory topic collection
    //!
    Data::TopicDataObjectCollection<VEHICLE_ROUTING_TOPICS> m_VehicleRoutingTopic;


    // ============================================================================= //
    // ============================== Member Variables ============================= //
    // ============================================================================= //
private:

    //!
    //! \brief m_TcpServer TCP server to listen for GUI messages
    //!
    std::shared_ptr<QTcpServer> m_TcpServer;

    //!
    //! \brief m_ListenThread Thread that listens for new TCP connections
    //!
    QThread *m_ListenThread;

    //!
    //! \brief m_timer Timer that fires to adjust timeout flags
    //!
    std::shared_ptr<GUITimer> m_timer;

    //!
    //! \brief m_guiHostAddress TCP listen address for GUI-to-MACE connection
    //!
    QHostAddress m_guiHostAddress;

    //!
    //! \brief m_mlGuiHostAddress TCP listen address for MLGUI-to-MACE connection
    //!
    QHostAddress m_mlGuiHostAddress;

    //!
    //! \brief m_listenPort TCP listen port for GUI-to-MACE connection
    //!
    int m_listenPort;

    //!
    //! \brief m_mlListenPort TCP listen port for MLGUI-to-MACE connection
    //!
    int m_mlListenPort;

    //!
    //! \brief m_toMACEHandler Handler for all comms going to MACE from the GUI
    //!
    std::shared_ptr<GUItoMACE> m_toMACEHandler;

    //!
    //! \brief m_toGUIHandler Handler for all comms going to the GUI from MACE
    //!
    std::shared_ptr<MACEtoGUI> m_toGUIHandler;

    // TESTING:
    double latitude;
    // END TESTING

};

#endif // MODULE_GROUND_STATION_H


