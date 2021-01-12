#ifndef GUITOMACE_H
#define GUITOMACE_H

#include <QHostAddress>
#include <QJsonObject>
#include <QJsonArray>
#include <QJsonDocument>
#include <QTcpSocket>

#include <memory>

#include "base/pose/dynamics_aid.h"

#include "mace_core/i_module_topic_events.h"
#include "mace_core/i_module_command_ground_station.h"
#include "mace_core/i_module_events_ground_station.h"

#include "data/i_topic_component_data_object.h"
#include "data/topic_data_object_collection.h"

#include "data_generic_item/data_generic_item_components.h"
#include "data_generic_item_topic/data_generic_item_topic_components.h"

#include "data_generic_command_item/command_item_components.h"
#include "data_generic_command_item_topic/command_item_topic_components.h"
#include "data_generic_mission_item_topic/mission_item_topic_components.h"

#include "data_vehicle_sensors/components.h"

#include "base/state_space/cartesian_2D_space.h"

#include "commsMACE/udp_link_mace.h"
#include "commsMACE/udp_configuration_mace.h"

#include "messagetypes.h"

#include "spdlog/spdlog.h"
#include "spdlog/async.h" //support for async logging.
#include "spdlog/sinks/basic_file_sink.h"

class GUItoMACE : public MaceLog
{
public:
    GUItoMACE(const MaceCore::IModuleCommandGroundStation *ptrRef);
    GUItoMACE(const MaceCore::IModuleCommandGroundStation *ptrRef, const QHostAddress &sendAddress, const int &sendPort);

    ~GUItoMACE();

    //!
    //! \brief initiateLogs Start log files and logging for the Ground Station module
    //!
    void initiateLogs(const std::string &loggerName, const std::string &loggingPath);

    // ============================================================================= //
    // ===================== Commands from GUI to MACE Core ======================== //
    // ============================================================================= //
    //!
    //! \brief parseTCPRequest Parse data that has been sent to MACE via the MACE GUI
    //! \param jsonObj JSON data to parse from the MACE GUI
    //!
    void parseTCPRequest(const QJsonObject &jsonObj);

    //!
    //! \brief issueCommand Issue command via the GUI to MACE
    //! \param vehicleID Vehicle ID that the command is initated from (0 = all vehicles)
    //! \param jsonObj JSON data containing the command to be issued
    //!
    bool issuedCommand(const std::string &command, const int &vehicleID, const QJsonDocument &data);

    //!
    //! \brief setVehicleMode GUI command initiating a vehicle mode change
    //! \param vehicleID Vehicle ID that the command is initated from (0 = all vehicles)
    //! \param jsonObj JSON data containing the new vehicle mode
    //!
    void setVehicleMode(const int &vehicleID, const QJsonDocument &data);

    //!
    //! \brief setVehicleArm GUI command initiating a vehicle arm status change
    //! \param vehicleID Vehicle ID that the command is initated from (0 = all vehicles)
    //! \param jsonObj JSON data containing the new vehicle arm status
    //!
    void setVehicleArm(const int &vehicleID, const QJsonDocument &data);

    //!
    //! \brief setVehicleHome GUI command to set a new vehicle home position
    //! \param vehicleID Vehicle ID that the command is initated from (0 = all vehicles)
    //! \param jsonObj JSON data containing the new vehicle home data
    //!
    void setVehicleHome(const int &vehicleID, const QJsonDocument &data);

    //!
    //! \brief setGlobalOrigin GUI command to set a new global origin position
    //! \param jsonObj JSON data containing the new global origin data
    //!
    void setGlobalOrigin(const QJsonDocument &data);

    //!
    //! \brief setEnvironmentVertices GUI command to set new environment boundary vertices
    //! \param jsonObj JSON data containing the new environment vertices
    //!
    void setEnvironmentVertices(const QJsonDocument &data);

    //!
    //! \brief setGoHere GUI command to set a new "go here" lat/lon/alt position
    //! \param vehicleID Vehicle ID that the command is initated from (0 = all vehicles)
    //! \param jsonObj JSON data containing the "go here" position
    //!
    void setGoHere(const int &vehicleID, const QJsonDocument &data);

    //!
    //! \brief takeoff GUI command initiating a takeoff
    //! \param vehicleID Vehicle ID that the command is initated from (0 = all vehicles)
    //! \param jsonObj JSON data containing the takeoff position and altitude
    //!
    void takeoff(const int &vehicleID, const QJsonDocument &data);

    //!
    //! \brief getVehicleMission GUI command that grabs a vehicle mission from MACE
    //! \param vehicleID Vehicle ID that the command is initated from (0 = all vehicles)
    //!
    void getVehicleMission(const int &vehicleID);

    //!
    //! \brief getConnectedVehicles Initiate a request to MACE Core for the list of currently connected vehicles
    //!
    void getConnectedVehicles();

    //!
    //! \brief getVehicleHome Initiate a request to MACE Core for the vehicle home location
    //! \param vehicleID Vehicle ID that the command is initated from (0 = all vehicles)
    //!
    void getVehicleHome(const int &vehicleID);

    //!
    //! \brief getEnvironmentBoundary Initiate a request to MACE core for the current environment boundary vertices
    //!
    void getEnvironmentBoundary();

    //!
    //! \brief getGlobalOrigin Initiate a request to MACE core for the current global origin position
    //!
    void getGlobalOrigin();

    //!
    //! \brief setSendAddress Set the TCP send address for GUI-to-MACE comms
    //! \param sendAddress TCP send address
    //!
    void setSendAddress(const QHostAddress &sendAddress);

    //!
    //! \brief setSendPort Set the TCP send port for GUI-to-MACE comms
    //! \param sendPort TCP send port
    //!
    void setSendPort(const int &sendPort);

    // TESTING:
    void testFunction1(const int &vehicleID);
    void testFunction2(const int &vehicleID);
    // END TESTING


    // ============================================================================= //
    // ======================== Send data to the MACE GUI ========================== //
    // ============================================================================= //
    //!
    //! \brief writeTCPData Write data to the MACE GUI via TCP
    //! \param data Data to be sent to the MACE GUI
    //! \return True: success / False: failure
    //!
    bool writeTCPData(QByteArray data);

    //!
    //! \brief writeUDPData Write data to the MACE GUI via UDP
    //! \param data Data to be sent to the MACE GUI
    //! \return True: success / False: failure
    //!
    bool writeUDPData(QByteArray data);

private:
    //!
    //! \brief m_parent Reference to parent object
    //!
    const MaceCore::IModuleCommandGroundStation* m_parent;

    //!
    //! \brief m_sendAddress TCP send address for MACE-to-GUI connection
    //!
    QHostAddress m_sendAddress;

    //!
    //! \brief m_sendPort TCP send port for MACE-to-GUI connection
    //!
    int m_sendPort;

    mace::state_space::Cartesian2DSpacePtr goalSpace;
    mace::state_space::Cartesian2DSpace_SamplerPtr m_goalSampler;

    //!
    //! \brief m_udpConfig UDP configuration for UDP comms
    //!
    CommsMACE::UdpConfiguration m_udpConfig;

    //!
    //! \brief m_udpLink UDP comms link object
    //!
    std::shared_ptr<CommsMACE::UdpLink> m_udpLink;

protected:
    //!
    //! \brief m_logger spdlog logging object
    //!
    std::shared_ptr<spdlog::logger> m_logger;
};

#endif // GUITOMACE_H
