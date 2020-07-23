#ifndef MACETOGUI_H
#define MACETOGUI_H

#include <QHostAddress>
#include <QJsonObject>
#include <QJsonArray>
#include <QJsonDocument>
#include <QTcpSocket>

#include <memory>

#include "mace_core/i_module_topic_events.h"
#include "mace_core/i_module_command_ground_station.h"

#include "data/i_topic_component_data_object.h"
#include "data/topic_data_object_collection.h"

#include "data_generic_item/data_generic_item_components.h"
#include "data_generic_item_topic/data_generic_item_topic_components.h"

#include "data_generic_command_item/command_item_components.h"
#include "data_generic_command_item_topic/command_item_topic_components.h"
#include "data_generic_mission_item_topic/mission_item_topic_components.h"

#include "data_vehicle_sensors/components.h"

#include "base_topic/pose/topic_geodetic_position.h"
#include "base_topic/pose/topic_agent_orientation.h"
#include "base_topic/measurements/topic_speed.h"

#include "messagetypes.h"

class MACEtoGUI
{
public:
    MACEtoGUI();
    MACEtoGUI(const QHostAddress &sendAddress, const int &sendPort);

    ~MACEtoGUI();

    // ============================================================================= //
    // ================================= Setters =================================== //
    // ============================================================================= //
    //!
    //! \brief setSendAddress Set the TCP send address for MACE-to-GUI comms
    //! \param sendAddress TCP send address
    //!
    void setSendAddress(const QHostAddress &sendAddress);

    //!
    //! \brief setSendPort Set the TCP send port for MACE-to-GUI comms
    //! \param sendPort TCP send port
    //!
    void setSendPort(const int &sendPort);

    //!
    //! \brief setPositionTimeout Set position timeout flag
    //! \param flag Boolean denoting if timer has fired (true) or not (false)
    //!
    void setPositionTimeout(const bool &flag);

    //!
    //! \brief setAttitudeTimeout Set attitude timeout flag
    //! \param flag Boolean denoting if timer has fired (true) or not (false)
    //!
    void setAttitudeTimeout(const bool &flag);

    //!
    //! \brief setFuelTimeout Set fuel timeout flag
    //! \param flag Boolean denoting if timer has fired (true) or not (false)
    //!
    void setFuelTimeout(const bool &flag);

    //!
    //! \brief setModeTimeout Set mode timeout flag
    //! \param flag Boolean denoting if timer has fired (true) or not (false)
    //!
    void setModeTimeout(const bool &flag);

    // ============================================================================= //
    // ================================= Getters =================================== //
    // ============================================================================= //

    //!
    //! \brief getPositionTimeout Get the current position timeout flag value
    //! \return Position timeout flag
    //!
    bool getPositionTimeout() { return m_positionTimeoutOccured; }

    //!
    //! \brief getAttitudeTimeout Get the current attitude timeout flag value
    //! \return Attitude timeout flag
    //!
    bool getAttitudeTimeout() { return m_attitudeTimeoutOccured; }

    //!
    //! \brief getFuelTimeout Get the current fuel timeout flag value
    //! \return Fuel timeout flag
    //!
    bool getFuelTimeout() { return m_fuelTimeoutOccured; }

    //!
    //! \brief getModeTimeout Get the current mode timeout flag value
    //! \return Mode timeout flag
    //!
    bool getModeTimeout() { return m_modeTimeoutOccured; }


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
    //! \brief sendPositionData Send vehicle position data to the MACE GUI
    //! \param vehicleID Vehicle ID with new position update
    //! \param component Global position component
    //!
    void sendPositionData(const int &vehicleID, const std::shared_ptr<mace::pose_topics::Topic_GeodeticPosition> &component);

    //!
    //! \brief sendAttitudeData Send vehicle attitude data to the MACE GUI
    //! \param vehicleID Vehicle ID with new attitude update
    //! \param component Vehicle attitude component
    //!
    void sendAttitudeData(const int &vehicleID, const std::shared_ptr<pose_topics::Topic_AgentOrientation> &component);

    //!
    //! \brief sendVehicleFuel Send vehicle fuel data to the MACE GUI
    //! \param vehicleID Vehicle ID with the new fuel update
    //! \param component Vehicle battery component
    //!
    void sendVehicleFuel(const int &vehicleID, const std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_Battery> &component);

    //!
    //! \brief sendVehicleMode Send vehilce mode data to the MACE GUI
    //! \param vehicleID Vehicle ID with the new flight mode update
    //! \param component Vehicle flight mode component
    //!
    void sendVehicleMode(const int &vehicleID, const std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_FlightMode> &component);

    //!
    //! \brief sendVehicleText Send vehicle message data to the MACE GUI
    //! \param vehicleID Vehicle ID with the new message
    //! \param component Vehicle text component
    //!
    void sendVehicleText(const int &vehicleID, const std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_Text> &component);

    //!
    //! \brief sendVehicleMission Send vehicle mission data to the MACE GUI
    //! \param vehicleID Vehicle ID with the new mission available
    //! \param missionList Mission list component
    //!
    void sendVehicleMission(const int &vehicleID, const MissionItem::MissionList &missionList);

    //!
    //! \brief sendVehicleHome Send new vehicle home to the MACE GUI
    //! \param vehicleID Vehicle ID with the new vehicle home available
    //! \param home New vehicle home
    //!
    void sendVehicleHome(const int &vehicleID, const command_item::SpatialHome &home);

    //!
    //! \brief MACEtoGUI::sendGlobalOrigin Send new global origin to the MACE GUI
    //! \param origin New global origin
    //!
    void sendGlobalOrigin(const command_item::SpatialHome &origin);

    //!
    //! \brief sendSensorFootprint Send vehicle sensor footprint to the MACE GUI
    //! \param vehicleID Vehicle ID with the new sensor footprint available
    //! \param component Vehicle sensor footprint component
    //!
    void sendSensorFootprint(const int &vehicleID, const std::shared_ptr<DataVehicleSensors::SensorVertices_Global> &component);

    //!
    //! \brief sendEnvironmentVertices Send environment boundary vertices to the MACE GUI
    //! \param component Environment boundary component
    //!
    void sendEnvironmentVertices(const std::vector<GeodeticPosition_3D> &component);

    //!
    //! \brief sendCurrentMissionItem Send vehicle mission to the MACE GUI
    //! \param vehicleID Vehicle ID with the new vehicle mission
    //! \param component Vehicle mission component
    //!
    void sendCurrentMissionItem(const int &vehicleID, const std::shared_ptr<MissionTopic::MissionItemCurrentTopic> &component);

    //!
    //! \brief sendVehicleGPS Send vehicle GPS status to the MACE GUI
    //! \param vehicleID Vehicle ID with the new GPS status
    //! \param component GPS status component
    //!
    void sendVehicleGPS(const int &vehicleID, const std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_GPS> &component);

    //!
    //! \brief sendVehicleHeartbeat Send vehicle heartbeat to the MACE GUI
    //! \param vehicleID Vehicle ID with the new vehicle heartbeat
    //! \param component Vehicle heartbeat component
    //!
    void sendVehicleHeartbeat(const int &vehicleID, const std::shared_ptr<DataGenericItem::DataGenericItem_Heartbeat> &component);

    //!
    //! \brief sendMissionItemReached Send mission item reached topic to the MACE GUI
    //! \param vehicleID Vehicle ID corresponding to the vehicle who reached a mission item
    //! \param component Mission item reached component
    //!
    void sendMissionItemReached(const int &vehicleID, const std::shared_ptr<MissionTopic::MissionItemReachedTopic> &component);

    //!
    //! \brief sendVehicleArm Send vehicle arm status to the MACE GUI
    //! \param vehicleID Vehicle ID with the new ARM status
    //! \param component Vehicle Arm component
    //!
    void sendVehicleArm(const int &vehicleID, const std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_SystemArm> &component);

    //!
    //! \brief sendVehicleAirspeed Send vehicle airspeed to the MACE GUI
    //! \param vehicleID Vehicle ID with the new airspeed
    //! \param component Vehicle airspeed component
    //!
    void sendVehicleAirspeed(const int &vehicleID, const mace::measurement_topics::Topic_AirSpeedPtr &component);

    //!
    //! \brief sendMissionState Send vehicle mission state to the MACE GUI
    //! \param key Key denoting which mission is available
    //! \param list Mission list to send to the MACE GUI
    //!
    void sendMissionState(const MissionItem::MissionKey &key, const MissionItem::MissionList &list);

    //!
    //! \brief sendVehicleTarget Send current vehicle target to the MACE GUI
    //! \param vehicleID Vehicle ID with the new vehicle target
    //! \param component Vehicle target component
    //!
    void sendVehicleTarget(const int &vehicleID, const Abstract_GeodeticPosition* targetPosition);

    // ============================================================================= //
    // ================================== Helpers ================================== //
    // ============================================================================= //
private:

    //!
    //! \brief missionListToJSON Convert a mission list to a JSON array
    //! \param list Mission list to convert to a JSON array
    //! \param missionItems JSON Container for converted mission items
    //!
    void missionListToJSON(const MissionItem::MissionList &list, QJsonArray &missionItems);

private:
    //!
    //! \brief m_sendAddress TCP send address for MACE-to-GUI connection
    //!
    QHostAddress m_sendAddress;

    //!
    //! \brief m_sendPort TCP send port for MACE-to-GUI connection
    //!
    int m_sendPort;

    //!
    //! \brief m_positionTimeoutOccured Timeout flag to check if new position data should be sent to the MACE GUI
    //!
    bool m_positionTimeoutOccured;

    //!
    //! \brief m_attitudeTimeoutOccured Timeout flag to check if new attitude data should be sent to the MACE GUI
    //!
    bool m_attitudeTimeoutOccured;

    //!
    //! \brief m_modeTimeoutOccured Timeout flag to check if new mode data should be sent to the MACE GUI
    //!
    bool m_modeTimeoutOccured;

    //!
    //! \brief m_fuelTimeoutOccured Timeout flag to check if new fuel data should be sent to the MACE GUI
    //!
    bool m_fuelTimeoutOccured;

};

#endif // MACETOGUI_H
