#ifndef MLMACETOGUI_H
#define MLMACETOGUI_H

#include <QHostAddress>
#include <QJsonObject>
#include <QJsonArray>
#include <QJsonDocument>
#include <QTcpSocket>

#include <memory>

#include "mace_core/i_module_topic_events.h"
#include "mace_core/i_module_command_ml_station.h"

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

#include "mlmessagetypes.h"

class MLMACEtoGUI
{
public:
    MLMACEtoGUI();
    MLMACEtoGUI(const QHostAddress &sendAddress, const int &sendPort);

    ~MLMACEtoGUI();

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
    //! \brief sendVehicleParameterList Send the list of vehicle specific parameters
    //! \param vehicleID Vehicle ID parameters pertain to
    //! \param params Parameter map (Key = string, Value = DataGenericItem::DataGenericItem_ParamValue)
    //!
    void sendVehicleParameterList(const int &vehicleID, const std::map<string, DataGenericItem::DataGenericItem_ParamValue> &params);

    //!
    //! \brief sendEnvironmentVertices Send environment boundary vertices to the MACE GUI
    //! \param component Environment boundary component
    //!
    void sendEnvironmentVertices(const std::vector<GeodeticPosition_3D> &component);

    //!
    //! \brief sendVehicleAirspeed Send vehicle airspeed to the MACE GUI
    //! \param vehicleID Vehicle ID with the new airspeed
    //! \param component Vehicle airspeed component
    //!
    void sendVehicleAirspeed(const int &vehicleID, const mace::measurement_topics::Topic_AirSpeedPtr &component);

private:
    //!
    //! \brief m_sendAddress TCP send address for MACE-to-GUI connection
    //!
    QHostAddress m_sendAddress;

    //!
    //! \brief m_sendPort TCP send port for MACE-to-GUI connection
    //!
    int m_sendPort;


};

#endif // MLMACETOGUI_H
