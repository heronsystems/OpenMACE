#ifndef MLGUItoMACE_H
#define MLGUItoMACE_H

#include <QHostAddress>
#include <QJsonObject>
#include <QJsonArray>
#include <QJsonDocument>
#include <QTcpSocket>

#include <memory>

#include "base/pose/dynamics_aid.h"

#include "mace_core/i_module_topic_events.h"
#include "mace_core/i_module_command_ml_station.h"
#include "mace_core/i_module_events_ml_station.h"

#include "data/i_topic_component_data_object.h"
#include "data/topic_data_object_collection.h"

#include "data_generic_item/data_generic_item_components.h"
#include "data_generic_item_topic/data_generic_item_topic_components.h"

#include "data_generic_command_item/command_item_components.h"
#include "data_generic_command_item_topic/command_item_topic_components.h"
#include "data_generic_mission_item_topic/mission_item_topic_components.h"

#include "data_vehicle_sensors/components.h"

#include "base/state_space/cartesian_2D_space.h"

#include "mlmessagetypes.h"

class MLGUItoMACE
{
public:
    MLGUItoMACE(const MaceCore::IModuleCommandMLStation *ptrRef);
    MLGUItoMACE(const MaceCore::IModuleCommandMLStation *ptrRef, const QHostAddress &sendAddress, const int &sendPort);

    ~MLGUItoMACE();

    // ============================================================================= //
    // ===================== Commands from GUI to MACE Core ======================== //
    // ============================================================================= //
    //!
    //! \brief parseTCPRequest Parse data that has been sent to MACE via the MACE GUI
    //! \param jsonObj JSON data to parse from the MACE GUI
    //!
    void parseTCPRequest(const QJsonObject &jsonObj);

    //!
    //! \brief setEnvironmentVertices GUI command to set new environment boundary vertices
    //! \param jsonObj JSON data containing the new environment vertices
    //!
    void setEnvironmentVertices(const QJsonDocument &data);

    //!
    //! \brief getConnectedVehicles Initiate a request to MACE Core for the list of currently connected vehicles
    //!
    void getConnectedVehicles();

    //!
    //! \brief getEnvironmentBoundary Initiate a request to MACE core for the current environment boundary vertices
    //!
    void getEnvironmentBoundary();

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


    // ============================================================================= //
    // ======================== Send data to the MACE GUI ========================== //
    // ============================================================================= //
    //!
    //! \brief writeTCPData Write data to the MACE GUI via TCP
    //! \param data Data to be sent to the MACE GUI
    //! \return True: success / False: failure
    //!
    bool writeTCPData(QByteArray data);

private:
    //!
    //! \brief m_parent Reference to parent object
    //!
    const MaceCore::IModuleCommandMLStation* m_parent;

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


};

#endif // MLGUItoMACE_H
