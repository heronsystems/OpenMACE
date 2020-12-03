#include "mlmacetogui.h"


MLMACEtoGUI::MLMACEtoGUI() :
    m_sendAddress(QHostAddress::LocalHost),
    m_sendPort(1234),
    m_udpConfig("127.0.0.1", 5678, "127.0.0.1", 9080)
{
    m_udpLink = std::make_shared<CommsMACE::UdpLink>(m_udpConfig);
    m_udpLink->Connect();
}

MLMACEtoGUI::MLMACEtoGUI(const QHostAddress &sendAddress, const int &sendPort) :
    m_sendAddress(sendAddress),
    m_sendPort(sendPort),
    m_udpConfig("127.0.0.1", 5678, sendAddress.toString().toStdString(), sendPort)
{
    m_udpLink = std::make_shared<CommsMACE::UdpLink>(m_udpConfig);
    m_udpLink->Connect();
}

MLMACEtoGUI::~MLMACEtoGUI() {
    m_logger->flush();
}

//!
//! \brief setSendAddress Set the TCP send address for MACE-to-GUI comms
//! \param sendAddress TCP send address
//!
void MLMACEtoGUI::setSendAddress(const QHostAddress &sendAddress) {
    m_sendAddress = sendAddress;
}

//!
//! \brief setSendPort Set the TCP send port for MACE-to-GUI comms
//! \param sendPort TCP send port
//!
void MLMACEtoGUI::setSendPort(const int &sendPort) {
    m_sendPort = sendPort;
}

void MLMACEtoGUI::initiateLogs(const std::string &loggerName, const std::string &loggingPath)
{
    try
    {
        m_logger = spdlog::basic_logger_mt<spdlog::async_factory>(loggerName, loggingPath);
    }
    catch (const spdlog::spdlog_ex& ex)
    {
        std::cout << "Log initialization failed: " << ex.what() << std::endl;
    }


    // Flush logger every 2 seconds:
//    spdlog::flush_every(std::chrono::seconds(2));
    m_logger->flush_on(spdlog::level::info);      // flush when "info" or higher message is logged

}


//!
//! \brief sendVehicleParameterList Send the list of vehicle specific parameters
//! \param vehicleID Vehicle ID parameters pertain to
//! \param params Parameter map (Key = string, Value = DataGenericItem::DataGenericItem_ParamValue)
//!
void MLMACEtoGUI::sendVehicleParameterList(const int &vehicleID, const std::map<std::string, DataGenericItem::DataGenericItem_ParamValue> &params)
{
//    QJsonObject obj = home.toJSON(vehicleID, MLMessageString(MLMessageTypes::VEHICLE_HOME));
    QJsonObject obj;
    obj["agentID"] = std::to_string(vehicleID).c_str();;
    obj["message_type"] = MLMessageString(MLMessageTypes::VEHICLE_PARAM_LIST).c_str();

    QJsonArray verticies;
    for(auto&& param : params) {
        QJsonObject obj;
        obj["param_id"] = QString::fromStdString(param.second.getID());
        obj["value"] = param.second.getValue();

        verticies.push_back(obj);
    }

    obj["param_list"] = verticies;

    QJsonDocument doc(obj);
    bool bytesWritten = writeUDPData(doc.toJson());
    if(!bytesWritten){
        std::cout << "Write parameter list failed..." << std::endl;
    }

}


//!
//! \brief sendPositionData Send vehicle position data to the MACE GUI
//! \param vehicleID Vehicle ID with new position update
//! \param component Global position component
//!
void MLMACEtoGUI::sendPositionData(const int &vehicleID, const std::shared_ptr<mace::pose_topics::Topic_GeodeticPosition> &component)
{
    if(component->getPositionObj()->getCoordinateSystemType() == CoordinateSystemTypes::GEODETIC)
    {

        QJsonDocument doc(component->toJSON(vehicleID,MLMessageString(MLMessageTypes::VEHICLE_POSITION)));
        bool bytesWritten = writeUDPData(doc.toJson());

        if(!bytesWritten){
            std::cout << "Write Position Data failed..." << std::endl;
        }
        logToFile(doc);
    }
}

//!
//! \brief sendAttitudeData Send vehicle attitude data to the MACE GUI
//! \param vehicleID Vehicle ID with new attitude update
//! \param component Vehicle attitude component
//!
void MLMACEtoGUI::sendAttitudeData(const int &vehicleID, const std::shared_ptr<mace::pose_topics::Topic_AgentOrientation> &component)
{
    QJsonDocument doc(component->toJSON(vehicleID,MLMessageString(MLMessageTypes::VEHICLE_ATTITUDE)));

    bool bytesWritten = writeUDPData(doc.toJson());

    if(!bytesWritten){
        std::cout << "Write Attitude Data failed..." << std::endl;
    }
    logToFile(doc);
}

//!
//! \brief sendVehicleAirspeed Send vehicle airspeed to the MACE GUI
//! \param vehicleID Vehicle ID with the new airspeed
//! \param component Vehicle airspeed component
//!
void MLMACEtoGUI::sendVehicleAirspeed(const int &vehicleID, const mace::measurement_topics::Topic_AirSpeedPtr &component)
{
        QJsonDocument doc(component->toJSON(vehicleID,MLMessageString(MLMessageTypes::VEHICLE_AIRSPEED)));
        bool bytesWritten = writeUDPData(doc.toJson());

        if(!bytesWritten){
            std::cout << "Write vehicle airspeed Data failed..." << std::endl;
        }
        logToFile(doc);
}

//!
//! \brief sendVehicleFuel Send vehicle fuel data to the MACE GUI
//! \param vehicleID Vehicle ID with the new fuel update
//! \param component Vehicle battery component
//!
void MLMACEtoGUI::sendVehicleFuel(const int &vehicleID, const std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_Battery> &component)
{

    QJsonDocument doc(component->toJSON(vehicleID,MLMessageString(MLMessageTypes::VEHICLE_FUEL)));
    bool bytesWritten = writeUDPData(doc.toJson());

    if(!bytesWritten){
        std::cout << "Write Fuel Data failed..." << std::endl;
    }
    logToFile(doc);

}


//!
//! \brief sendEnvironmentVertices Send environment boundary vertices to the MACE GUI
//! \param component Environment boundary component
//!
void MLMACEtoGUI::sendEnvironmentVertices(const std::vector<mace::pose::GeodeticPosition_3D> &component) {

    QJsonObject json;
    json["message_type"] = QString::fromStdString(MLMessageString(MLMessageTypes::ENVIRONMENT_BOUNDARY));
    json["agentID"] = 0;

    QJsonArray verticies;
    for(auto&& vertex : component) {
        QJsonObject obj;
        obj["lat"] = vertex.getLatitude();
        obj["lng"] = vertex.getLongitude();
        obj["alt"] = vertex.getAltitude();

        verticies.push_back(obj);
    }

    json["environmentBoundary"] = verticies;

    QJsonDocument doc(json);
    bool bytesWritten = writeUDPData(doc.toJson());

    if(!bytesWritten){
        std::cout << "Write environment boundary failed..." << std::endl;
    }
}

//!
//! \brief writeTCPData Write data to the MACE GUI via TCP
//! \param data Data to be sent to the MACE GUI
//! \return True: success / False: failure
//!
bool MLMACEtoGUI::writeTCPData(QByteArray data)
{
    std::shared_ptr<QTcpSocket> tcpSocket = std::make_shared<QTcpSocket>();
    tcpSocket->connectToHost(m_sendAddress, m_sendPort);
    tcpSocket->waitForConnected();
    if(tcpSocket->state() == QAbstractSocket::ConnectedState)
    {
        tcpSocket->write(data); //write the data itself
        tcpSocket->flush();
        tcpSocket->waitForBytesWritten();
        return true;
    } else {
        std::cout << "TCP socket not connected MACE TO MLGUI" << std::endl;
        tcpSocket->close();
        return false;
    }
}

//!
//! \brief writeUDPData Write data to the MACE GUI via UDP
//! \param data Data to be sent to the MACE GUI
//! \return True: success / False: failure
//!
bool MLMACEtoGUI::writeUDPData(QByteArray data)
{
    // TODO: implement
    // Use UdpLink
    if(m_udpLink->isConnected()) {
        m_udpLink->WriteBytes(data, data.length());
    }
    else {
        std::cout << "UDP Link not connected..." << std::endl;
        return false;
    }

    return true;
}
