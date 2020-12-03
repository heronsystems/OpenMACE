#include "mlguitomace.h"

MLGUItoMACE::MLGUItoMACE(const MaceCore::IModuleCommandMLStation* ptrRef) :
    m_sendAddress(QHostAddress::LocalHost),
    m_sendPort(1234),
    goalSpace(nullptr),
    m_goalSampler(nullptr),
    m_udpConfig("127.0.0.1", 5678, "127.0.0.1", 9080)
{
    m_udpLink = std::make_shared<CommsMACE::UdpLink>(m_udpConfig);
    m_udpLink->Connect();
    m_parent = ptrRef;
    goalSpace = std::make_shared<mace::state_space::Cartesian2DSpace>();
    mace::state_space::Cartesian2DSpaceBounds bounds(-20,20,-10,10);
    goalSpace->setBounds(bounds);

    m_goalSampler = std::make_shared<mace::state_space::Cartesian2DSpace_Sampler>(goalSpace);
}

MLGUItoMACE::MLGUItoMACE(const MaceCore::IModuleCommandMLStation* ptrRef, const QHostAddress &sendAddress, const int &sendPort) :
    m_sendAddress(sendAddress),
    m_sendPort(sendPort),
    m_udpConfig("127.0.0.1", 5678, sendAddress.toString().toStdString(), sendPort)

{
    m_parent = ptrRef;

    m_udpLink = std::make_shared<CommsMACE::UdpLink>(m_udpConfig);
    m_udpLink->Connect();
}

MLGUItoMACE::~MLGUItoMACE() {
    m_logger->flush();
}

bool MLGUItoMACE::IsSpectreActive(const int &vehicleID){
    auto vehicle = m_currentModels.find(vehicleID);
    if (vehicle != m_currentModels.end()){
        if(ModelTypeIsSpectre(vehicle->second)){
            return true;
        }
    }
    return false;
}

void MLGUItoMACE::CheckRuntime(){
    if(m_cutoff > 1){
        if (QDateTime::currentMSecsSinceEpoch() >= m_cutoff){
            this->endTest();
        }
    }
}

void MLGUItoMACE::initiateLogs(const std::string &loggerName, const std::string &loggingPath)
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
//! \brief setSendAddress Set the TCP send address for MACE-to-GUI comms
//! \param sendAddress TCP send address
//!
void MLGUItoMACE::setSendAddress(const QHostAddress &sendAddress) {
    m_sendAddress = sendAddress;
    m_udpConfig.setSenderAddress(sendAddress.toString().toStdString());
}

//!
//! \brief setSendPort Set the TCP send port for MACE-to-GUI comms
//! \param sendPort TCP send port
//!
void MLGUItoMACE::setSendPort(const int &sendPort) {
    m_sendPort = sendPort;
    m_udpConfig.setSenderPortNumber(sendPort);
}

//!
//! \brief getEnvironmentBoundary Initiate a request to MACE core for the current environment boundary vertices
//!
void MLGUItoMACE::getEnvironmentBoundary() {
    std::shared_ptr<const MaceCore::MaceData> data = m_parent->getDataObject();
    if (data->EnvironmentBoundarySet()) {
        BoundaryItem::EnvironmentBoundary environmentBoundary = data->GetEnvironmentBoundary();

        QJsonObject json;
        json["message_type"] = QString::fromStdString(MLMessageString(MLMessageTypes::ENVIRONMENT_BOUNDARY));
        json["boundary_name"] = QString::fromStdString(environmentBoundary.getName());
        json["boundary_type"] = QString::fromStdString(environmentBoundary.getType());

        QJsonArray vertices;
        for(auto&& vertex : environmentBoundary.getVertices()) {
            QJsonObject obj;
            obj["lat"] = vertex.getLatitude();
            obj["lng"] = vertex.getLongitude();
            obj["alt"] = 0.0;

            vertices.push_back(obj);
        }

        json["vertices"] = vertices;

        QJsonDocument doc(json);
        bool bytesWritten = writeUDPData(doc.toJson());

        if(!bytesWritten){
            std::cout << "Write environment boundary failed..." << std::endl;
        }
    }
}

//!
//! \brief startRound Instruct MACE to run the given adept modules with the parameters specified
//!
void MLGUItoMACE::startTest(const QJsonArray &aircraft, const QJsonArray &data) {
    DataGenericItem::DataGenericItem_MLTest startCommand;
    startCommand.setDescriptor(QJsonDocument::fromJson(data.last().toString().toUtf8()));
    int counter = 0;
    std::string logmessage = std::string("\"message_type\":\"test_start\", ") + "\"agentID\":\"";
    foreach (const QJsonValue &agent, aircraft)
    {
        QJsonDocument tmpDoc = QJsonDocument::fromJson(data[counter].toString().toUtf8());
        startCommand.addAircraft(agent.toString().toStdString(), tmpDoc);
        logmessage += " " + agent.toString().toStdString();
        counter++;
    }
    logToFile(logmessage + "\", " + data.last().toString().toStdString());

    if (startCommand.getDescriptor().getTermType() == Data::AdeptTerminateType::RUNTIME){
        m_cutoff = QDateTime::currentMSecsSinceEpoch() + startCommand.getDescriptor().getTermValue()*1000;
    }

    m_parent->NotifyListeners([&](MaceCore::IModuleEventsMLStation* ptr) {
        ptr->Event_StartRound(m_parent,startCommand);
    });

    m_currentModels = startCommand.getAgentModels();
}

//!
//! \brief endRound Instruct MACE to stop a currently running test
//!
void MLGUItoMACE::endTest() {
    m_cutoff = 0;
    m_currentModels.clear();

    logToFile("\"message_type\":\"test_end\" ");

    m_parent->NotifyListeners([&](MaceCore::IModuleEventsMLStation* ptr) {
        ptr->Event_EndRound(m_parent);
    });

    QJsonObject json;
    json["message_type"] = MLMessageString(MLMessageTypes::ENDTEST).c_str();
    QJsonDocument doc(json);
    bool bytesWritten = writeUDPData(doc.toJson());

    if(!bytesWritten){
        std::cout << "Write New Vehicle Data failed..." << std::endl;
    }
}

void MLGUItoMACE::markTime(const QJsonArray &aircraft, const QJsonArray &data){
    std::string vehicleID = aircraft.first().toString().toStdString();
    std::string time = data.first().toString().toStdString();
    logToFile(std::string("\"message_type\":\"mark_time\", ") + "\"agentID\":\"" + vehicleID + "\", \"TIME\":\"" + time + "\"");

    m_parent->NotifyListeners([&](MaceCore::IModuleEventsMLStation* ptr) {
        ptr->Event_MarkTime(m_parent, vehicleID, time);
    });
}

//!
//! \brief setEnvironmentVertices GUI command to set new environment boundary vertices
//! \param jsonObj JSON data containing the new environment vertices
//!
void MLGUItoMACE::setEnvironmentVertices(const QJsonDocument &data)
{
    UNUSED(data);
}


//!
//! \brief getConnectedVehicles Initiate a request to MACE Core for the list of currently connected vehicles
//!
void MLGUItoMACE::getConnectedVehicles()
{
//    mLogs->debug("Module  ML Ground Station saw a request for getting connected vehicles.");

    std::shared_ptr<const MaceCore::MaceData> macedata = m_parent->getDataObject();
    std::vector<std::string> vehicleIDs;
    macedata->GetAvailableMLVehicles(vehicleIDs);

    QJsonArray ids;
    if(vehicleIDs.size() > 0){
        for (const std::string& i : vehicleIDs) {
            ids.append(i.c_str());
        }
    }
    else {
        std::cout << "No vehicles currently available" << std::endl;
    }

    QJsonObject json;
    json["message_type"] = MLMessageString(MLMessageTypes::CONNECTED_VEHICLES).c_str();
    json["connectedVehicles"] = ids;

    QJsonDocument doc(json);
    bool bytesWritten = writeUDPData(doc.toJson());

    if(!bytesWritten){
        std::cout << "Write New Vehicle Data failed..." << std::endl;
    }
}


//!
//! \brief parseTCPRequest Parse data that has been sent to MACE via the MACE GUI
//! \param jsonObj JSON data to parse from the MACE GUI
//!
void MLGUItoMACE::parseTCPRequest(const QJsonObject &jsonObj)
{
    qDebug() << jsonObj;

    std::string command = jsonObj["command"].toString().toStdString();
    QJsonArray aircraft = jsonObj["aircraft"].toArray();
    QJsonArray data = jsonObj["data"].toArray();

    if (command == "TEST_END")
    {
        endTest();
    }
    else if (command == "TEST_START")
    {
       startTest(aircraft, data);
    }
    else if (command == "MARK_TIME")
    {
        markTime(aircraft, data);
    } else if (command == "GET_CONNECTED_VEHICLES")
    {
        getConnectedVehicles();
    }

}

//!
//! \brief writeTCPData Write data to the MACE GUI via TCP
//! \param data Data to be sent to the MACE GUI
//! \return True: success / False: failure
//!
bool MLGUItoMACE::writeTCPData(QByteArray data)
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
    }
    else
    {
        std::cout << "TCP socket not connected MLGUI TO MACE" << std::endl;
        std::cout << tcpSocket->errorString().toStdString() << std::endl;
        tcpSocket->close();
        return false;
    }
}

//!
//! \brief writeUDPData Write data to the MACE GUI via UDP
//! \param data Data to be sent to the MACE GUI
//! \return True: success / False: failure
//!
bool MLGUItoMACE::writeUDPData(QByteArray data)
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

