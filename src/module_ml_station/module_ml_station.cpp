#include "module_ml_station.h"
#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <functional>

#include <QCoreApplication>
#include <QString>
#include <QDataStream>

#include "mace_core/module_factory.h"

class ServerThread : public QThread
{
public:
    ServerThread(const std::function<void(void)> &func):
        m_func(func),
        m_Shutdown(false)
    {
        if(QCoreApplication::instance() == nullptr)
        {
            int argc = 0;
            char * argv[] = {(char *)"sharedlib.app"};
            pApp = new QCoreApplication(argc, argv);
        }
    }

    virtual void run()
    {
        while(true)
        {
            if(m_Shutdown == true)
            {
                break;
            }
            QCoreApplication::processEvents();
            m_func();
        }
    }

    virtual void shutdown()
    {
        m_Shutdown = true;
        this->wait();
    }

private:

    std::function<void(void)> m_func;
    QCoreApplication *pApp;
    bool m_Shutdown;
};

ModuleMLStation::ModuleMLStation() :
    m_SensorDataTopic("sensorData"),
    m_SensorFootprintDataTopic("sensorFootprint"),
    m_VehicleDataTopic("vehicleData"),
    m_ListenThread(nullptr),
    m_guiHostAddress(QHostAddress::LocalHost),
    m_listenPort(5678)
{

    // Start timer:
    m_timer = std::make_shared<MLTimer>([=]()
    {
        if(m_TcpServer) {
            if(!m_TcpServer->isListening()) {
                std::cout << "Server status: Disconnected" << std::endl;
            }
        }
    });

    m_timer->setSingleShot(false);
    m_timer->setInterval(MLTimer::Interval(300));
    m_timer->start(true);

    m_toMACEHandler = std::make_shared<MLGUItoMACE>(this);
    m_toGUIHandler = std::make_shared<MLMACEtoGUI>();
}

ModuleMLStation::~ModuleMLStation()
{
    if(m_ListenThread != nullptr)
    {
        ((ServerThread*)m_ListenThread)->shutdown();
        delete m_ListenThread;
    }

    if(m_timer)
    {
        m_timer->stop();
    }

    if(m_TcpServer) {
        m_TcpServer->close();
    }
}

std::vector<MaceCore::TopicCharacteristic> ModuleMLStation::GetEmittedTopics()
{
    std::vector<MaceCore::TopicCharacteristic> topics;
    return topics;
}

//!
//! \brief initiateLogs Start log files and logging for the ML Station module
//!
void ModuleMLStation::initiateLogs()
{

}

//!
//! \brief Starts the TCP server for the GCS to send requests to
//! \return
//!
bool ModuleMLStation::StartTCPServer()
{
    m_TcpServer = std::make_shared<QTcpServer>();
    m_ListenThread = new ServerThread([&](){
        if(m_TcpServer->hasPendingConnections())
            this->on_newConnection();
    });

    // For some reason, listening on any other specific address (i.e. not Any) fails.
    //      - As a workaround, I check the incoming connection below for equality with the guiHostAddress before parsing
    m_TcpServer->listen(QHostAddress::Any, m_listenPort);

//    m_TcpServer->listen(m_guiHostAddress, m_listenPort);

    m_TcpServer->moveToThread(m_ListenThread);
    m_ListenThread->start();


    if(!m_TcpServer->isListening())
    {
        std::cout << "Server could not start..." << std::endl;
    }
    else
    {
        std::cout << "MLGUI TCP Server started" << std::endl;
    }

    return m_TcpServer->isListening();
}

//!
//! \brief on_newConnection Slot to fire when a new TCP connection is initiated
//!
void ModuleMLStation::on_newConnection()
{
    while (m_TcpServer->hasPendingConnections())
    {
        QTcpSocket *socket = m_TcpServer->nextPendingConnection();

        // Workaround for server listening on Any address (see above)
        //  - Compare incoming address with guiHostAddress. IF the same continue, otherwise break out.
        std::string peerAddress = socket->peerAddress().toString().toStdString();
        // Split string and find just the address part:
        std::size_t found = peerAddress.find_last_of(":\\");
        peerAddress = peerAddress.substr(found+1);
        if(peerAddress != m_guiHostAddress.toString().toStdString()) {
            std::cout << "Unknown IP address: " << peerAddress << ". Compare to MLGUI Host Address: " << m_guiHostAddress.toString().toStdString() << std::endl;
            return;
        }

        while (socket->waitForReadyRead())
        {
            QByteArray data = socket->readAll();
            QJsonObject jsonObj;
            QJsonDocument doc = QJsonDocument::fromJson(data);
            // check validity of the document
            if(!doc.isNull())
            {
                if(doc.isObject())
                {
                    jsonObj = doc.object();
                    m_toMACEHandler->parseTCPRequest(jsonObj);
                }
                else
                {
                    std::cout << "Command is not a valid JSON object." << std::endl;
                    socket->close();
                    return;
                }
            }
            else
            {
                std::cout << "Invalid JSON..." << std::endl;
                std::cout << data.toStdString() << std::endl;
                socket->close();
                return;
            }

            QByteArray returnData("CommandSeen");
            socket->write(returnData);
            socket->flush();
            socket->waitForBytesWritten(3000);
        }

        // TODO-PAT: Try to leave this socket open if possible??
        socket->close();
    }
}



//!
//! \brief Describes the strucure of the parameters for this module
//! \return Strucure
//!
std::shared_ptr<MaceCore::ModuleParameterStructure> ModuleMLStation::ModuleConfigurationStructure() const
{
    MaceCore::ModuleParameterStructure structure;
    std::shared_ptr<MaceCore::ModuleParameterStructure> maceCommsParams = std::make_shared<MaceCore::ModuleParameterStructure>();
    maceCommsParams->AddTerminalParameters("GUIHostAddress", MaceCore::ModuleParameterTerminalTypes::STRING, false);
    maceCommsParams->AddTerminalParameters("ListenPort", MaceCore::ModuleParameterTerminalTypes::INT, false);
    maceCommsParams->AddTerminalParameters("SendPort", MaceCore::ModuleParameterTerminalTypes::INT, false);
    structure.AddNonTerminal("MACEComms", maceCommsParams, false);

    structure.AddTerminalParameters("ID", MaceCore::ModuleParameterTerminalTypes::INT, false);

    return std::make_shared<MaceCore::ModuleParameterStructure>(structure);
}


//!
//! \brief Provides object contains parameters values to configure module with
//! \param params Parameters to configure
//!
void ModuleMLStation::ConfigureModule(const std::shared_ptr<MaceCore::ModuleParameterValue> &params)
{
    QHostAddress guiHostAddress;
    int listenPort = 5678;
    int sendPort = 1234;
    if(params->HasNonTerminal("MACEComms")) {
        std::shared_ptr<MaceCore::ModuleParameterValue> maceCommsXML = params->GetNonTerminalValue("MACEComms");
        if(maceCommsXML->HasTerminal("GUIHostAddress")) {
            std::string hostAddress = maceCommsXML->GetTerminalValue<std::string>("GUIHostAddress");
            guiHostAddress = QHostAddress(QString::fromStdString(hostAddress));
//            sendAddress = QHostAddress(QString::fromStdString(hostAddress));
        }
        if(maceCommsXML->HasTerminal("ListenPort")) {
            listenPort = maceCommsXML->GetTerminalValue<int>("ListenPort");
        }
        if(maceCommsXML->HasTerminal("SendPort")) {
            sendPort = maceCommsXML->GetTerminalValue<int>("SendPort");
        }
    }


    if(params->HasTerminal("ID"))
    {
        this->SetID(params->GetTerminalValue<int>("ID"));
    }

    m_guiHostAddress = guiHostAddress;
    m_listenPort = listenPort;

    m_toGUIHandler->setSendAddress(guiHostAddress);
    m_toGUIHandler->setSendPort(sendPort);

    m_toMACEHandler->setSendAddress(guiHostAddress);
    m_toMACEHandler->setSendPort(sendPort);

}

//!
//! \brief AssignLoggingDirectory
//! \param path
//!
void ModuleMLStation::AssignLoggingDirectory(const std::string &path)
{
    ModuleBase::AssignLoggingDirectory(path);
    initiateLogs();
}

//!
//! \brief start Start event listener thread
//!
void ModuleMLStation::start()
{
    AbstractModule_EventListeners::start();
}

//!
//! \brief This module as been attached as a module
//! \param ptr pointer to object that attached this instance to itself
//!
void ModuleMLStation::AttachedAsModule(MaceCore::IModuleTopicEvents *ptr)
{
    ptr->Subscribe(this, m_VehicleDataTopic.Name());
    ptr->Subscribe(this, m_SensorDataTopic.Name());
    ptr->Subscribe(this, m_SensorFootprintDataTopic.Name());

}


//!
//! \brief New non-spooled topic given
//!
//! NonSpooled topics send their data immediatly.
//! \param topicName Name of stopic
//! \param sender Module that sent topic
//! \param data Data for topic
//! \param target Target module (or broadcasted)
//!
void ModuleMLStation::NewTopicData(const std::string &topicName, const MaceCore::ModuleCharacteristic &sender, const MaceCore::TopicDatagram &data, const OptionalParameter<MaceCore::ModuleCharacteristic> &target)
{
    UNUSED(topicName);
    UNUSED(sender);
    UNUSED(data);
    UNUSED(target);
}


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
void ModuleMLStation::NewTopicSpooled(const std::string &topicName, const MaceCore::ModuleCharacteristic &sender, const std::vector<std::string> &componentsUpdated, const OptionalParameter<MaceCore::ModuleCharacteristic> &target)
{
    UNUSED(target);

    uint8_t vehicleID;
    if(this->getDataObject()->getMavlinkIDFromModule(sender, vehicleID)) {
        //example read of vehicle data
        if(topicName == m_VehicleDataTopic.Name())
        {
            //get latest datagram from mace_data
            MaceCore::TopicDatagram read_topicDatagram = this->getDataObject()->GetCurrentTopicDatagram(m_VehicleDataTopic.Name(), sender);

            //example of how to get data and parse through the components that were updated
            for(size_t i = 0 ; i < componentsUpdated.size() ; i++) {
                if(componentsUpdated.at(i) == mace::pose_topics::Topic_AgentOrientation::Name()) {
                    std::shared_ptr<mace::pose_topics::Topic_AgentOrientation> component = std::make_shared<mace::pose_topics::Topic_AgentOrientation>();
                    m_VehicleDataTopic.GetComponent(component, read_topicDatagram);

                    // Write Attitude data to the GUI:
                    m_toGUIHandler->sendAttitudeData(vehicleID, component);
                }

                else if(componentsUpdated.at(i) == mace::pose_topics::Topic_GeodeticPosition::Name()) {
                    std::shared_ptr<mace::pose_topics::Topic_GeodeticPosition> component = std::make_shared<mace::pose_topics::Topic_GeodeticPosition>();
                    m_VehicleDataTopic.GetComponent(component, read_topicDatagram);

                    // Write Position data to the GUI:
                    m_toGUIHandler->sendPositionData(vehicleID, component);
                }
                else if(componentsUpdated.at(i) == mace::measurement_topics::Topic_AirSpeed::Name()) {
                    mace::measurement_topics::Topic_AirSpeedPtr component = std::make_shared<mace::measurement_topics::Topic_AirSpeed>();
                    m_VehicleDataTopic.GetComponent(component, read_topicDatagram);
                    
                    // Write Airspeed data to the GUI:
                    m_toGUIHandler->sendVehicleAirspeed(vehicleID, component);
                }
                else if(componentsUpdated.at(i) == DataGenericItemTopic::DataGenericItemTopic_Battery::Name()) {
                    std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_Battery> component = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_Battery>();
                    m_VehicleDataTopic.GetComponent(component, read_topicDatagram);
                    
                    // Write fueld data to the GUI:
                    m_toGUIHandler->sendVehicleFuel(vehicleID, component);
                }
            }
        }
    }
    
}


//!
//! \brief NewlyAvailableVehicle Subscriber to a newly available vehilce topic
//! \param vehicleID Vehilce ID of the newly available vehicle
//!
void ModuleMLStation::NewlyAvailableVehicle(const int &vehicleID, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    UNUSED(vehicleID); UNUSED(sender);

    std::shared_ptr<const MaceCore::MaceData> data = this->getDataObject();
    std::vector<unsigned int> vehicleIDs;
    data->GetAvailableVehicles(vehicleIDs);

    QJsonArray ids;
    if(vehicleIDs.size() > 0){
        for (const unsigned int& i : vehicleIDs) {
            ids.append((int)i);
        }
    }
    else {
        std::cout << "No vehicles currently available" << std::endl;
    }

    QJsonObject json;
    json["message_type"] = MLMessageString(MLMessageTypes::CONNECTED_VEHICLES).c_str();
    json["connectedVehicles"] = ids;

    QJsonDocument doc(json);
    bool bytesWritten = m_toGUIHandler->writeTCPData(doc.toJson());

    if(!bytesWritten){
        std::cout << "Write ConnectedVehicles failed..." << std::endl;
    }
}

void ModuleMLStation::NewlyAvailableParameterList(const std::map<std::string, DataGenericItem::DataGenericItem_ParamValue> &params, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    uint8_t vehicleID;
    this->getDataObject()->getMavlinkIDFromModule(sender.Value(), vehicleID);
    m_toGUIHandler->sendVehicleParameterList(vehicleID, params);
}


void ModuleMLStation::NewlyAvailableBoundary(const uint8_t &key, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    UNUSED(key);
    UNUSED(sender);

    /* MTB - Removing 7/2/2018
     * @pnolan Issue: 138
     *
     * GS rendering of a boundary needs finer tuning. It needs to check what boundaries it already knows about, and selectivly render resource fence vs operation boundary
     *
     * I am removing for now to avoid these issues hindering progress.
    BoundaryItem::BoundaryList boundary;
    if(this->getDataObject()->getBoundaryFromIdentifier(key, boundary))
    {
        GeodeticPosition_3D origin = this->getDataObject()->GetGlobalOrigin();

        std::vector<mace::pose::Position<mace::pose::CartesianPosition_2D>> lVertices = boundary.boundingPolygon.getVector();
        std::vector<mace::pose::GeodeticPosition_3D> gVertices;
        for(size_t i = 0; i < lVertices.size(); i++)
        {
            GeodeticPosition_3D gPos3D;
            CartesianPosition_3D lpos3D(lVertices.at(i).getXPosition(),lVertices.at(i).getYPosition(),0.0);
            mace::pose::DynamicsAid::LocalPositionToGlobal(origin,lpos3D,gPos3D);
            gVertices.push_back(gPos3D);
        }
        //Write to the GUI
        m_toGUIHandler->sendEnvironmentVertices(gVertices);
    }
    */
}




