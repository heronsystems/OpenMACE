#include "guitomace.h"

GUItoMACE::GUItoMACE(const MaceCore::IModuleCommandGroundStation* ptrRef) :
    m_sendAddress(QHostAddress::LocalHost),
    m_sendPort(1234),
    goalSpace(nullptr),
    m_goalSampler(nullptr),
    m_udpConfig("127.0.0.1", 5678, m_sendAddress.toString().toStdString(), m_sendPort)
{
    m_udpLink = std::make_shared<CommsMACE::UdpLink>(m_udpConfig);
    m_udpLink->Connect();

    m_parent = ptrRef;
    goalSpace = std::make_shared<mace::state_space::Cartesian2DSpace>();
    mace::state_space::Cartesian2DSpaceBounds bounds(-20,20,-10,10);
    goalSpace->setBounds(bounds);

    m_goalSampler = std::make_shared<mace::state_space::Cartesian2DSpace_Sampler>(goalSpace);
}

GUItoMACE::GUItoMACE(const MaceCore::IModuleCommandGroundStation* ptrRef, const QHostAddress &sendAddress, const int &sendPort) :
    m_sendAddress(sendAddress),
    m_sendPort(sendPort),
    m_udpConfig("127.0.0.1", 5678, sendAddress.toString().toStdString(), sendPort)
{
    m_parent = ptrRef;

    m_udpLink = std::make_shared<CommsMACE::UdpLink>(m_udpConfig);
    m_udpLink->Connect();
}

GUItoMACE::~GUItoMACE() {
    m_logger->flush();
}

//!
//! \brief initiateLogs Start log files and logging for the Ground Station module
//!
void GUItoMACE::initiateLogs(const std::string &loggerName, const std::string &loggingPath)
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
void GUItoMACE::setSendAddress(const QHostAddress &sendAddress) {
    m_sendAddress = sendAddress;
}

//!
//! \brief setSendPort Set the TCP send port for MACE-to-GUI comms
//! \param sendPort TCP send port
//!
void GUItoMACE::setSendPort(const int &sendPort) {
    m_sendPort = sendPort;
}

//!
//! \brief getEnvironmentBoundary Initiate a request to MACE core for the current environment boundary vertices
//!
void GUItoMACE::getEnvironmentBoundary() {
    std::shared_ptr<const MaceCore::MaceData> data = m_parent->getDataObject();
    if (data->EnvironmentBoundarySet()) {
        BoundaryItem::EnvironmentBoundary environmentBoundary = data->GetEnvironmentBoundary();

        QJsonObject json;
        json["message_type"] = QString::fromStdString(guiMessageString(GuiMessageTypes::ENVIRONMENT_BOUNDARY));
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
        //    bool bytesWritten = writeTCPData(doc.toJson());
        bool bytesWritten = writeUDPData(doc.toJson());

        if(!bytesWritten){
            std::cout << "Write environment boundary failed..." << std::endl;
        }
    }
}

//!
//! \brief getGlobalOrigin Initiate a request to MACE core for the current global origin position
//!
void GUItoMACE::getGlobalOrigin()
{
    mace::pose::GeodeticPosition_3D globalOrigin = m_parent->getDataObject()->GetGlobalOrigin();

    QJsonObject json;
    json["dataType"] = "GlobalOrigin";
    json["vehicleID"] = 0;
    json["lat"] = globalOrigin.getLatitude();
    json["lng"] = globalOrigin.getLongitude();
    json["alt"] = globalOrigin.getAltitude();
    json["gridSpacing"] = m_parent->getDataObject()->GetGridSpacing();

    QJsonDocument doc(json);
//    bool bytesWritten = writeTCPData(doc.toJson());
    bool bytesWritten = writeUDPData(doc.toJson());

    if(!bytesWritten){
        std::cout << "Write global origin failed..." << std::endl;
    }
}

//!
//! \brief setVehicleHome GUI command to set a new vehicle home position
//! \param vehicleID Vehicle ID that the command is initated from (0 = all vehicles)
//! \param jsonObj JSON data containing the new vehicle home data
//!
void GUItoMACE::setVehicleHome(const int &vehicleID, const QJsonDocument &data)
{
    command_item::SpatialHome tmpHome;
    tmpHome.setTargetSystem(vehicleID);
    double referenceAlt =  m_parent->getDataObject()->GetVehicleHomePostion(vehicleID).getPosition()->getDataVector().z();
    mace::pose::GeodeticPosition_3D homePosition(data.object().value("lat").toDouble(),
                                                 data.object().value("lng").toDouble(),
                                                 data.object().value("alt").toDouble() + referenceAlt); // TODO-PAT/AARON: Figure out a more robust way to handle the home position altitude conversion
    tmpHome.setPosition(&homePosition);

    std::stringstream buffer;
    buffer << tmpHome;
//    mLogs->debug("Module Ground Station issuing a new vehicle home to system " + std::to_string(vehicleID) + ".");
//    mLogs->info(buffer.str());

    m_parent->NotifyListeners([&](MaceCore::IModuleEventsGroundStation* ptr) {
        ptr->Event_SetHomePosition(m_parent, tmpHome);
    });
}

//!
//! \brief setGlobalOrigin GUI command to set a new global origin position
//! \param jsonObj JSON data containing the new global origin data
//!
void GUItoMACE::setGlobalOrigin(const QJsonDocument &data)
{
    mace::pose::GeodeticPosition_3D origin;
    origin.setCoordinateFrame(GeodeticFrameTypes::CF_GLOBAL_AMSL);
    origin.setAltitudeReferenceFrame(AltitudeReferenceTypes::REF_ALT_MSL);

    origin.setLatitude(data.object().value("lat").toDouble());
    origin.setLongitude(data.object().value("lng").toDouble());
    origin.setAltitude(data.object().value("alt").toDouble());

    m_parent->NotifyListeners([&](MaceCore::IModuleEventsGroundStation* ptr) {
        ptr->Event_SetGlobalOrigin(m_parent, origin);
    });
}

//!
//! \brief setEnvironmentVertices GUI command to set new environment boundary vertices
//! \param jsonObj JSON data containing the new environment vertices
//!
void GUItoMACE::setEnvironmentVertices(const QJsonDocument &data)
{
    UNUSED(data);
//    BoundaryItem::BoundaryList operationalBoundary;

//    mace::pose::GeodeticPosition_3D origin = m_parent->getDataObject()->GetGlobalOrigin();

//    foreach(const QJsonValue & v, data) {
//        std::cout << "Lat: " << v.toObject().value("lat").toDouble() << " / Lon: " << v.toObject().value("lng").toDouble() << std::endl;
//        double tmpLat = v.toObject().value("lat").toDouble();
//        double tmpLon = v.toObject().value("lng").toDouble();
//        double tmpAlt = v.toObject().value("alt").toDouble();

//        mace::pose::GeodeticPosition_3D vertexGlobal(tmpLat,tmpLon,tmpAlt);
//        mace::pose::CartesianPosition_3D vertexLocal3D;

//        if(origin.isAnyPositionValid()) {
//            mace::pose::DynamicsAid::GlobalPositionToLocal(&origin,&vertexGlobal,&vertexLocal3D);
//            mace::pose::CartesianPosition_2D vertexLocal2D(vertexLocal3D.getXPosition(),vertexLocal3D.getYPosition());

//            operationalBoundary.appendVertexItem(&vertexLocal2D);
//        }
//    }

//    BoundaryItem::BoundaryCharacterisic key(BoundaryItem::BOUNDARYTYPE::OPERATIONAL_FENCE);

//    if(operationalBoundary.getQueueSize() > 0) {
//        m_parent->NotifyListeners([&](MaceCore::IModuleEventsGroundStation* ptr) {
//            ptr->Event_SetBoundary(m_parent, key, operationalBoundary);
//        });
//    }

//    // Get and send vertices to the GUI:
//    getEnvironmentBoundary();
}

//!
//! \brief setGoHere GUI command to set a new "go here" lat/lon/alt position
//! \param vehicleID Vehicle ID that the command is initated from (0 = all vehicles)
//! \param jsonObj JSON data containing the "go here" position
//!
void GUItoMACE::setGoHere(const int &vehicleID, const QJsonDocument &data)
{
    // TODO:
    std::cout << "Go here command issued" << std::endl;

    //Ken Fix: Target system should propogate or not exist at the mission item level using action/command logic
    command_item::Action_ExecuteSpatialItem cmdGoTo;
    cmdGoTo.setTargetSystem(vehicleID);

    command_item::SpatialWaypointPtr spatialAction = std::make_shared<command_item::SpatialWaypoint>(vehicleID);
//    spatialAction->getPosition().setCoordinateFrame(Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT);

    mace::pose::GeodeticPosition_3D goPosition(data.object().value("lat").toDouble(),
                                               data.object().value("lng").toDouble(),
                                               data.object().value("alt").toDouble()); // TODO: Either set altitude from GUI or stay at same altitude as vehicle
    spatialAction->setPosition(&goPosition);
    cmdGoTo.setSpatialAction(spatialAction);

    m_parent->NotifyListeners([&](MaceCore::IModuleEventsGroundStation* ptr){
        ptr->Event_IssueCommandGoTo(m_parent, cmdGoTo);
    });
}

//!
//! \brief takeoff GUI command initiating a takeoff
//! \param vehicleID Vehicle ID that the command is initated from (0 = all vehicles)
//! \param jsonObj JSON data containing the takeoff position and altitude
//!
void GUItoMACE::takeoff(const int &vehicleID, const QJsonDocument &data)
{
    command_item::SpatialTakeoff newTakeoff;
    bool latLonFlag = data.object().value("latLonFlag").toBool();

    mace::pose::GeodeticPosition_3D takeoffPosition;

    if(latLonFlag) {
        takeoffPosition.setLatitude(data.object().value("takeoffPosition").toObject()["lat"].toDouble());
        takeoffPosition.setLongitude(data.object().value("takeoffPosition").toObject()["lng"].toDouble());
    }
    takeoffPosition.setAltitude(data.object().value("takeoffPosition").toObject()["alt"].toDouble());
    newTakeoff.setPosition(&takeoffPosition);
    newTakeoff.setTargetSystem(vehicleID);

    std::stringstream buffer;
    buffer << newTakeoff;
//    mLogs->debug("Module Ground Station issuing a takeoff command to system " + std::to_string(vehicleID) + ".");
//    mLogs->info(buffer.str());

    m_parent->NotifyListeners([&](MaceCore::IModuleEventsGroundStation* ptr){
        ptr->Event_IssueCommandTakeoff(m_parent, newTakeoff);
    });
}

//!
//! \brief issueCommand Issue command via the GUI to MACE
//! \param vehicleID Vehicle ID that the command is initated from (0 = all vehicles)
//! \param jsonObj JSON data containing the command to be issued
//!
bool GUItoMACE::issuedCommand(const std::string &command, const int &vehicleID, const QJsonDocument &data)
{
    if(command == "FORCE_DATA_SYNC") {
//        mLogs->debug("Module Ground Station issuing command force data sync to system " + std::to_string(vehicleID) + ".");
        m_parent->NotifyListeners([&](MaceCore::IModuleEventsGroundStation* ptr){
            ptr->Event_ForceVehicleDataSync(m_parent, vehicleID);
        });
        return true;
    }
    else if(command == "TAKEOFF"){
        takeoff(vehicleID, data);
        return true;
    }
    else if(command == "RTL") {
//        mLogs->debug("Module Ground Station issuing command RTL to system " + std::to_string(vehicleID) + ".");
        command_item::SpatialRTL rtlCommand;
        rtlCommand.setTargetSystem(vehicleID);
        // TODO: Set generating system and coordinate frame

        m_parent->NotifyListeners([&](MaceCore::IModuleEventsGroundStation* ptr){
            ptr->Event_IssueCommandRTL(m_parent, rtlCommand);
        });
        return true;
    }
    else if(command == "LAND") {
//        mLogs->debug("Module Ground Station issuing land command to system " + std::to_string(vehicleID) + ".");
        command_item::SpatialLand landCommand;
        landCommand.setTargetSystem(vehicleID);
        // TODO: Set generating system and coordinate frame

        m_parent->NotifyListeners([&](MaceCore::IModuleEventsGroundStation* ptr){
            ptr->Event_IssueCommandLand(m_parent, landCommand);
        });
        return true;
    }
    else if(command == "START_MISSION") {
//        mLogs->debug("Module Ground Station issuing mission start command to system " + std::to_string(vehicleID) + ".");
        command_item::ActionMissionCommand missionCommand;
        missionCommand.setMissionStart();
        missionCommand.setTargetSystem(vehicleID);
        // TODO: Set generating system and coordinate frame

        m_parent->NotifyListeners([&](MaceCore::IModuleEventsGroundStation* ptr){
            ptr->Event_IssueMissionCommand(m_parent, missionCommand);
        });
        return true;
    }
    else if(command== "PAUSE_MISSION") {
//        mLogs->debug("Module Ground Station issuing mission pause command to system " + std::to_string(vehicleID) + ".");
        command_item::ActionMissionCommand missionCommand;
        missionCommand.setMissionPause();
        missionCommand.setTargetSystem(vehicleID);
        // TODO: Set generating system and coordinate frame

        m_parent->NotifyListeners([&](MaceCore::IModuleEventsGroundStation* ptr){
            ptr->Event_IssueMissionCommand(m_parent, missionCommand);
        });
        return true;
    }
    else if(command == "RESUME_MISSION") {
//        mLogs->debug("Module Ground Station issuing mission resume command to system " + std::to_string(vehicleID) + ".");
        command_item::ActionMissionCommand missionCommand;
        missionCommand.setMissionResume();
        missionCommand.setTargetSystem(vehicleID);
        // TODO: Set generating system and coordinate frame

        m_parent->NotifyListeners([&](MaceCore::IModuleEventsGroundStation* ptr){
            ptr->Event_IssueMissionCommand(m_parent, missionCommand);
        });
        return true;
    }

    return false;

}


void GUItoMACE::testFunction1(const int &vehicleID)
{
    m_parent->NotifyListeners([&](MaceCore::IModuleEventsGroundStation* ptr){
        ptr->RequestDummyFunction(m_parent, vehicleID);
    });

//    command_item::Action_DynamicTarget newCommand;
//    newCommand.setTargetSystem(vehicleID);
//    newCommand.setOriginatingSystem(255);
//    command_target::DynamicTarget_Kinematic newTarget;
//    mace::pose::CartesianPosition_3D currentPositionTarget;
//    currentPositionTarget.setCoordinateFrame(CartesianFrameTypes::CF_LOCAL_OFFSET_NED);
//    currentPositionTarget.setAltitudeReferenceFrame(AltitudeReferenceTypes::REF_ALT_RELATIVE);
//    currentPositionTarget.updatePosition(10,0,0);
//    newTarget.setPosition(&currentPositionTarget);
////    mace::pose::Cartesian_Velocity3D currentVelocityTarget(CartesianFrameTypes::CF_LOCAL_NED);
////    currentVelocityTarget.setXVelocity(5.0);
////    currentVelocityTarget.setYVelocity(0.0);
////    currentVelocityTarget.setZVelocity(0.0);
////    newTarget.setVelocity(&currentVelocityTarget);

//    mace::pose::Rotation_2D yaw;
//    yaw.setPhi(M_PI_4);
//    newTarget.setYaw(&yaw);

//    newCommand.setDynamicTarget(&newTarget);
//    m_parent->NotifyListeners([&](MaceCore::IModuleEventsGroundStation* ptr){
//        ptr->EventPP_ExecuteDynamicTarget(m_parent, newCommand);
//    });

}

void GUItoMACE::testFunction2(const int &vehicleID)
{
    command_item::Action_DynamicTarget newCommand;
    newCommand.setTargetSystem(vehicleID);
    newCommand.setOriginatingSystem(255);
    command_target::DynamicTarget_Kinematic newTarget;
    mace::pose::CartesianPosition_3D currentPositionTarget;
    currentPositionTarget.setCoordinateFrame(CartesianFrameTypes::CF_BODY_OFFSET_NED);
    currentPositionTarget.setAltitudeReferenceFrame(AltitudeReferenceTypes::REF_ALT_RELATIVE);
    currentPositionTarget.updatePosition(10,0,0);
    newTarget.setPosition(&currentPositionTarget);
//    mace::pose::Cartesian_Velocity3D currentVelocityTarget(CartesianFrameTypes::CF_LOCAL_NED);
//    currentVelocityTarget.setXVelocity(5.0);
//    currentVelocityTarget.setYVelocity(0.0);
//    currentVelocityTarget.setZVelocity(0.0);
//    newTarget.setVelocity(&currentVelocityTarget);

    mace::pose::Rotation_2D yaw;
    yaw.setPhi(M_PI_4);
    newTarget.setYaw(&yaw);

    newCommand.setDynamicTarget(&newTarget);
    m_parent->NotifyListeners([&](MaceCore::IModuleEventsGroundStation* ptr){
        ptr->EventPP_ExecuteDynamicTarget(m_parent, newCommand);
    });
}


//!
//! \brief getConnectedVehicles Initiate a request to MACE Core for the list of currently connected vehicles
//!
void GUItoMACE::getConnectedVehicles()
{
//    mLogs->debug("Module Ground Station saw a request for getting connected vehicles.");

    std::shared_ptr<const MaceCore::MaceData> macedata = m_parent->getDataObject();
    std::vector<unsigned int> vehicleIDs;
    macedata->GetAvailableVehicles(vehicleIDs);

    QJsonArray ids;
    QJsonArray modes;
    if(vehicleIDs.size() > 0){
        for (const uint& i : vehicleIDs) {
            ids.append((int)i);
            std::string mode;
            macedata->GetVehicleFlightMode(i, mode);
            modes.append(QString::fromStdString(mode));
        }
    }
    else {
        std::cout << "No vehicles currently available" << std::endl;
    }

    QJsonObject connectedVehicles;
    connectedVehicles["ids"] = ids;
    connectedVehicles["modes"] = modes;

    QJsonObject json;
    json["dataType"] = "ConnectedVehicles";
    json["vehicleID"] = 0;
    json["connectedVehicles"] = connectedVehicles;

    QJsonDocument doc(json);
    //    bool bytesWritten = writeTCPData(doc.toJson());
    bool bytesWritten = writeUDPData(doc.toJson());

    if(!bytesWritten){
        std::cout << "Write New Vehicle Data failed..." << std::endl;
    }
}

//!
//! \brief getVehicleMission GUI command that grabs a vehicle mission from MACE
//! \param vehicleID Vehicle ID that the command is initated from (0 = all vehicles)
//!
void GUItoMACE::getVehicleMission(const int &vehicleID)
{
    m_parent->NotifyListeners([&](MaceCore::IModuleEventsGroundStation* ptr){
        ptr->Event_GetCurrentMission(this, vehicleID);
    });
}

//!
//! \brief getVehicleHome Initiate a request to MACE Core for the vehicle home location
//! \param vehicleID Vehicle ID that the command is initated from (0 = all vehicles)
//!
void GUItoMACE::getVehicleHome(const int &vehicleID)
{
    m_parent->NotifyListeners([&](MaceCore::IModuleEventsGroundStation* ptr){
        ptr->Event_GetHomePosition(this, vehicleID);
    });
}

//!
//! \brief setVehicleArm GUI command initiating a vehicle arm status change
//! \param vehicleID Vehicle ID that the command is initated from (0 = all vehicles)
//! \param jsonObj JSON data containing the new vehicle arm status
//!
void GUItoMACE::setVehicleArm(const int &vehicleID, const QJsonDocument &data)
{
    command_item::ActionArm tmpArm;
    tmpArm.setTargetSystem(vehicleID); // the vehicle ID coordinates to the specific vehicle //vehicle 0 is reserved for all connected vehicles
    tmpArm.setVehicleArm(data.object().value("arm").toBool());

    std::stringstream buffer;
    buffer << tmpArm;
//    mLogs->debug("Module Ground Station issuing a arm command to system " + std::to_string(vehicleID) + ".");
//    mLogs->info(buffer.str());

    m_parent->NotifyListeners([&](MaceCore::IModuleEventsGroundStation* ptr){
        ptr->Event_IssueCommandSystemArm(m_parent, tmpArm);
    });
}

//!
//! \brief setVehicleMode GUI command initiating a vehicle mode change
//! \param vehicleID Vehicle ID that the command is initated from (0 = all vehicles)
//! \param jsonObj JSON data containing the new vehicle mode
//!
void GUItoMACE::setVehicleMode(const int &vehicleID, const QJsonDocument &data)
{
    command_item::ActionChangeMode tmpMode;
    tmpMode.setTargetSystem(vehicleID); // the vehicle ID coordinates to the specific vehicle //vehicle 0 is reserved for all connected vehicles
    tmpMode.setRequestMode(data.object().value("mode").toString().toStdString()); //where the string here is the desired Flight Mode...available modes can be found in the appropriate topic
    std::cout<<"We are changing the vehicle mode as issued by the GUI: "<<tmpMode.getRequestMode()<<std::endl;

    m_parent->NotifyListeners([&](MaceCore::IModuleEventsGroundStation* ptr){
        ptr->Event_ChangeSystemMode(m_parent, tmpMode);
    });




//    std::shared_ptr<MaceCore::ITopicComponentPrototype> data = std::make_shared<Data::TopicComponents::String>(
//                    jsonObj["vehicleCommand"].toString().toStdString()
//                );


//    MaceCore::ModuleCharacteristic target;
//    target.ID = vehicleID;
//    target.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;

//    MaceCore::TopicDatagram topicDatagram;
//    m_VehicleTopics->m_CommandSystemMode.SetComponent(data, topicDatagram);
//    m_parent->NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
//        ptr->NewTopicDataValues(m_parent, m_VehicleTopics->m_CommandSystemMode.Name(), m_parent->GetCharacteristic(), MaceCore::TIME(), topicDatagram, target);
//    });
}

//!
//! \brief parseTCPRequest Parse data that has been sent to MACE via the MACE GUI
//! \param jsonObj JSON data to parse from the MACE GUI
//!
void GUItoMACE::parseTCPRequest(const QJsonObject &jsonObj)
{
    qDebug() << jsonObj;

    std::string command = jsonObj["command"].toString().toStdString();
    QJsonArray aircraft = jsonObj["aircraft"].toArray();
    QJsonArray data = jsonObj["data"].toArray();

    // Log to file:
    logToFile(m_logger, QJsonDocument(jsonObj).toJson(QJsonDocument::Compact).toStdString());


    int vehicleID = 0;
    bool isVehicleCommand = false;
    // Handle "global" (or non-vehicle specific) commands:
    QJsonDocument tmpDoc = QJsonDocument::fromJson(data[0].toString().toUtf8());
    if (command == "SET_SWARM_ORIGIN")
    {
        setGlobalOrigin(tmpDoc);
    }
    else if (command == "SET_ENVIRONMENT_VERTICES")
    {
        setEnvironmentVertices(tmpDoc);
    }
    else if (command == "GET_CONNECTED_VEHICLES")
    {
        getConnectedVehicles();
    }
    else if (command == "GET_ENVIRONMENT_BOUNDARY")
    {
        getEnvironmentBoundary();
    }
    else if (command == "GET_GLOBAL_ORIGIN")
    {
        getGlobalOrigin();
    }
    else {
        isVehicleCommand = true;
    }

    int counter = 0;
    // Handle vehicle-specific commands:
    if(isVehicleCommand) {
        foreach (const QJsonValue &agent, aircraft)
        {
            vehicleID = agent.toString().toInt();

            QJsonDocument tmpDoc = QJsonDocument::fromJson(data[counter].toString().toUtf8());
            qDebug() << "Agent: " << vehicleID;
            qDebug() << tmpDoc;

            if(issuedCommand(command, vehicleID, tmpDoc))
            {
            }
            else if (command == "SET_VEHICLE_MODE")
            {
                setVehicleMode(vehicleID, tmpDoc);
            }
            else if (command == "SET_VEHICLE_HOME")
            {
                setVehicleHome(vehicleID, tmpDoc);
            }
            else if (command == "SET_VEHICLE_ARM")
            {
                setVehicleArm(vehicleID, tmpDoc);
            }
            else if (command == "SET_GO_HERE")
            {
                setGoHere(vehicleID, tmpDoc);
            }
            else if (command == "GET_VEHICLE_MISSION")
            {
                getVehicleMission(vehicleID);
            }
            else if (command == "GET_VEHICLE_HOME")
            {
                getVehicleHome(vehicleID);
            }            
            else if (command == "TEST_FUNCTION1")
            {
                testFunction1(vehicleID);
            }
            else if (command == "TEST_FUNCTION2")
            {
                testFunction2(vehicleID);
            }
            else {
                std::cout << "Command " << command << " not recognized." << std::endl;
                return;
            }

            counter++;
        }
    }
}

//!
//! \brief writeTCPData Write data to the MACE GUI via TCP
//! \param data Data to be sent to the MACE GUI
//! \return True: success / False: failure
//!
bool GUItoMACE::writeTCPData(QByteArray data)
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
        std::cout << "TCP socket not connected GUI TO MACE" << std::endl;
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
bool GUItoMACE::writeUDPData(QByteArray data)
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
