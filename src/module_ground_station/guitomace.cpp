#include "guitomace.h"

GUItoMACE::GUItoMACE(const MaceCore::IModuleCommandGroundStation* ptrRef) :
    m_sendAddress(QHostAddress::LocalHost),
    m_sendPort(1234)
{
    m_parent = ptrRef;
}

GUItoMACE::GUItoMACE(const MaceCore::IModuleCommandGroundStation* ptrRef, const QHostAddress &sendAddress, const int &sendPort) :
    m_sendAddress(sendAddress),
    m_sendPort(sendPort)
{
    m_parent = ptrRef;
}

GUItoMACE::~GUItoMACE() {
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
    std::vector<mace::pose::GeodeticPosition_2D> environmentVertices = data->GetEnvironmentBoundary();

    QJsonObject json;
    json["dataType"] = "EnvironmentBoundary";
    json["vehicleID"] = 0;

    QJsonArray verticies;
    for(auto&& vertex : environmentVertices) {
        QJsonObject obj;
        obj["lat"] = vertex.getLatitude();
        obj["lng"] = vertex.getLongitude();
        obj["alt"] = 0.0;

        verticies.push_back(obj);
    }

    json["environmentBoundary"] = verticies;

    QJsonDocument doc(json);
    bool bytesWritten = writeTCPData(doc.toJson());

    if(!bytesWritten){
        std::cout << "Write environment boundary failed..." << std::endl;
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
    bool bytesWritten = writeTCPData(doc.toJson());

    if(!bytesWritten){
        std::cout << "Write global origin failed..." << std::endl;
    }
}

//!
//! \brief setVehicleHome GUI command to set a new vehicle home position
//! \param vehicleID Vehicle ID that the command is initated from (0 = all vehicles)
//! \param jsonObj JSON data containing the new vehicle home data
//!
void GUItoMACE::setVehicleHome(const int &vehicleID, const QJsonObject &jsonObj)
{
    command_item::SpatialHome tmpHome;
    tmpHome.setTargetSystem(vehicleID);
    QJsonObject position = QJsonDocument::fromJson(jsonObj["vehicleCommand"].toString().toUtf8()).object();

    mace::pose::GeodeticPosition_3D homePosition(position.value("lat").toDouble(),
                                                 position.value("lng").toDouble(),
                                                 position.value("alt").toDouble());
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
void GUItoMACE::setGlobalOrigin(const QJsonObject &jsonObj)
{
    QJsonObject position = QJsonDocument::fromJson(jsonObj["vehicleCommand"].toString().toUtf8()).object();

    mace::pose::GeodeticPosition_3D origin;
    origin.setLatitude(position.value("lat").toDouble());
    origin.setLongitude(position.value("lng").toDouble());
    origin.setAltitude(position.value("alt").toDouble());

    m_parent->NotifyListeners([&](MaceCore::IModuleEventsGroundStation* ptr) {
        ptr->Event_SetGlobalOrigin(this, origin);
    });
}

//!
//! \brief setEnvironmentVertices GUI command to set new environment boundary vertices
//! \param jsonObj JSON data containing the new environment vertices
//!
void GUItoMACE::setEnvironmentVertices(const QJsonObject &jsonObj)
{
    BoundaryItem::BoundaryList operationalBoundary;

    mace::pose::GeodeticPosition_3D origin = m_parent->getDataObject()->GetGlobalOrigin();

    QJsonObject tmpBoundaryObj = QJsonDocument::fromJson(jsonObj["vehicleCommand"].toString().toUtf8()).object();
    QJsonArray boundary = tmpBoundaryObj.value("boundary").toArray();

    foreach(const QJsonValue & v, boundary) {
        std::cout << "Lat: " << v.toObject().value("lat").toDouble() << " / Lon: " << v.toObject().value("lng").toDouble() << std::endl;
        double tmpLat = v.toObject().value("lat").toDouble();
        double tmpLon = v.toObject().value("lng").toDouble();
        double tmpAlt = v.toObject().value("alt").toDouble();

        mace::pose::GeodeticPosition_3D vertexGlobal(tmpLat,tmpLon,tmpAlt);
        mace::pose::CartesianPosition_3D vertexLocal3D;

        if(origin.isAnyPositionValid()) {
            mace::pose::DynamicsAid::GlobalPositionToLocal(&origin,&vertexGlobal,&vertexLocal3D);
            mace::pose::CartesianPosition_2D vertexLocal2D(vertexLocal3D.getXPosition(),vertexLocal3D.getYPosition());

            operationalBoundary.appendVertexItem(&vertexLocal2D);
        }
    }

    BoundaryItem::BoundaryCharacterisic key(BoundaryItem::BOUNDARYTYPE::OPERATIONAL_FENCE);

    if(operationalBoundary.getQueueSize() > 0) {
        m_parent->NotifyListeners([&](MaceCore::IModuleEventsGroundStation* ptr) {
            ptr->Event_SetBoundary(m_parent, key, operationalBoundary);
        });
    }

    // Get and send vertices to the GUI:
    getEnvironmentBoundary();
}

//!
//! \brief setGoHere GUI command to set a new "go here" lat/lon/alt position
//! \param vehicleID Vehicle ID that the command is initated from (0 = all vehicles)
//! \param jsonObj JSON data containing the "go here" position
//!
void GUItoMACE::setGoHere(const int &vehicleID, const QJsonObject &jsonObj)
{
    // TODO:
    std::cout << "Go here command issued" << std::endl;

    //Ken Fix: Target system should propogate or not exist at the mission item level using action/command logic
    command_item::Action_ExecuteSpatialItem cmdGoTo;
    cmdGoTo.setTargetSystem(vehicleID);

    command_item::SpatialWaypointPtr spatialAction = std::make_shared<command_item::SpatialWaypoint>(vehicleID);
//    spatialAction->getPosition().setCoordinateFrame(Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT);

    QJsonObject vehicleCommand = QJsonDocument::fromJson(jsonObj["vehicleCommand"].toString().toUtf8()).object();

    mace::pose::GeodeticPosition_3D goPosition(vehicleCommand.value("lat").toDouble(),
                                               vehicleCommand.value("lon").toDouble(),
                                               10.0);
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
void GUItoMACE::takeoff(const int &vehicleID, const QJsonObject &jsonObj)
{
    command_item::SpatialTakeoff newTakeoff;
    QJsonObject vehicleCommand = QJsonDocument::fromJson(jsonObj["vehicleCommand"].toString().toUtf8()).object();
    QJsonObject position = vehicleCommand["takeoffPosition"].toObject();
    bool latLonFlag = vehicleCommand["latLonFlag"].toBool();

    mace::pose::GeodeticPosition_3D takeoffPosition;

    if(latLonFlag) {
        takeoffPosition.setLatitude(position.value("lat").toDouble());
        takeoffPosition.setLongitude(position.value("lon").toDouble());
    }
    takeoffPosition.setAltitude(position.value("alt").toDouble());
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
void GUItoMACE::issueCommand(const int &vehicleID, const QJsonObject &jsonObj)
{    
    if(jsonObj["vehicleCommand"] == "FORCE_DATA_SYNC") {
//        mLogs->debug("Module Ground Station issuing command force data sync to system " + std::to_string(vehicleID) + ".");
        m_parent->NotifyListeners([&](MaceCore::IModuleEventsGroundStation* ptr){
            ptr->Event_ForceVehicleDataSync(m_parent, vehicleID);
        });
    }
    else if(jsonObj["vehicleCommand"] == "RTL") {
//        mLogs->debug("Module Ground Station issuing command RTL to system " + std::to_string(vehicleID) + ".");
        command_item::SpatialRTL rtlCommand;
        rtlCommand.setTargetSystem(vehicleID);
        // TODO: Set generating system and coordinate frame

        m_parent->NotifyListeners([&](MaceCore::IModuleEventsGroundStation* ptr){
            ptr->Event_IssueCommandRTL(m_parent, rtlCommand);
        });
    }
    else if(jsonObj["vehicleCommand"] == "LAND") {
//        mLogs->debug("Module Ground Station issuing land command to system " + std::to_string(vehicleID) + ".");
        command_item::SpatialLand landCommand;
        landCommand.setTargetSystem(vehicleID);
        // TODO: Set generating system and coordinate frame

        m_parent->NotifyListeners([&](MaceCore::IModuleEventsGroundStation* ptr){
            ptr->Event_IssueCommandLand(m_parent, landCommand);
        });
    }
    else if(jsonObj["vehicleCommand"] == "AUTO_START") {
//        mLogs->debug("Module Ground Station issuing mission start command to system " + std::to_string(vehicleID) + ".");
        command_item::ActionMissionCommand missionCommand;
        missionCommand.setMissionStart();
        missionCommand.setTargetSystem(vehicleID);
        // TODO: Set generating system and coordinate frame

        m_parent->NotifyListeners([&](MaceCore::IModuleEventsGroundStation* ptr){
            ptr->Event_IssueMissionCommand(m_parent, missionCommand);
        });
    }
    else if(jsonObj["vehicleCommand"] == "AUTO_PAUSE") {
//        mLogs->debug("Module Ground Station issuing mission pause command to system " + std::to_string(vehicleID) + ".");
        command_item::ActionMissionCommand missionCommand;
        missionCommand.setMissionPause();
        missionCommand.setTargetSystem(vehicleID);
        // TODO: Set generating system and coordinate frame

        m_parent->NotifyListeners([&](MaceCore::IModuleEventsGroundStation* ptr){
            ptr->Event_IssueMissionCommand(m_parent, missionCommand);
        });
    }
    else if(jsonObj["vehicleCommand"] == "AUTO_RESUME") {
//        mLogs->debug("Module Ground Station issuing mission resume command to system " + std::to_string(vehicleID) + ".");
        command_item::ActionMissionCommand missionCommand;
        missionCommand.setMissionResume();
        missionCommand.setTargetSystem(vehicleID);
        // TODO: Set generating system and coordinate frame

        m_parent->NotifyListeners([&](MaceCore::IModuleEventsGroundStation* ptr){
            ptr->Event_IssueMissionCommand(m_parent, missionCommand);
        });
    }

}


void GUItoMACE::testFunction1(const int &vehicleID)
{

    command_item::Action_DynamicTarget newCommand;
    newCommand.setTargetSystem(vehicleID);
    newCommand.setOriginatingSystem(255);
    command_target::DynamicTarget newTarget;
    mace::pose::Cartesian_Velocity3D currentVelocityTarget(CartesianFrameTypes::CF_LOCAL_NED);
    currentVelocityTarget.setXVelocity(5.0);
    currentVelocityTarget.setYVelocity(0.0);
    currentVelocityTarget.setZVelocity(0.0);
    newTarget.setVelocity(&currentVelocityTarget);

    mace::pose::Rotation_2D yaw;
    yaw.setPhi(M_PI_4);
    newTarget.setYaw(&yaw);

    newCommand.setDynamicTarget(newTarget);
    m_parent->NotifyListeners([&](MaceCore::IModuleEventsGroundStation* ptr){
        ptr->EventPP_ExecuteDynamicTarget(m_parent, newCommand);
    });

}

void GUItoMACE::testFunction2(const int &vehicleID)
{
    command_item::Action_DynamicTarget newCommand;
    newCommand.setTargetSystem(vehicleID);
    newCommand.setOriginatingSystem(255);
    command_target::DynamicTarget newTarget;

    mace::pose::GeodeticPosition_3D newPosition(-35.3616686, 149.1638553, 15);
    newPosition.setCoordinateFrame(GeodeticFrameTypes::CF_GLOBAL_RELATIVE_ALT);
    newPosition.setAltitudeReferenceFrame(AltitudeReferenceTypes::REF_ALT_RELATIVE);
    newTarget.setPosition(&newPosition);

    mace::pose::Cartesian_Velocity3D currentVelocityTarget(CartesianFrameTypes::CF_LOCAL_NED);
    currentVelocityTarget.setXVelocity(-2.0);
    currentVelocityTarget.setYVelocity(-2.0);
    currentVelocityTarget.setZVelocity(1.0);
    newTarget.setVelocity(&currentVelocityTarget);

    newCommand.setDynamicTarget(newTarget);
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

    std::shared_ptr<const MaceCore::MaceData> data = m_parent->getDataObject();
    std::vector<unsigned int> vehicleIDs;
    data->GetAvailableVehicles(vehicleIDs);           

    QJsonArray ids;
    QJsonArray modes;
    if(vehicleIDs.size() > 0){
        for (const int& i : vehicleIDs) {
            ids.append(i);
            std::string mode;
            data->GetVehicleFlightMode(i, mode);
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
    bool bytesWritten = writeTCPData(doc.toJson());

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
void GUItoMACE::setVehicleArm(const int &vehicleID, const QJsonObject &jsonObj)
{
    command_item::ActionArm tmpArm;
    tmpArm.setTargetSystem(vehicleID); // the vehicle ID coordinates to the specific vehicle //vehicle 0 is reserved for all connected vehicles

    QJsonObject arm = QJsonDocument::fromJson(jsonObj["vehicleCommand"].toString().toUtf8()).object();
    tmpArm.setVehicleArm(arm.value("arm").toBool());

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
void GUItoMACE::setVehicleMode(const int &vehicleID, const QJsonObject &jsonObj)
{
    command_item::ActionChangeMode tmpMode;
    tmpMode.setTargetSystem(vehicleID); // the vehicle ID coordinates to the specific vehicle //vehicle 0 is reserved for all connected vehicles
    tmpMode.setRequestMode(jsonObj["vehicleCommand"].toString().toStdString()); //where the string here is the desired Flight Mode...available modes can be found in the appropriate topic
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
    QString command = jsonObj["tcpCommand"].toString();
    int vehicleID = jsonObj["vehicleID"].toInt();

    QByteArray data;
    if(command == "SET_VEHICLE_MODE")
    {
        setVehicleMode(vehicleID, jsonObj);
    }
    else if(command == "ISSUE_COMMAND")
    {
        issueCommand(vehicleID, jsonObj);
    }
    else if(command == "SET_VEHICLE_HOME")
    {
        setVehicleHome(vehicleID, jsonObj);
    }
    else if(command == "SET_GLOBAL_ORIGIN")
    {
        setGlobalOrigin(jsonObj);
    }
    else if(command == "SET_ENVIRONMENT_VERTICES")
    {
        setEnvironmentVertices(jsonObj);
    }
    else if(command == "SET_VEHICLE_ARM")
    {
        setVehicleArm(vehicleID, jsonObj);
    }
    else if(command == "SET_GO_HERE")
    {
        setGoHere(vehicleID, jsonObj);
    }
    else if(command == "GET_VEHICLE_MISSION")
    {
        getVehicleMission(vehicleID);
    }
    else if(command == "GET_CONNECTED_VEHICLES")
    {
        getConnectedVehicles();
    }
    else if(command == "GET_VEHICLE_HOME")
    {
        getVehicleHome(vehicleID);
    }
    else if(command == "TEST_FUNCTION1")
    {
        testFunction1(vehicleID);
    }
    else if(command == "TEST_FUNCTION2")
    {
        testFunction2(vehicleID);
    }
    else if(command == "VEHICLE_TAKEOFF")
    {
        takeoff(vehicleID, jsonObj);
    }
    else if(command == "GET_ENVIRONMENT_BOUNDARY")
    {
        getEnvironmentBoundary();
    }
    else if(command == "GET_GLOBAL_ORIGIN")
    {
        getGlobalOrigin();
    }
    else
    {
        std::cout << "Command " << command.toStdString() << " not recognized." << std::endl;
        data = "command_not_recognized";
        return;
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
