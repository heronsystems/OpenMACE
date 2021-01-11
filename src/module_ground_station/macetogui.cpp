#include "macetogui.h"


MACEtoGUI::MACEtoGUI() :
    m_sendAddress(QHostAddress::LocalHost),
    m_sendPort(1234),
    m_udpConfig("127.0.0.1", 5678, "127.0.0.1", 8080)
{
    m_udpLink = std::make_shared<CommsMACE::UdpLink>(m_udpConfig);
    m_udpLink->Connect();
}

MACEtoGUI::MACEtoGUI(const QHostAddress &sendAddress, const int &sendPort) :
    m_sendAddress(sendAddress),
    m_sendPort(sendPort),
    m_udpConfig("127.0.0.1", 5678, sendAddress.toString().toStdString(), sendPort)
{
    m_udpLink = std::make_shared<CommsMACE::UdpLink>(m_udpConfig);
    m_udpLink->Connect();
}

MACEtoGUI::~MACEtoGUI() {
    m_logger->flush();
}

//!
//! \brief initiateLogs Start log files and logging for the Ground Station module
//!
void MACEtoGUI::initiateLogs(const std::string &loggerName, const std::string &loggingPath)
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
void MACEtoGUI::setSendAddress(const QHostAddress &sendAddress) {
    m_sendAddress = sendAddress;
}

//!
//! \brief setSendPort Set the TCP send port for MACE-to-GUI comms
//! \param sendPort TCP send port
//!
void MACEtoGUI::setSendPort(const int &sendPort) {
    m_sendPort = sendPort;
}


//!
//! \brief sendCurrentMissionItem Send vehicle mission to the MACE GUI
//! \param vehicleID Vehicle ID with the new vehicle mission
//! \param component Vehicle mission component
//!
void MACEtoGUI::sendCurrentMissionItem(const int &vehicleID, const std::shared_ptr<MissionTopic::MissionItemCurrentTopic> &component) {
    UNUSED(vehicleID);

    QJsonDocument doc(component->toJSON(static_cast<int>(component->getMissionKey().m_systemID),guiMessageString(GuiMessageTypes::CURRENT_MISSION_ITEM)));
//    bool bytesWritten = writeTCPData(doc.toJson());
    bool bytesWritten = writeUDPData(doc.toJson());

    if(!bytesWritten){
        std::cout << "Write current mission item failed..." << std::endl;
    }

    // Log to file:
    logToFile(m_logger, doc.toJson(QJsonDocument::Compact).toStdString());
}

//!
//! \brief sendVehicleTarget Send current vehicle target to the MACE GUI
//! \param vehicleID Vehicle ID with the new vehicle target
//! \param component Vehicle target component
//!
void MACEtoGUI::sendVehicleTarget(const int &vehicleID, const std::shared_ptr<MissionTopic::VehicleTargetTopic> &component)
{
    QJsonObject obj;
    switch (component->m_targetPosition->getCoordinateSystemType()) {
    case CoordinateSystemTypes::GEODETIC:
    {
//        Abstract_GeodeticPosition* targetPosition;
//        targetPosition = dynamic_cast<mace::pose::Abstract_GeodeticPosition*>(component->m_targetPosition);
        obj = component->toJSON(vehicleID, guiMessageString(GuiMessageTypes::VEHICLE_TARGET));

        break;
    }
    case CoordinateSystemTypes::CARTESIAN:
    {
//        mace::pose::GeodeticPosition_3D globalOrigin = this->getDataObject()->GetGlobalOrigin();

//        if(!globalOrigin.isAnyPositionValid())
//            return;

//        mace::pose::GeodeticPosition_2D targetPosition;
//        DynamicsAid::LocalPositionToGlobal(&globalOrigin, dynamic_cast<mace::pose::Abstract_CartesianPosition*>(component->m_targetPosition), &targetPosition);

        break;
    }
    case CoordinateSystemTypes::UNKNOWN:
    case CoordinateSystemTypes::NOT_IMPLIED:
        break;
    }


//    obj["distance_to_target"] = component->m_distanceToTarget;



    QJsonDocument doc(obj);
    //    bool bytesWritten = writeTCPData(doc.toJson());
    bool bytesWritten = writeUDPData(doc.toJson());

    if(!bytesWritten){
        std::cout << "Write vehicle target failed..." << std::endl;
    }

    // Log to file:
    logToFile(m_logger, doc.toJson(QJsonDocument::Compact).toStdString());
}

//!
//! \brief sendVehicleHome Send new vehicle home to the MACE GUI
//! \param vehicleID Vehicle ID with the new vehicle home available
//! \param home New vehicle home
//!
void MACEtoGUI::sendVehicleHome(const int &vehicleID, const command_item::SpatialHome &home)
{
    if(home.getPosition()->getCoordinateSystemType() == CoordinateSystemTypes::GEODETIC)
    {
        QJsonObject obj = home.toJSON(vehicleID, guiMessageString(GuiMessageTypes::VEHICLE_HOME));
        obj["name"] = QString::fromStdString("Agent " + std::to_string(vehicleID) + " Home");
        obj["type"] = "takeoff_land";
        QJsonDocument doc(obj);
        //    bool bytesWritten = writeTCPData(doc.toJson());
        bool bytesWritten = writeUDPData(doc.toJson());
        if(!bytesWritten){
            std::cout << "Write Home position failed..." << std::endl;
        }

        // Log to file:
        logToFile(m_logger, doc.toJson(QJsonDocument::Compact).toStdString());
    }
}

//!
//! \brief sendVehicleParameterList Send the list of vehicle specific parameters
//! \param vehicleID Vehicle ID parameters pertain to
//! \param params Parameter map (Key = string, Value = DataGenericItem::DataGenericItem_ParamValue)
//!
void MACEtoGUI::sendVehicleParameterList(const int &vehicleID, const std::map<std::string, DataGenericItem::DataGenericItem_ParamValue> &params)
{
//    QJsonObject obj = home.toJSON(vehicleID, guiMessageString(GuiMessageTypes::VEHICLE_HOME));
    QJsonObject obj;
    obj["agentID"] = std::to_string(vehicleID).c_str();;
    obj["message_type"] = guiMessageString(GuiMessageTypes::VEHICLE_PARAM_LIST).c_str();

    QJsonArray verticies;
    for(auto&& param : params) {
        QJsonObject obj;
        obj["param_id"] = QString::fromStdString(param.second.getID());
        obj["value"] = param.second.getValue();

        verticies.push_back(obj);
    }

    obj["param_list"] = verticies;

    QJsonDocument doc(obj);
    //    bool bytesWritten = writeTCPData(doc.toJson());
    bool bytesWritten = writeUDPData(doc.toJson());
    if(!bytesWritten){
        std::cout << "Write parameter list failed..." << std::endl;
    }


//    std::cout << "Parameter TKOFF_ALT for Vehicle ID " << vehicleID << ": " << params.at("TKOFF_ALT").getValue() << std::endl;


    // Write out generic message to display on GUI:
    QJsonObject textJson;
    textJson["message_type"] = guiMessageString(GuiMessageTypes::VEHICLE_TEXT).c_str();
    textJson["agentID"] = std::to_string(vehicleID).c_str();;
    textJson["severity"] =  QString::fromStdString(DataGenericItem::DataGenericItem_Text::StatusSeverityToString(MAV_SEVERITY::MAV_SEVERITY_INFO));
    std::string text = "TKOFF_ALT param set to " /*+ std::to_string(params.at("TKOFF_ALT").getValue())*/;
    textJson["text"] = QString::fromStdString(text);
    QJsonDocument textDoc(textJson);
    //    bool bytesWritten = writeTCPData(doc.toJson());
    bool textBytesWritten = writeUDPData(textDoc.toJson());
    if(!textBytesWritten){
        std::cout << "Write parameter list TEXT failed..." << std::endl;
    }
}

//!
//! \brief sendGlobalOrigin Send new global origin to the MACE GUI
//! \param origin New global origin
//!
void MACEtoGUI::sendGlobalOrigin(const command_item::SpatialHome &origin)
{
    if(origin.getPosition()->getCoordinateSystemType() == CoordinateSystemTypes::GEODETIC)
    {
        QJsonObject obj = origin.toJSON(0, guiMessageString(GuiMessageTypes::GLOBAL_ORIGIN));
        obj["name"] = "Swarm origin";
        obj["type"] = "origin";
        QJsonDocument doc(obj);
        //    bool bytesWritten = writeTCPData(doc.toJson());
        bool bytesWritten = writeUDPData(doc.toJson());

        if(!bytesWritten){
            std::cout << "Write global origin failed..." << std::endl;
        }

        // Log to file:
        logToFile(m_logger, doc.toJson(QJsonDocument::Compact).toStdString());
    }
}

//!
//! \brief sendPositionData Send vehicle position data to the MACE GUI
//! \param vehicleID Vehicle ID with new position update
//! \param component Global position component
//!
void MACEtoGUI::sendPositionData(const int &vehicleID, const std::shared_ptr<mace::pose_topics::Topic_GeodeticPosition> &component)
{
    if(component->getPositionObj()->getCoordinateSystemType() == CoordinateSystemTypes::GEODETIC)
    {

        QJsonDocument doc(component->toJSON(vehicleID,guiMessageString(GuiMessageTypes::VEHICLE_POSITION)));
        //    bool bytesWritten = writeTCPData(doc.toJson());
        bool bytesWritten = writeUDPData(doc.toJson());

        if(!bytesWritten){
            std::cout << "Write Position Data failed..." << std::endl;
        }

        // Log to file:
        logToFile(m_logger, doc.toJson(QJsonDocument::Compact).toStdString());
    }
}

//!
//! \brief sendAttitudeData Send vehicle attitude data to the MACE GUI
//! \param vehicleID Vehicle ID with new attitude update
//! \param component Vehicle attitude component
//!
void MACEtoGUI::sendAttitudeData(const int &vehicleID, const std::shared_ptr<mace::pose_topics::Topic_AgentOrientation> &component)
{
    QJsonDocument doc(component->toJSON(vehicleID,guiMessageString(GuiMessageTypes::VEHICLE_ATTITUDE)));

    //    bool bytesWritten = writeTCPData(doc.toJson());
    bool bytesWritten = writeUDPData(doc.toJson());

    if(!bytesWritten){
        std::cout << "Write Attitude Data failed..." << std::endl;
    }

    // Log to file:
    logToFile(m_logger, doc.toJson(QJsonDocument::Compact).toStdString());
}

//!
//! \brief sendVehicleAirspeed Send vehicle airspeed to the MACE GUI
//! \param vehicleID Vehicle ID with the new airspeed
//! \param component Vehicle airspeed component
//!
void MACEtoGUI::sendVehicleAirspeed(const int &vehicleID, const mace::measurement_topics::Topic_AirSpeedPtr &component)
{
        QJsonDocument doc(component->toJSON(vehicleID,guiMessageString(GuiMessageTypes::VEHICLE_AIRSPEED)));
        //    bool bytesWritten = writeTCPData(doc.toJson());
        bool bytesWritten = writeUDPData(doc.toJson());

        if(!bytesWritten){
            std::cout << "Write vehicle airspeed Data failed..." << std::endl;
        }

        // Log to file:
        logToFile(m_logger, doc.toJson(QJsonDocument::Compact).toStdString());
}

//!
//! \brief sendVehicleFuel Send vehicle fuel data to the MACE GUI
//! \param vehicleID Vehicle ID with the new fuel update
//! \param component Vehicle battery component
//!
void MACEtoGUI::sendVehicleFuel(const int &vehicleID, const std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_Battery> &component)
{

    QJsonDocument doc(component->toJSON(vehicleID,guiMessageString(GuiMessageTypes::VEHICLE_FUEL)));
    //    bool bytesWritten = writeTCPData(doc.toJson());
    bool bytesWritten = writeUDPData(doc.toJson());

    if(!bytesWritten){
        std::cout << "Write Fuel Data failed..." << std::endl;
    }

    // Log to file:
    logToFile(m_logger, doc.toJson(QJsonDocument::Compact).toStdString());
}

//!
//! \brief sendVehicleMode Send vehilce mode data to the MACE GUI
//! \param vehicleID Vehicle ID with the new flight mode update
//! \param component Vehicle flight mode component
//!
void MACEtoGUI::sendVehicleMode(const int &vehicleID, const std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_FlightMode> &component)
{
    QJsonDocument doc(component->toJSON(vehicleID,guiMessageString(GuiMessageTypes::VEHICLE_MODE)));
    //    bool bytesWritten = writeTCPData(doc.toJson());
    bool bytesWritten = writeUDPData(doc.toJson());

    if(!bytesWritten){
        std::cout << "Write Vehicle Mode Data failed..." << std::endl;
    }

    // Log to file:
    logToFile(m_logger, doc.toJson(QJsonDocument::Compact).toStdString());
}

//!
//! \brief sendVehicleText Send vehicle message data to the MACE GUI
//! \param vehicleID Vehicle ID with the new message
//! \param component Vehicle text component
//!
void MACEtoGUI::sendVehicleText(const int &vehicleID, const std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_Text> &component)
{
    QJsonDocument doc(component->toJSON(vehicleID, guiMessageString(GuiMessageTypes::VEHICLE_TEXT)));
    //    bool bytesWritten = writeTCPData(doc.toJson());
    bool bytesWritten = writeUDPData(doc.toJson());

    if(!bytesWritten){
        std::cout << "Write Vehicle Text Data failed..." << std::endl;
    }

    // Log to file:
    logToFile(m_logger, doc.toJson(QJsonDocument::Compact).toStdString());
}

//!
//! \brief sendVehicleGPS Send vehicle GPS status to the MACE GUI
//! \param vehicleID Vehicle ID with the new GPS status
//! \param component GPS status component
//!
void MACEtoGUI::sendVehicleGPS(const int &vehicleID, const std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_GPS> &component)
{
    QJsonDocument doc(component->toJSON(vehicleID,guiMessageString(GuiMessageTypes::VEHICLE_GPS)));
    //    bool bytesWritten = writeTCPData(doc.toJson());
    bool bytesWritten = writeUDPData(doc.toJson());

    if(!bytesWritten){
        std::cout << "Write Vehicle GPS Data failed..." << std::endl;
    }

    // Log to file:
    logToFile(m_logger, doc.toJson(QJsonDocument::Compact).toStdString());
}

//!
//! \brief sendVehicleArm Send vehicle arm status to the MACE GUI
//! \param vehicleID Vehicle ID with the new ARM status
//! \param component Vehicle Arm component
//!
void MACEtoGUI::sendVehicleArm(const int &vehicleID, const std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_SystemArm> &component)
{
    QJsonDocument doc(component->toJSON(vehicleID,guiMessageString(GuiMessageTypes::VEHICLE_ARM)));
    //    bool bytesWritten = writeTCPData(doc.toJson());
    bool bytesWritten = writeUDPData(doc.toJson());

    if(!bytesWritten){
        std::cout << "Write vehicle arm failed..." << std::endl;
    }

    // Log to file:
    logToFile(m_logger, doc.toJson(QJsonDocument::Compact).toStdString());
}

//!
//! \brief sendMissionItemReached Send mission item reached topic to the MACE GUI
//! \param vehicleID Vehicle ID corresponding to the vehicle who reached a mission item
//! \param component Mission item reached component
//!
void MACEtoGUI::sendMissionItemReached(const int &vehicleID, const std::shared_ptr<MissionTopic::MissionItemReachedTopic> &component)
{
    QJsonDocument doc(component->toJSON(vehicleID,guiMessageString(GuiMessageTypes::MISSION_ITEM_REACHED)));
    //    bool bytesWritten = writeTCPData(doc.toJson());
    bool bytesWritten = writeUDPData(doc.toJson());

    if(!bytesWritten){
        std::cout << "Write mission item reached failed..." << std::endl;
    }

    // Log to file:
    logToFile(m_logger, doc.toJson(QJsonDocument::Compact).toStdString());
}

//!
//! \brief sendVehicleHeartbeat Send vehicle heartbeat to the MACE GUI
//! \param vehicleID Vehicle ID with the new vehicle heartbeat
//! \param component Vehicle heartbeat component
//!
void MACEtoGUI::sendVehicleHeartbeat(const int &vehicleID, const std::shared_ptr<DataGenericItem::DataGenericItem_Heartbeat> &component)
{
    QJsonDocument doc(component->toJSON(vehicleID,guiMessageString(GuiMessageTypes::VEHICLE_HEARTBEAT)));
    //    bool bytesWritten = writeTCPData(doc.toJson());
    bool bytesWritten = writeUDPData(doc.toJson());

    if(!bytesWritten){
        std::cout << "Write vehicle heartbeat failed..." << std::endl;
    }

    // Log to file:
    logToFile(m_logger, doc.toJson(QJsonDocument::Compact).toStdString());
}

//!
//! \brief sendVehicleMission Send vehicle mission data to the MACE GUI
//! \param vehicleID Vehicle ID with the new mission available
//! \param missionList Mission list component
//!
void MACEtoGUI::sendVehicleMission(const int &vehicleID, const MissionItem::MissionList &missionList)
{
    QJsonObject jsonMission = missionList.toJSON(vehicleID,guiMessageString(GuiMessageTypes::VEHICLE_MISSION));
    QJsonObject jsonPath = missionList.toJSON(vehicleID,guiMessageString(GuiMessageTypes::VEHICLE_PATH));
    QJsonArray missionItems;
    QJsonArray path;
    missionListToJSON(missionList, missionItems, path);
    jsonMission["missionItems"] = missionItems;
    jsonPath["vertices"] = path;
    QJsonDocument docMission(jsonMission);
    QJsonDocument docPath(jsonPath);
//    bool bytesWritten = writeTCPData(docMission.toJson());
    bool bytesWritten = writeUDPData(docMission.toJson());

    if(!bytesWritten){
        std::cout << "Write Vehicle Mission Data failed..." << std::endl;
    }

    // Log to file:
    logToFile(m_logger, docMission.toJson(QJsonDocument::Compact).toStdString());

//    bytesWritten = writeTCPData(docPath.toJson());
    bytesWritten = writeUDPData(docPath.toJson());

    if(!bytesWritten){
        std::cout << "Write Vehicle Path Data failed..." << std::endl;
    }

    // Log to file:
    logToFile(m_logger, docPath.toJson(QJsonDocument::Compact).toStdString());
}

void MACEtoGUI::sendVehiclePath(const int &vehicleID, const VehiclePath_Linear &waypointList)
{
    //    QJsonObject jsonPath = waypointList.toJSON(vehicleID,guiMessageString(GuiMessageTypes::VEHICLE_PATH));
    QJsonObject jsonPath;
    jsonPath["message_type"] = guiMessageString(GuiMessageTypes::VEHICLE_PATH).c_str();
    jsonPath["agentID"] = std::to_string(vehicleID).c_str();;
    QJsonArray path;
    waypointListToJSON(waypointList.getVertices(), path);
    jsonPath["vertices"] = path;
    QJsonDocument docPath(jsonPath);
    bool bytesWritten = writeUDPData(docPath.toJson());
    if(!bytesWritten){
        std::cout << "Write Vehicle Path Data failed..." << std::endl;
    }
    // Log to file:
    logToFile(m_logger, docPath.toJson(QJsonDocument::Compact).toStdString());
}

//!
//! \brief sendSensorFootprint Send vehicle sensor footprint to the MACE GUI
//! \param vehicleID Vehicle ID with the new sensor footprint available
//! \param component Vehicle sensor footprint component
//!
void MACEtoGUI::sendSensorFootprint(const int &vehicleID, const std::shared_ptr<DataVehicleSensors::SensorVertices_Global> &component) {
   
    QJsonDocument doc(component->toJSON(vehicleID,guiMessageString(GuiMessageTypes::SENSOR_FOOTPRINT)));
    //    bool bytesWritten = writeTCPData(doc.toJson());
    bool bytesWritten = writeUDPData(doc.toJson());

    if(!bytesWritten){
        std::cout << "Write vehicle sensor footprint failed..." << std::endl;
    }

    // Log to file:
    logToFile(m_logger, doc.toJson(QJsonDocument::Compact).toStdString());
}

//!
//! \brief sendEnvironmentVertices Send environment boundary vertices to the MACE GUI
//! \param component Environment boundary component
//!
void MACEtoGUI::sendEnvironmentVertices(const std::vector<mace::pose::GeodeticPosition_3D> &component) {

    QJsonObject json;
    json["message_type"] = QString::fromStdString(guiMessageString(GuiMessageTypes::ENVIRONMENT_BOUNDARY));
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
    //    bool bytesWritten = writeTCPData(doc.toJson());
    bool bytesWritten = writeUDPData(doc.toJson());

    if(!bytesWritten){
        std::cout << "Write environment boundary failed..." << std::endl;
    }

    // Log to file:
    logToFile(m_logger, doc.toJson(QJsonDocument::Compact).toStdString());
}

void MACEtoGUI::waypointListToJSON(const std::vector<mace::pose::GeodeticPosition_3D> &waypointList, QJsonArray &path)
{
    for(size_t i = 0; i < waypointList.size(); i++)
    {
        QJsonObject obj;
        obj["lat"] = waypointList[i].getLatitude();
        obj["lng"] = waypointList[i].getLongitude();
        if(waypointList[i].is3D()) {
            obj["alt"] = waypointList[i].getAltitude();
        }
        path.push_back(obj);
    }
}

void MACEtoGUI::missionListToJSON(const MissionItem::MissionList &list, QJsonArray &missionItems,QJsonArray &path)
{
    for(size_t i = 0; i < list.getQueueSize(); i++)
    {
        //TODO-PAT: Look into unique_ptr or auto_ptr?? Not sure I like this...
        std::shared_ptr<command_item::AbstractCommandItem> missionItem = list.getMissionItem(i);

        QJsonObject obj;
        obj["description"] = QString::fromStdString(missionItem->getDescription());
        obj["type"] = QString::fromStdString(command_item::CommandItemToString(missionItem->getCommandType()));

        switch (missionItem->getCommandType()) {
        case MAV_CMD::MAV_CMD_COMPONENT_ARM_DISARM:
        {
            std::shared_ptr<command_item::ActionArm> castItem = std::dynamic_pointer_cast<command_item::ActionArm>(missionItem);
            obj["positionalFrame"] = "global";
            UNUSED(castItem);
            break;
        }
        case MAV_CMD::MAV_CMD_DO_SET_MODE:
        {
            std::shared_ptr<command_item::ActionChangeMode> castItem = std::dynamic_pointer_cast<command_item::ActionChangeMode>(missionItem);
            obj["positionalFrame"] = "global";
            UNUSED(castItem);
            break;
        }
        case MAV_CMD::MAV_CMD_NAV_LAND:
        {
            std::shared_ptr<command_item::SpatialLand> castItem = std::dynamic_pointer_cast<command_item::SpatialLand>(missionItem);
            obj["positionalFrame"] = "global";
            UNUSED(castItem);

            break;
        }
        case MAV_CMD::MAV_CMD_NAV_RETURN_TO_LAUNCH:
        {
            std::shared_ptr<command_item::SpatialRTL> castItem = std::dynamic_pointer_cast<command_item::SpatialRTL>(missionItem);
            obj["positionalFrame"] = "global";
            UNUSED(castItem);
            break;
        }
        case MAV_CMD::MAV_CMD_NAV_TAKEOFF:
        {
            std::shared_ptr<command_item::SpatialTakeoff> castItem = std::dynamic_pointer_cast<command_item::SpatialTakeoff>(missionItem);
            if(castItem->position->getCoordinateSystemType() == CoordinateSystemTypes::GEODETIC)
            {
                obj["positionalFrame"] = "global";
                if(castItem->position->is3D())
                {
                    const mace::pose::GeodeticPosition_3D* castPosition = castItem->position->positionAs<mace::pose::GeodeticPosition_3D>();
                    obj["lat"] = castPosition->getLatitude();
                    obj["lng"] = castPosition->getLongitude();
                    obj["alt"] = castPosition->getAltitude();
                }
            }

            break;
        }
        case MAV_CMD::MAV_CMD_NAV_WAYPOINT:
        {
            std::shared_ptr<command_item::SpatialWaypoint> castItem = std::dynamic_pointer_cast<command_item::SpatialWaypoint>(missionItem);
            obj["positionalFrame"] = "global";
            castItem->getPosition()->updateQJSONObject(obj);

            QJsonObject waypoint;
            castItem->getPosition()->updateQJSONObject(waypoint);
            path.push_back(waypoint);
            break;
        }
        case MAV_CMD::MAV_CMD_NAV_LOITER_TIME:
        {
            std::shared_ptr<command_item::SpatialLoiter_Time> castItem = std::dynamic_pointer_cast<command_item::SpatialLoiter_Time>(missionItem);
            obj["positionalFrame"] = "global";
            //            obj["lat"] = castItem->position->getX();
            //            obj["lng"] = castItem->position->getY();
            //            obj["alt"] = castItem->position->getZ();
            obj["duration"] = castItem->duration;
            if(castItem->direction == Data::LoiterDirection::CW)
            {
                obj["radius"] = castItem->radius;
            }else{
                obj["radius"] = 0-castItem->radius;
            }
            break;
        }
        case MAV_CMD::MAV_CMD_NAV_LOITER_TURNS:
        {
            std::shared_ptr<command_item::SpatialLoiter_Turns> castItem = std::dynamic_pointer_cast<command_item::SpatialLoiter_Turns>(missionItem);
            obj["positionalFrame"] = "global";
            //            obj["lat"] = castItem->position->getX();
            //            obj["lng"] = castItem->position->getY();
            //            obj["alt"] = castItem->position->getZ();
            obj["turns"] = static_cast<int>(castItem->turns);
            if(castItem->direction == Data::LoiterDirection::CW)
            {
                obj["radius"] = castItem->radius;
            }else{
                obj["radius"] = 0-castItem->radius;
            }
            break;
        }
        case MAV_CMD::MAV_CMD_NAV_LOITER_UNLIM:
        {
            std::shared_ptr<command_item::SpatialLoiter_Unlimited> castItem = std::dynamic_pointer_cast<command_item::SpatialLoiter_Unlimited>(missionItem);
            obj["positionalFrame"] = "global";
            //            obj["lat"] = castItem->position->getX();
            //            obj["lng"] = castItem->position->getY();
            //            obj["alt"] = castItem->position->getZ();
            if(castItem->direction == Data::LoiterDirection::CW)
            {
                obj["radius"] = castItem->radius;
            }else{
                obj["radius"] = 0-castItem->radius;
            }
            break;
        }
        default:
            break;
        }

        missionItems.push_back(obj);
    }
}

//!
//! \brief writeTCPData Write data to the MACE GUI via TCP
//! \param data Data to be sent to the MACE GUI
//! \return True: success / False: failure
//!
bool MACEtoGUI::writeTCPData(QByteArray data)
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
        std::cout << "TCP socket not connected MACE TO GUI" << std::endl;
        tcpSocket->close();
        return false;
    }
}

//!
//! \brief writeUDPData Write data to the MACE GUI via UDP
//! \param data Data to be sent to the MACE GUI
//! \return True: success / False: failure
//!
bool MACEtoGUI::writeUDPData(QByteArray data)
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
