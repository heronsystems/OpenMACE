#include "module_external_link.h"

void ModuleExternalLink::ParseForData(const mavlink_message_t* message){
    MaceCore::TopicDatagram topicDatagram;

    MaceCore::ModuleCharacteristic sender;
    sender.MaceInstance = message->sysid;
    sender.ModuleID = message->compid;

    switch (message->msgid) {
    case MAVLINK_MSG_ID_MACE_HEARTBEAT:
    {
        mavlink_mace_heartbeat_t decodedMSG;
        mavlink_msg_mace_heartbeat_decode(message,&decodedMSG);
        HeartbeatInfo(sender, decodedMSG);
        break;
    }
    case MAVLINK_MSG_ID_VEHICLE_SYNC:
    {
        mavlink_vehicle_sync_t decodedMSG;
        mavlink_msg_vehicle_sync_decode(message,&decodedMSG);
//        if(mLog)
//            mLog->debug("External link saw a request to sync its data to a remote instance.");

        MaceCore::ModuleCharacteristic module = this->getDataObject()->GetVehicleFromMAVLINKID(decodedMSG.target_system);

        //We may not handle it this way anymore
        ModuleExternalLink::NotifyListeners([&](MaceCore::IModuleEventsExternalLink* ptr){
            ptr->ExternalEvent_RequestingDataSync(this, module);
        });
        break;
    }
//    case MAVLINK_MSG_ID_VEHICLE_ARMED:
//    {
//        mavlink_vehicle_armed_t decodedMSG;
//        mavlink_msg_vehicle_armed_decode(message,&decodedMSG);
//        DataGenericItem::DataGenericItem_SystemArm newItem(decodedMSG);
//        std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_SystemArm> ptrArm = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_SystemArm>(newItem);
//        PublishVehicleData(sender, ptrArm);
//        break;
//    }
//    case MAVLINK_MSG_ID_VEHICLE_MODE:
//    {
//        mavlink_vehicle_mode_t decodedMSG;
//        mavlink_msg_vehicle_mode_decode(message,&decodedMSG);
//        DataGenericItem::DataGenericItem_FlightMode newItem(decodedMSG);
//        std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_FlightMode> ptrMode = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_FlightMode>(newItem);
//        PublishVehicleData(sender, ptrMode);
//        break;
//    }
    case MAVLINK_MSG_ID_BATTERY_STATUS:
    {
        mavlink_battery_status_t decodedMSG;
        mavlink_msg_battery_status_decode(message,&decodedMSG);
        DataGenericItem::DataGenericItem_Battery newItem(decodedMSG);
        std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_Battery> ptrBattery = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_Battery>(newItem);
        PublishVehicleData(sender, ptrBattery);
        break;
    }
    case MAVLINK_MSG_ID_GPS_RAW_INT:
    {
        //This is message definition 24
        //The global position, as returned by the Global Positioning System (GPS). This is NOT the global position estimate of the system, but rather a RAW sensor value. See message GLOBAL_POSITION for the global position estimate. Coordinate frame is right-handed, Z-axis up (GPS frame).
        mavlink_gps_raw_int_t decodedMSG;
        mavlink_msg_gps_raw_int_decode(message,&decodedMSG);
        DataGenericItem::DataGenericItem_GPS newItem(decodedMSG);
        std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_GPS> ptrGPS = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_GPS>(newItem);
        PublishVehicleData(sender, ptrGPS);
        break;
    }
    case MAVLINK_MSG_ID_GPS_STATUS:
    {
        //This is message definition 25
        //The positioning status, as reported by GPS. This message is intended to display status information about each satellite visible to the receiver. See message GLOBAL_POSITION for the global position estimate. This message can contain information for up to 20 satellites.
        break;
    }
    case MAVLINK_MSG_ID_ATTITUDE:
    {
        //This is message definition 30
        //The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right).
        mavlink_attitude_t decodedMSG;
        mavlink_msg_attitude_decode(message,&decodedMSG);
        mace::pose::Rotation_3D newRotation(static_cast<double>(decodedMSG.roll), static_cast<double>(decodedMSG.pitch), static_cast<double>(decodedMSG.yaw));
        mace::pose_topics::Topic_AgentOrientationPtr ptrAttitude = std::make_shared<mace::pose_topics::Topic_AgentOrientation>(&newRotation);
        PublishVehicleData(sender, ptrAttitude);
        break;
    }
    case MAVLINK_MSG_ID_ATTITUDE_QUATERNION:
    {
        //This is message definition 30
        //The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right).
        mavlink_attitude_quaternion_t decodedMSG;
        mavlink_msg_attitude_quaternion_decode(message, &decodedMSG);
        mace::pose::Rotation_3D newRotation;
        //Ken Fix This
        break;
    }
//    case MAVLINK_MSG_ID_ATTITUDE_STATE_FULL:
//    {
        //This is message definition 30
        //The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right).
//        mavlink_attitude_state_full_t decodedMSG;
//        mavlink_msg_attitude_state_full_decode(message,&decodedMSG);
//        DataState::StateAttitude newAttitude(decodedMSG);
//        std::shared_ptr<DataStateTopic::StateAttitudeTopic> ptrAttitude = std::make_shared<DataStateTopic::StateAttitudeTopic>(newAttitude);
//        PublishVehicleData(sender, ptrAttitude);
//        break;
//    }
    case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
    {
        //This is message definition 32
        //The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right).
        mavlink_local_position_ned_t decodedMSG;
        mavlink_msg_local_position_ned_decode(message,&decodedMSG);
        //Ken Fix
//        DataState::StateLocalPosition newPosition(decodedMSG);
//        std::shared_ptr<DataStateTopic::StateLocalPositionTopic> ptrLocalPosition = std::make_shared<DataStateTopic::StateLocalPositionTopic>(newPosition);

//        PublishVehicleData(sender, ptrLocalPosition);

        break;
    }
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
    {
        //This is message definition 33
        //The filtered global position (e.g. fused GPS and accelerometers). The position is in GPS-frame (right-handed, Z-up). It is designed as scaled integer message since the resolution of float is not sufficient.
        mavlink_global_position_int_t decodedMSG;
        mavlink_msg_global_position_int_decode(message,&decodedMSG);
        mace::pose::GeodeticPosition_3D currentPosition;
        currentPosition.fromMACEMsg(decodedMSG);
        mace::pose_topics::Topic_GeodeticPositionPtr ptrPosition = std::make_shared<mace::pose_topics::Topic_GeodeticPosition>(&currentPosition);
        PublishVehicleData(sender, ptrPosition);

        break;
    }
    case MAVLINK_MSG_ID_VFR_HUD:
    {
        //This is message definition 74
        //Metrics typically displayed on a HUD for fixed wing aircraft

        mavlink_vfr_hud_t decodedMSG;
        mavlink_msg_vfr_hud_decode(message,&decodedMSG);

        mace::measurements::Speed currentAirspeed;
        currentAirspeed.setSpeed(static_cast<double>(decodedMSG.airspeed));
        currentAirspeed.updateSpeedType(SPEED_TYPE::SPEED_TYPE_AIRSPEED);
        mace::measurement_topics::Topic_AirSpeedPtr ptrAirspeedTopic = std::make_shared<mace::measurement_topics::Topic_AirSpeed>(currentAirspeed);

        PublishVehicleData(sender, ptrAirspeedTopic);

        mace::measurements::Speed currentGroundSpeed;
        currentGroundSpeed.setSpeed(static_cast<double>(decodedMSG.groundspeed));
        currentGroundSpeed.updateSpeedType(SPEED_TYPE::SPEED_TYPE_GROUNDSPEED);

        break;
    }
    case MAVLINK_MSG_ID_RADIO_STATUS:
    {
        //This is message definition 109
        //Status generated by radio and injected into MAVLink stream.
        mavlink_radio_status_t decodedMSG;
        mavlink_msg_radio_status_decode(message,&decodedMSG);
        break;
    }
    case MAVLINK_MSG_ID_POWER_STATUS:
    {
        //This is message definition 125
        break;
    }

    case MAVLINK_MSG_ID_STATUSTEXT:
    {
        //This is message definition 253
        mavlink_statustext_t decodedMSG;
        mavlink_msg_statustext_decode(message,&decodedMSG);
        DataGenericItem::DataGenericItem_Text newText(decodedMSG);
        std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_Text> ptrStatusText = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_Text>(newText);
        PublishVehicleData(sender, ptrStatusText);
        break;
    }

    ////////////////////////////////////////////////////////////////////////////
    /// HOME BASED EVENTS:
    ////////////////////////////////////////////////////////////////////////////

    /*
    case MAVLINK_MSG_ID_MISSION_REQUEST_HOME:
    {
        std::cout<<"Saw a mission request home"<<std::endl;

        mavlink_mission_request_home_t decodedMSG;
        mavlink_msg_mission_request_home_decode(message,&decodedMSG);


        auto func = [this](int vehicleID)
        {
            CommandItem::SpatialHome home = this->getDataObject()->GetVehicleHomePostion(vehicleID);

            mavlink_message_t msg;
            mavlink_home_position_t homeMACE;
            homeMACE.latitude = home.position->getX() * pow(10,7);
            homeMACE.longitude = home.position->getY() * pow(10,7);
            homeMACE.altitude = home.position->getZ() * pow(10,3);
            mavlink_msg_home_position_encode_chan(associatedSystemID,0,m_LinkChan,&msg,&homeMACE);
            transmitMessage(msg);
        };

        if(decodedMSG.target_system == 0) {
            //need to iterate over all internal vehicles
            std::vector<int> vehicles;
            this->getDataObject()->GetLocalVehicles(vehicles);
            for(auto it = vehicles.cbegin() ; it != vehicles.cend() ; ++it) {
                func(*it);
            }
        }
        else {
            func(decodedMSG.target_system);
        }
        break;
    }
    case MAVLINK_MSG_ID_SET_HOME_POSITION:
    {
        //we need to respond to this with an acknowledgement receiving it
        mavlink_set_home_position_t decodedMSG;
        mavlink_msg_set_home_position_decode(message,&decodedMSG);

        mavlink_message_t msg;
        mavlink_home_position_ack_t ack;
        ack.target_system = systemID;
        ack.ack = 0;
        mavlink_msg_home_position_ack_encode_chan(associatedSystemID,0,m_LinkChan,&msg,&ack);
        transmitMessage(msg);

        CommandItem::SpatialHome systemHome;
        systemHome.position->setCoordinateFrame(Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT);
        systemHome.position->setX(decodedMSG.latitude / pow(10,7));
        systemHome.position->setY(decodedMSG.longitude / pow(10,7));
        systemHome.position->setZ(decodedMSG.altitude / pow(10,3));
        systemHome.setTargetSystem(decodedMSG.target_system);

        ModuleExternalLink::NotifyListeners([&](MaceCore::IModuleEventsExternalLink* ptr){
            ptr->Event_SetHomePosition(this, systemHome);
        });
        break;
    }
    */
    case MAVLINK_MSG_ID_HOME_POSITION:
    {
        mavlink_home_position_t decodedMSG;
        mavlink_msg_home_position_decode(message,&decodedMSG);

        MaceLog::Emergency("Received home position message in EL. Should we do anything with it?");
//        if(m_HomeController->isThreadActive())
//            m_HomeController->receivedMissionHome(decodedMSG);
//        else
//        {
//            CommandItem::SpatialHome newHome;
//            newHome.position->setX(decodedMSG.latitude / pow(10,7));
//            newHome.position->setY(decodedMSG.longitude / pow(10,7));
//            newHome.position->setZ(decodedMSG.altitude / pow(10,7));
//            newHome.setOriginatingSystem(systemID);
//            newHome.setTargetSystem(systemID);
//            cbiHomeController_ReceviedHome(newHome);
//        }

        break;
    }


    ////////////////////////////////////////////////////////////////////////////
    /// MISSION BASED EVENTS:
    ////////////////////////////////////////////////////////////////////////////

    case MAVLINK_MSG_ID_NEW_ONBOARD_MISSION:
    {
        /*
        mavlink_new_onboard_mission_t decodedMSG;
        mavlink_msg_new_onboard_mission_decode(message,&decodedMSG);

        MissionItem::MISSIONTYPE missionType = static_cast<MissionItem::MISSIONTYPE>(decodedMSG.mission_type);
        MissionItem::MISSIONSTATE missionState = static_cast<MissionItem::MISSIONSTATE>(decodedMSG.mission_state);

        MissionItem::MissionKey key(decodedMSG.mission_system,decodedMSG.mission_creator,decodedMSG.mission_id,missionType,missionState);

        ModuleExternalLink::NotifyListeners([&](MaceCore::IModuleEventsExternalLink* ptr){
            ptr->ExternalEvent_NewOnboardMission(this, key);
        });

        printf("Notified of Remote Mission. S_ID: %d M_ID: %d\n", key.m_systemID, key.m_missionID);

        */

        //m_MissionController->requestMission(key, sender);

        //bool valid = this->getDataObject()->getMissionKeyValidity(key);
        break;
    }
//    case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
//    {
//        mavlink_mission_request_list_t decodedMSG;
//        mavlink_msg_mission_request_list_decode(message,&decodedMSG);
////        decodedMSG.target_component;
////        decodedMSG.target_system;


//        MissionItem::MISSIONTYPE missionType = static_cast<MissionItem::MISSIONTYPE>(decodedMSG.mission_type);

//        ModuleExternalLink::NotifyListeners([&](MaceCore::IModuleEventsExternalLink* ptr){
//            ptr->Event_GetOnboardMission(this, decodedMSG.target_system, missionType);
//        });


//        MaceLog::Info("Get onboard mission for: " + std::to_string(decodedMSG.target_system) + " (type: " + MissionItem::MissionTypeToString(missionType) + ")");


//        break;
//    }

//    case MAVLINK_MSG_ID_MACE_MISSION_COUNT:
//    {
//        mavlink_mace_mission_count_t decodedMSG;
//        mavlink_msg_mace_mission_count_decode(message, &decodedMSG);

////        decodedMSG.count;
////        decodedMSG.mission_creator;
////        decodedMSG.mission_id;
////        decodedMSG.mission_type;
////        decodedMSG.target_component;
////        decodedMSG.target_system;

//        ModuleExternalLink::NotifyListeners([&](MaceCore::IModuleEventsExternalLink* ptr){
////            ptr->Even (this, decodedMSG.target_system, missionType);
//        });

//        break;
//    }
//    case MAVLINK_MSG_ID_MACE_MISSION_REQUEST_INT:
//    {
//        mavlink_mace_mission_request_int_t decodedMSG;
//        mavlink_msg_mace_mission_request_int_decode(message, &decodedMSG);

//        break;
//    }
//    case MAVLINK_MSG_ID_MACE_MISSION_ITEM_INT:
//    {
//        mavlink_mace_mission_item_int_t decodedMSG;
//        mavlink_msg_mace_mission_item_int_decode(message, &decodedMSG);

//        break;
//    }

    case MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST:
    {
        break;
    }
    case MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST:
    {
        break;
    }
    case MAVLINK_MSG_ID_MISSION_SET_CURRENT:
    {
        mavlink_mission_set_current_t decodedMSG;
        mavlink_msg_mission_set_current_decode(message,&decodedMSG);

        break;
    }
    case MAVLINK_MSG_ID_MISSION_CURRENT:
    {

        mavlink_mission_current_t decodedMSG;
        mavlink_msg_mission_current_decode(message,&decodedMSG);

//        MissionItem::MissionItemCurrent current;
//        MissionItem::MissionKey key(decodedMSG.mission_system,decodedMSG.mission_creator,decodedMSG.mission_id,static_cast<MissionItem::MISSIONTYPE>(decodedMSG.mission_type),static_cast<MissionItem::MISSIONSTATE>(decodedMSG.mission_state));
//        current.setMissionKey(key);
//        current.setMissionCurrentIndex(decodedMSG.seq);

//        std::shared_ptr<MissionTopic::MissionItemCurrentTopic> ptrMissionCurrent = std::make_shared<MissionTopic::MissionItemCurrentTopic>(current);

        //This function updates MACECore
//        ModuleExternalLink::NotifyListeners([&](MaceCore::IModuleEventsExternalLink* ptr){
//            ptr->GVEvents_MissionItemCurrent(this, current);
//        });

//        PublishMissionData(sender, ptrMissionCurrent);
        break;
    }
    case MAVLINK_MSG_ID_MISSION_ITEM_REACHED:
    {
//        mavlink_mission_item_reached_t decodedMSG;
//        mavlink_msg_mission_item_reached_decode(message,&decodedMSG);

//        MissionItem::MissionKey key(decodedMSG.mission_system,decodedMSG.mission_creator,decodedMSG.mission_id,static_cast<MissionItem::MISSIONTYPE>(decodedMSG.mission_type),static_cast<MissionItem::MISSIONSTATE>(decodedMSG.mission_state));
//        MissionItem::MissionItemAchieved achieved;
//        achieved.setMissionKey(key);
//        achieved.setMissionAchievedIndex(decodedMSG.seq);
//        std::shared_ptr<MissionTopic::MissionItemReachedTopic> ptrMissionReached = std::make_shared<MissionTopic::MissionItemReachedTopic>(achieved);

//        PublishMissionData(sender, ptrMissionReached);
        break;
    }
//    case MAVLINK_MSG_ID_MISSION_EXE_STATE:
//    {
//        mavlink_mission_exe_state_t decodedMSG;
//        mavlink_msg_mission_exe_state_decode(message,&decodedMSG);
//        MissionItem::MissionKey key(decodedMSG.mission_system,decodedMSG.mission_creator,decodedMSG.mission_id,static_cast<MissionItem::MISSIONTYPE>(decodedMSG.mission_type));
//        Data::MissionExecutionState state = static_cast<Data::MissionExecutionState>(decodedMSG.mission_state);

//        ModuleExternalLink::NotifyListeners([&](MaceCore::IModuleEventsExternalLink* ptr){
//            ptr->GVEvents_MissionExeStateUpdated(this, key, state);
//        });

//        break;
//    }


    case MAVLINK_MSG_ID_GUIDED_TARGET_STATS:
    {
        mavlink_guided_target_stats_t decodedMSG;
        mavlink_msg_guided_target_stats_decode(message,&decodedMSG);

        std::shared_ptr<MissionTopic::VehicleTargetTopic> ptrTarget = std::make_shared<MissionTopic::VehicleTargetTopic>(decodedMSG);
        ptrTarget->setVehicleID(message->sysid);


         PublishMissionData(sender, ptrTarget);
        break;
    }
    case MAVLINK_MSG_ID_COMMAND_ACK:
    {
        mavlink_command_ack_t decodedMSG;
        mavlink_msg_command_ack_decode(message, &decodedMSG);
        // TODO: Anything with this ACK?
        MaceLog::Emergency("Received command_ack over external link, should we do anything with it?");
        break;
    }
    default:
    {
        std::cout<<"I received an unknown supported message with the ID "<<(int)message->msgid<<std::endl;
    }

    }//end of switch statement

}
