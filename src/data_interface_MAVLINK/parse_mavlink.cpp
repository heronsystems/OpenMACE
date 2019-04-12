#include "vehicle_object_mavlink.h"

namespace DataInterface_MAVLINK
{
void VehicleObject_MAVLINK::parseMessage(const mavlink_message_t *msg){

    ///////////////////////////////////////////////////////////////////////////////
    /// VEHICLE DATA ITEMS: The following contain information about the direct
    /// state of the vehicle. Each are used in a comparitive operation to
    /// determine if the data has changed and should be published throughout MACE.
    //////////////////////////////////////////////////////////////////////////////

    switch ((int)msg->msgid) {
    case MAVLINK_MSG_ID_SYS_STATUS:
    {
        mavlink_sys_status_t decodedMSG;
        mavlink_msg_sys_status_decode(msg,&decodedMSG);

        DataGenericItem::DataGenericItem_Battery battery;
        battery.setBatteryVoltage(decodedMSG.voltage_battery/1000.0);
        battery.setBatteryCurrent(decodedMSG.current_battery/100.0);
        battery.setBatteryRemaining(decodedMSG.battery_remaining);
        if(state->vehicleFuel.set(battery))
        {
            std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_Battery> ptrBattery = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_Battery>(battery);
            if(this->m_CB != NULL)
                this->m_CB->cbi_VehicleStateData(systemID,ptrBattery);
        }
        break;
    }
    case MAVLINK_MSG_ID_GPS_RAW_INT:
    {
        //This is message definition 24
        //The global position, as returned by the Global Positioning System (GPS). This is NOT the global position estimate of the system, but rather a RAW sensor value. See message GLOBAL_POSITION for the global position estimate. Coordinate frame is right-handed, Z-axis up (GPS frame).
        mavlink_gps_raw_int_t decodedMSG;
        mavlink_msg_gps_raw_int_decode(msg,&decodedMSG);
        DataGenericItem::DataGenericItem_GPS gpsItem;
        gpsItem.setHDOP(decodedMSG.eph);
        gpsItem.setVDOP(decodedMSG.epv);
        gpsItem.setSatVisible(decodedMSG.satellites_visible);

        switch(decodedMSG.fix_type)
        {
        case GPS_FIX_TYPE_2D_FIX:
            gpsItem.setGPSFix(gpsItem.GPSFixType::GPS_FIX_2D_FIX);
            break;
        case GPS_FIX_TYPE_3D_FIX:
            gpsItem.setGPSFix(gpsItem.GPSFixType::GPS_FIX_3D_FIX);
            break;
        case GPS_FIX_TYPE_DGPS:
            gpsItem.setGPSFix(gpsItem.GPSFixType::GPS_FIX_DGPS);
            break;
        case GPS_FIX_TYPE_NO_FIX:
            gpsItem.setGPSFix(gpsItem.GPSFixType::GPS_FIX_NO_FIX);
            break;
        case GPS_FIX_TYPE_NO_GPS:
            gpsItem.setGPSFix(gpsItem.GPSFixType::GPS_FIX_NONE);
            break;
        case GPS_FIX_TYPE_RTK_FIXED:
            gpsItem.setGPSFix(gpsItem.GPSFixType::GPS_FIX_RTK_FIXED);
            break;
        case GPS_FIX_TYPE_RTK_FLOAT:
            gpsItem.setGPSFix(gpsItem.GPSFixType::GPS_FIX_RTK_FLOAT);
            break;
        case GPS_FIX_TYPE_STATIC:
            gpsItem.setGPSFix(gpsItem.GPSFixType::GPS_FIX_STATIC);
            break;
        default:
            gpsItem.setGPSFix(gpsItem.GPSFixType::GPS_FIX_NO_FIX);
            break;
        }

        if(state->vehicleGPSStatus.set(gpsItem))
        {
            std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_GPS> ptrGPSStatus = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_GPS>(gpsItem);
            if(this->m_CB != NULL)
                this->m_CB->cbi_VehicleStateData(systemID, ptrGPSStatus);
        }

        break;
    }
    case MAVLINK_MSG_ID_ATTITUDE:
    {
        //This is message definition 30
        //The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right).
        mavlink_attitude_t decodedMSG;
        mavlink_msg_attitude_decode(msg,&decodedMSG);
        DataState::StateAttitude attitude;
        attitude.setAttitude(decodedMSG.roll,decodedMSG.pitch,decodedMSG.yaw);
        attitude.setAttitudeRates(decodedMSG.rollspeed,decodedMSG.pitchspeed,decodedMSG.yawspeed);

        if(state->vehicleAttitude.set(attitude))
        {
            std::shared_ptr<DataStateTopic::StateAttitudeTopic> ptrAttitude = std::make_shared<DataStateTopic::StateAttitudeTopic>(attitude);
            if(this->m_CB != NULL)
                this->m_CB->cbi_VehicleStateData(systemID,ptrAttitude);
        }
        break;
    }
    case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
    {
        //This is message definition 32
        //The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right).
        mavlink_local_position_ned_t decodedMSG;
        mavlink_msg_local_position_ned_decode(msg,&decodedMSG);

        DataState::StateLocalPosition localPosition(Data::CoordinateFrameType::CF_LOCAL_NED);
        localPosition.setPositionX(decodedMSG.x);
        localPosition.setPositionY(decodedMSG.y);
        localPosition.setPositionZ(decodedMSG.z);

        if(state->vehicleLocalPosition.set(localPosition))
        {
            std::shared_ptr<DataStateTopic::StateLocalPositionTopic> ptrLocalPosition = std::make_shared<DataStateTopic::StateLocalPositionTopic>(localPosition);
            if(this->m_CB != NULL)
                this->m_CB->cbi_VehicleStateData(systemID,ptrLocalPosition);
        }
        break;
    }
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
    {
        state->performCallback();

        //This is message definition 33
        //The filtered global position (e.g. fused GPS and accelerometers). The position is in GPS-frame (right-handed, Z-up). It is designed as scaled integer message since the resolution of float is not sufficient.
        mavlink_global_position_int_t decodedMSG;
        mavlink_msg_global_position_int_decode(msg,&decodedMSG);
        double power = pow(10,7);

        DataState::StateGlobalPosition position;
        position.setPosition(decodedMSG.lat/power,decodedMSG.lon/power,decodedMSG.relative_alt/1000.0);
        //check that something has actually changed
        if(state->vehicleGlobalPosition.set(position))
        {
            std::shared_ptr<DataStateTopic::StateGlobalPositionTopic> ptrPosition = std::make_shared<DataStateTopic::StateGlobalPositionTopic>(position);
            if(this->m_CB != NULL)
                this->m_CB->cbi_VehicleStateData(systemID,ptrPosition);
        }

        DataState::StateGlobalPositionEx positionEx;
        positionEx.setPosition(decodedMSG.lat/power,decodedMSG.lon/power,decodedMSG.alt/1000.0);
        positionEx.heading = (decodedMSG.hdg/100.0)*(3.14/180.0);

        //check that something has actually changed
        if(state->vehicleGlobalPositionEx.set(positionEx))
        {
            std::shared_ptr<DataStateTopic::StateGlobalPositionExTopic> ptrPositionEx = std::make_shared<DataStateTopic::StateGlobalPositionExTopic>(positionEx);
            if(this->m_CB != NULL)
                this->m_CB->cbi_VehicleStateData(systemID,ptrPositionEx);
        }
        break;
    }
    case MAVLINK_MSG_ID_VFR_HUD:
    {
        //This is message definition 74
        //Metrics typically displayed on a HUD for fixed wing aircraft
        mavlink_vfr_hud_t decodedMSG;
        mavlink_msg_vfr_hud_decode(msg,&decodedMSG);

        DataState::StateAirspeed airspeed;
        airspeed.setAirspeed(decodedMSG.airspeed);
        //check that something has actually changed

        if(state->vehicleAirspeed.set(airspeed))
        {
            std::shared_ptr<DataStateTopic::StateAirspeedTopic> ptrAirspeedTopic = std::make_shared<DataStateTopic::StateAirspeedTopic>(airspeed);
            if(this->m_CB != NULL)
                this->m_CB->cbi_VehicleStateData(systemID,ptrAirspeedTopic);
        }
        break;
    }
    case MAVLINK_MSG_ID_RADIO_STATUS:
    {
        //This is message definition 109
        //Status generated by radio and injected into MAVLink stream.
        mavlink_radio_status_t decodedMSG;
        mavlink_msg_radio_status_decode(msg,&decodedMSG);
        break;
    }
    case MAVLINK_MSG_ID_POWER_STATUS:
    {
        //This is message definition 125
        break;
    }
    case MAVLINK_MSG_ID_BATTERY_STATUS:
    {
        //This is message definition 147
        //Battery information
        mavlink_battery_status_t decodedMSG;
        mavlink_msg_battery_status_decode(msg,&decodedMSG);
        break;
    }
    case MAVLINK_MSG_ID_STATUSTEXT:
    {
        //This is message definition 253
        mavlink_statustext_t decodedMSG;
        mavlink_msg_statustext_decode(msg,&decodedMSG);

        DataGenericItem::DataGenericItem_Text statusText;
        statusText.setText(decodedMSG.text);
        switch (decodedMSG.severity) {
        case MAV_SEVERITY_EMERGENCY:
            statusText.setSeverity(statusText.STATUS_SEVERITY::STATUS_EMERGENCY);
            break;
        case MAV_SEVERITY_ALERT:
            statusText.setSeverity(statusText.STATUS_SEVERITY::STATUS_ALERT);
            break;
        case MAV_SEVERITY_CRITICAL:
            statusText.setSeverity(statusText.STATUS_SEVERITY::STATUS_CRITICAL);
            break;
        case MAV_SEVERITY_ERROR:
            statusText.setSeverity(statusText.STATUS_SEVERITY::STATUS_ERROR);
            break;
        case MAV_SEVERITY_WARNING:
            statusText.setSeverity(statusText.STATUS_SEVERITY::STATUS_WARNING);
            break;
        case MAV_SEVERITY_NOTICE:
            statusText.setSeverity(statusText.STATUS_SEVERITY::STATUS_NOTICE);
            break;
        case MAV_SEVERITY_INFO:
            statusText.setSeverity(statusText.STATUS_SEVERITY::STATUS_INFO);
            break;
        case MAV_SEVERITY_DEBUG:
            statusText.setSeverity(statusText.STATUS_SEVERITY::STATUS_DEBUG);
            break;
        default:
            break;
        }

        state->vehicleTextAlert.set(statusText);

        std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_Text> ptrStatusText = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_Text>(statusText);
        if(this->m_CB != NULL)
            this->m_CB->cbi_VehicleStateData(systemID,ptrStatusText);
        break;
    }

        /////////////////////////////////////////////////////////////////////////
        /// MISSION ITEMS: The following case statements are executed
        /// for mission based message events.
        /////////////////////////////////////////////////////////////////////////

    case MAVLINK_MSG_ID_COMMAND_ACK:
    {
        mavlink_command_ack_t decodedMSG;
        mavlink_msg_command_ack_decode(msg,&decodedMSG);
        this->m_CommandController->receivedCommandACK(decodedMSG);
        if(this->m_CB != NULL)
            this->m_CB->cbi_VehicleCommandACK(systemID,decodedMSG);
        break;
    }


        /////////////////////////////////////////////////////////////////////////
        /// MISSION ITEMS: The following case statements are executed
        /// for mission based message events.
        /////////////////////////////////////////////////////////////////////////

    case MAVLINK_MSG_ID_MISSION_ITEM:
    {
        //This is message definition 39
        //Message encoding a mission item. This message is emitted to announce the presence of a mission item and to set a mission item on the system. The mission item can be either in x, y, z meters (type: LOCAL) or x:lat, y:lon, z:altitude. Local frame is Z-down, right handed (NED), global frame is Z-up, right handed (ENU). See also http://qgroundcontrol.org/mavlink/waypoint_protocol.
        mavlink_mission_item_t decodedMSG;
        mavlink_msg_mission_item_decode(msg,&decodedMSG);
        this->m_MissionController->recievedMissionItem(decodedMSG);
        break;
    }
    case MAVLINK_MSG_ID_MISSION_REQUEST:
    {
        //This is message definition 40
        //Request the information of the mission item with the sequence number seq. The response of the system to this message should be a MISSION_ITEM message. http://qgroundcontrol.org/mavlink/waypoint_protocol
        mavlink_mission_request_t decodedMSG;
        mavlink_msg_mission_request_decode(msg,&decodedMSG);
        this->m_MissionController->transmitMissionItem(decodedMSG);
        break;
    }
    case MAVLINK_MSG_ID_MISSION_CURRENT:
    {
        //This is message definition 42
        //Message that announces the sequence number of the current active mission item. The MAV will fly towards this mission item.
        mavlink_mission_current_t decodedMSG;
        mavlink_msg_mission_current_decode(msg,&decodedMSG);

        MissionItem::MissionItemCurrent current;
        current.setMissionKey(mission->getCurrentAutoMissionKey());
        current.setMissionCurrentIndex(decodedMSG.seq);

        if(mission->missionItemCurrent.set(current))
        {
            if(decodedMSG.seq == 0)
            {
                //the current target is home and we should handle this differently
            }
            else{
                int currentIndex = decodedMSG.seq - 1;

                MissionItem::MissionItemCurrent current;
                current.setMissionKey(mission->getCurrentAutoMissionKey());
                current.setMissionCurrentIndex(currentIndex);

                if(this->m_CB != NULL)
                    m_CB->cbi_VehicleMissionItemCurrent(current);
            }
        }

        break;
    }
    case MAVLINK_MSG_ID_MISSION_COUNT:
    {
        //This is message definition 44
        //This message is emitted as response to MISSION_REQUEST_LIST by the MAV and to initiate a write transaction.
        //The GCS can then request the individual mission item based on the knowledge of the total number of MISSION
        mavlink_mission_count_t decodedMSG;
        mavlink_msg_mission_count_decode(msg,&decodedMSG);
        this->m_MissionController->receivedMissionCount(decodedMSG);
        break;
    }
    case MAVLINK_MSG_ID_MISSION_ITEM_REACHED:
    {
        //This is message definition 46
        //A certain mission item has been reached. The system will either hold this position (or circle on the orbit) or
        //(if the autocontinue on the WP was set) continue to the next MISSION.
        mavlink_mission_item_reached_t decodedMSG;
        mavlink_msg_mission_item_reached_decode(msg,&decodedMSG);
        int missionIndex = decodedMSG.seq - 1; //transforms the reference away from mavlink to MACE

        if(missionIndex >= 0)
        {
            MissionItem::MissionItemAchieved itemAchieved;
            itemAchieved.setMissionKey(mission->getCurrentAutoMissionKey());
            itemAchieved.setMissionAchievedIndex(missionIndex);

            if(mission->missionItemReached.set(itemAchieved))
            {
                std::shared_ptr<MissionTopic::MissionItemReachedTopic> ptrMissionTopic = std::make_shared<MissionTopic::MissionItemReachedTopic>(itemAchieved);
                if(this->m_CB != NULL)
                    m_CB->cbi_VehicleMissionData(systemID,ptrMissionTopic);
            }
        }
        else{
            //we have reached the home position, should we do anything here?
        }

        break;
    }
    case MAVLINK_MSG_ID_MISSION_ACK:
    {
        //This is message definition 47
        //Ack message during MISSION handling. The type field states if this message is a positive ack (type=0) or if an error happened (type=non-zero).
        mavlink_mission_ack_t decodedMSG;
        mavlink_msg_mission_ack_decode(msg,&decodedMSG);
        if(this->m_MissionController->getCommsState() == Data::ControllerCommsState::TRANSMITTING)
            this->m_MissionController->receivedMissionACK(decodedMSG);
        else if(this->m_GuidedController->getCommsState() == Data::ControllerCommsState::TRANSMITTING)
            this->m_GuidedController->receivedMissionACK(decodedMSG);
        break;
    }
    case MAVLINK_MSG_ID_HOME_POSITION:
    {
        //This is message definition 242
        //This message can be requested by sending the MAV_CMD_GET_HOME_POSITION command. The position the system will return to and land on.
        //The position is set automatically by the system during the takeoff in case it was not explicitely set by the operator before or after.
        //The position the system will return to and land on. The global and local positions encode the position in the respective coordinate frames,
        //while the q parameter encodes the orientation of the surface. Under normal conditions it describes the heading and terrain slope, which can
        //be used by the aircraft to adjust the approach. The approach 3D vector describes the point to which the system should fly in normal flight
        //mode and then perform a landing sequence along the vector.
        mavlink_home_position_t decodedMSG;
        mavlink_msg_home_position_decode(msg,&decodedMSG);

        CommandItem::SpatialHome spatialHome;
        spatialHome.position->setX(decodedMSG.latitude / pow(10,7));
        spatialHome.position->setY(decodedMSG.longitude / pow(10,7));
        spatialHome.position->setZ(decodedMSG.altitude / 1000);
        spatialHome.setOriginatingSystem(systemID);
        mission->home.set(spatialHome);
        if(this->m_CB)
            this->cbiMissionController_ReceviedHome(spatialHome);
        break;
    }

    default:
    {
        //std::cout<<"I received an unknown supported message with the ID "<<(int)message->msgid<<std::endl;
    }
    } //end of switch statement
}

} //end of namespace DataInterface_MAVLINK
