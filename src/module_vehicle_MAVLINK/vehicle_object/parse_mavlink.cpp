#include "mavlink_vehicle_object.h"


bool MavlinkVehicleObject::parseMessage(const mavlink_message_t *msg){
    bool consumed = true;
    uint8_t systemID = msg->sysid;
    ///////////////////////////////////////////////////////////////////////////////
    /// VEHICLE DATA ITEMS: The following contain information about the direct
    /// state of the vehicle. Each are used in a comparitive operation to
    /// determine if the data has changed and should be published throughout MACE.
    //////////////////////////////////////////////////////////////////////////////

    switch (msg->msgid) {
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
            if(this->m_CB)
                this->m_CB->cbi_VehicleStateData(systemID,ptrBattery);
        }
        break;
    }
    case MAVLINK_MSG_ID_SYSTEM_TIME:
    {
        //This is message definition 2
        mavlink_system_time_t decodedMSG;
        mavlink_msg_system_time_decode(msg, &decodedMSG);
        DataGenericItem::DataGenericItem_SystemTime systemTime;
        systemTime.setTimeSinceEpoch(decodedMSG.time_unix_usec);
        systemTime.setTimeSinceBoot(decodedMSG.time_boot_ms);

        // Check the GPS fix is greater than 3D before updating system time
        DataGenericItem::DataGenericItem_GPS gpsItem = state->vehicleGPSStatus.get();
        // Check our system time is different from prior data before setting system time:
        if(gpsItem.is3DorGreater() && (state->vehicleSystemTime.set(systemTime))) {
            std::shared_ptr<DataGenericItem::DataGenericItem_SystemTime> ptrSystemTime = std::make_shared<DataGenericItem::DataGenericItem_SystemTime>(systemTime);
            if(this->m_CB){
                this->m_CB->cbi_VehicleSystemTime(systemID, ptrSystemTime);
            }
        }

        break;
    }
    case MAVLINK_MSG_ID_GPS_RAW_INT:
    {
        //This is message definition 24
        //The global position, as returned by the Global Positioning System (GPS). This is NOT the global position estimate of the system, but rather a RAW sensor value. See message GLOBAL_POSITION for the global position estimate. Coordinate frame is right-handed, Z-axis up (GPS frame).
        mavlink_gps_raw_int_t decodedMSG;
        mavlink_msg_gps_raw_int_decode(msg, &decodedMSG);
        DataGenericItem::DataGenericItem_GPS gpsItem;
        gpsItem.setHDOP(decodedMSG.eph);
        gpsItem.setVDOP(decodedMSG.epv);
        gpsItem.setSatVisible(decodedMSG.satellites_visible);

        using namespace DataGenericItem;
        switch(decodedMSG.fix_type)
        {
        case GPS_FIX_TYPE_2D_FIX:
            gpsItem.setGPSFix(DataGenericItem_GPS::GPSFixType::GPS_FIX_2D_FIX);
            break;
        case GPS_FIX_TYPE_3D_FIX:
            gpsItem.setGPSFix(DataGenericItem_GPS::GPSFixType::GPS_FIX_3D_FIX);
            break;
        case GPS_FIX_TYPE_DGPS:
            gpsItem.setGPSFix(DataGenericItem_GPS::GPSFixType::GPS_FIX_DGPS);
            break;
        case GPS_FIX_TYPE_NO_FIX:
            gpsItem.setGPSFix(DataGenericItem_GPS::GPSFixType::GPS_FIX_NO_FIX);
            break;
        case GPS_FIX_TYPE_NO_GPS:
            gpsItem.setGPSFix(DataGenericItem_GPS::GPSFixType::GPS_FIX_NONE);
            break;
        case GPS_FIX_TYPE_RTK_FIXED:
            gpsItem.setGPSFix(DataGenericItem_GPS::GPSFixType::GPS_FIX_RTK_FIXED);
            break;
        case GPS_FIX_TYPE_RTK_FLOAT:
            gpsItem.setGPSFix(DataGenericItem_GPS::GPSFixType::GPS_FIX_RTK_FLOAT);
            break;
        case GPS_FIX_TYPE_STATIC:
            gpsItem.setGPSFix(DataGenericItem_GPS::GPSFixType::GPS_FIX_STATIC);
            break;
        default:
            gpsItem.setGPSFix(DataGenericItem_GPS::GPSFixType::GPS_FIX_NO_FIX);
            break;
        }

        if(state->vehicleGPSStatus.set(gpsItem))
        {
            std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_GPS> ptrGPSStatus = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_GPS>(gpsItem);
            if(this->m_CB)
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
        mace::pose::Rotation_3D agentAttitude(static_cast<double>(decodedMSG.roll),
                                              static_cast<double>(decodedMSG.pitch),
                                              static_cast<double>(decodedMSG.yaw));

        mace::pose::Velocity_Rotation3D agentRotationalRate;
        agentRotationalRate.data = Eigen::Vector3d(static_cast<double>(decodedMSG.rollspeed),
                                                   static_cast<double>(decodedMSG.pitchspeed),
                                                   static_cast<double>(decodedMSG.yawspeed));

        Data::EnvironmentTime currentTime;
        Data::EnvironmentTime::CurrentTime(Data::Devices::SYSTEMCLOCK, currentTime);
        prevAttitude = currentTime;

        if(state->vehicleAttitude.set(agentAttitude))
        {
            std::shared_ptr<mace::pose_topics::Topic_AgentOrientation> ptrAttitude = std::make_shared<mace::pose_topics::Topic_AgentOrientation>(&agentAttitude);
            if(this->m_CB)
                this->m_CB->cbi_VehicleStateData(systemID,ptrAttitude);
        }

        if(state->vehicleRotationalVelocity.set(agentRotationalRate))
        {
            std::shared_ptr<mace::pose_topics::Topic_RotationalVelocity> ptrAttitudeRate = std::make_shared<mace::pose_topics::Topic_RotationalVelocity>(agentRotationalRate);
            if(this->m_CB)
                this->m_CB->cbi_VehicleStateData(systemID,ptrAttitudeRate);
        }


        break;
    }
    case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
    {
        //This is message definition 32
        //The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right).
        mavlink_local_position_ned_t decodedMSG;
        mavlink_msg_local_position_ned_decode(msg,&decodedMSG);

        //Ken Fix This and update coordinate frame
        mace::pose::CartesianPosition_3D localPosition(CartesianFrameTypes::CF_LOCAL_NED,
                                                       static_cast<double>(decodedMSG.x),
                                                       static_cast<double>(decodedMSG.y),
                                                       AltitudeReferenceTypes::REF_ALT_RELATIVE,
                                                       static_cast<double>(decodedMSG.z));

        mace::pose::Velocity_Cartesian3D localVelocity(CartesianFrameTypes::CF_BODY_NED);
        localVelocity.setXVelocity(static_cast<double>(decodedMSG.vx));
        localVelocity.setYVelocity(static_cast<double>(decodedMSG.vy));
        localVelocity.setZVelocity(static_cast<double>(decodedMSG.vz));

        Data::EnvironmentTime currentTime;
        Data::EnvironmentTime::CurrentTime(Data::Devices::SYSTEMCLOCK, currentTime);
        prevPosition = currentTime;

        if(state->vehicleLocalPosition.set(localPosition))
        {
            //Before publishing the topic we need to transform it
            localPosition.applyTransformation(state->getTransform_VehicleHomeTOSwarm());
            if(!state->shouldTransformLocalAltitude())
                localPosition.setAltitude(decodedMSG.z);

            std::shared_ptr<mace::pose_topics::Topic_CartesianPosition> ptrPosition = std::make_shared<mace::pose_topics::Topic_CartesianPosition>(&localPosition);
            if(this->m_CB)
                this->m_CB->cbi_VehicleStateData(systemID,ptrPosition);
        }

        if(state->vehicleLocalVelocity.set(localVelocity))
        {
            std::shared_ptr<mace::pose_topics::Topic_CartesianVelocity> ptrVelocity = std::make_shared<mace::pose_topics::Topic_CartesianVelocity>(&localVelocity);
            if(this->m_CB)
                this->m_CB->cbi_VehicleStateData(systemID,ptrVelocity);
        }
        break;
    }
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
    {
        //This is message definition 33
        //The filtered global position (e.g. fused GPS and accelerometers). The position is in GPS-frame (right-handed, Z-up). It is designed as scaled integer message since the resolution of float is not sufficient.
        mavlink_global_position_int_t decodedMSG;
        mavlink_msg_global_position_int_decode(msg,&decodedMSG);
        double power = pow(10,7);

        mace::pose::GeodeticPosition_3D globalPosition(GeodeticFrameTypes::CF_GLOBAL_RELATIVE_ALT,
                                                       static_cast<double>(decodedMSG.lat/power),
                                                       static_cast<double>(decodedMSG.lon/power),
                                                       AltitudeReferenceTypes::REF_ALT_RELATIVE,
                                                       static_cast<double>(decodedMSG.relative_alt/1000.0), "Agent Position");

        //check that something has actually changed
        if(state->vehicleGlobalPosition.set(globalPosition))
        {
            std::shared_ptr<mace::pose_topics::Topic_GeodeticPosition> ptrPosition = std::make_shared<mace::pose_topics::Topic_GeodeticPosition>(&globalPosition);
            if(this->m_CB)
                this->m_CB->cbi_VehicleStateData(systemID,ptrPosition);
        }
        break;
    }
    case MAVLINK_MSG_ID_VFR_HUD:
    {
        //This is message definition 74
        //Metrics typically displayed on a HUD for fixed wing aircraft
        mavlink_vfr_hud_t decodedMSG;
        mavlink_msg_vfr_hud_decode(msg,&decodedMSG);

        mace::measurements::Speed currentAirspeed;
        currentAirspeed.setSpeed(static_cast<double>(decodedMSG.airspeed));
        currentAirspeed.updateSpeedType(mace::measurements::Speed::SpeedTypes::AIRSPEED);
        if(state->vehicleAirspeed.set(currentAirspeed))
        {
            mace::measurement_topics::Topic_AirSpeedPtr ptrAirspeedTopic = std::make_shared<mace::measurement_topics::Topic_AirSpeed>(currentAirspeed);
            if(this->m_CB)
                this->m_CB->cbi_VehicleStateData(systemID,ptrAirspeedTopic);
        }

        mace::measurements::Speed currentGroundSpeed;
        currentGroundSpeed.setSpeed(static_cast<double>(decodedMSG.groundspeed));
        currentGroundSpeed.updateSpeedType(mace::measurements::Speed::SpeedTypes::GROUNDSPEED);
        if(state->vehicleGroundSpeed.set(currentGroundSpeed))
        {
            mace::measurement_topics::Topic_GroundSpeedPtr ptrGroundspeedTopic = std::make_shared<mace::measurement_topics::Topic_GroundSpeed>(currentGroundSpeed);
            if(this->m_CB)
                this->m_CB->cbi_VehicleStateData(systemID,ptrGroundspeedTopic);
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

        using namespace DataGenericItem;
        DataGenericItem::DataGenericItem_Text statusText;
        statusText.setText(decodedMSG.text);
        switch (decodedMSG.severity) {
        case MAV_SEVERITY_EMERGENCY:
            statusText.setSeverity(DataGenericItem_Text::STATUS_SEVERITY::STATUS_EMERGENCY);
            break;
        case MAV_SEVERITY_ALERT:
            statusText.setSeverity(DataGenericItem_Text::STATUS_SEVERITY::STATUS_ALERT);
            break;
        case MAV_SEVERITY_CRITICAL:
            statusText.setSeverity(DataGenericItem_Text::STATUS_SEVERITY::STATUS_CRITICAL);
            break;
        case MAV_SEVERITY_ERROR:
            statusText.setSeverity(DataGenericItem_Text::STATUS_SEVERITY::STATUS_ERROR);
            break;
        case MAV_SEVERITY_WARNING:
            statusText.setSeverity(DataGenericItem_Text::STATUS_SEVERITY::STATUS_WARNING);
            break;
        case MAV_SEVERITY_NOTICE:
            statusText.setSeverity(DataGenericItem_Text::STATUS_SEVERITY::STATUS_NOTICE);
            break;
        case MAV_SEVERITY_INFO:
            statusText.setSeverity(DataGenericItem_Text::STATUS_SEVERITY::STATUS_INFO);
            break;
        case MAV_SEVERITY_DEBUG:
            statusText.setSeverity(DataGenericItem_Text::STATUS_SEVERITY::STATUS_DEBUG);
            break;
        default:
            break;
        }

        state->vehicleTextAlert.set(statusText);

        std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_Text> ptrStatusText = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_Text>(statusText);
        if(this->m_CB)
            this->m_CB->cbi_VehicleStateData(systemID,ptrStatusText);
        break;
    }

        /////////////////////////////////////////////////////////////////////////
        /// MISSION ITEMS: The following case statements are executed
        /// for mission based message events.
        /////////////////////////////////////////////////////////////////////////
    case MAVLINK_MSG_ID_MISSION_CURRENT:
    {
        //This is message definition 42
        //Message that announces the sequence number of the current active mission item. The MAV will fly towards this mission item.
        mavlink_mission_current_t decodedMSG;
        mavlink_msg_mission_current_decode(msg, &decodedMSG);

        // Check for a mission first:
        //        if(mission->currentAutoMission.hasBeenSet()) {
        //            MissionItem::MissionItemCurrent current;
        //            current.setMissionKey(mission->currentAutoMission.get().getMissionKey());
        //            current.setMissionCurrentIndex(decodedMSG.seq);

        //            if(mission->missionItemCurrent.set(current))
        //            {
        //                if(decodedMSG.seq == 0)
        //                {
        //                    //the current target is home and we should handle this differently
        //                }
        //                else{
        //                    int currentIndex = decodedMSG.seq - 1;

        //                    MissionItem::MissionItemCurrent current;
        //                    current.setMissionKey(mission->currentAutoMission.get().getMissionKey());
        //                    current.setMissionCurrentIndex(currentIndex);

        //                    if(this->m_CB)
        //                        m_CB->cbi_VehicleMissionItemCurrent(current);
        //                }
        //            }
        //        }

        break;
    }
    case MAVLINK_MSG_ID_MISSION_ITEM_REACHED:
    {
        //This is message definition 46
        //A certain mission item has been reached. The system will either hold this position (or circle on the orbit) or
        //(if the autocontinue on the WP was set) continue to the next MISSION.
        mavlink_mission_item_reached_t decodedMSG;
        mavlink_msg_mission_item_reached_decode(msg,&decodedMSG);
        //        int missionIndex = decodedMSG.seq - 1; //transforms the reference away from mavlink to MACE

        //        if(missionIndex >= 0)
        //        {
        //            MissionItem::MissionItemAchieved itemAchieved;
        //            itemAchieved.setMissionKey(mission->currentAutoMission.get().getMissionKey());
        //            itemAchieved.setMissionAchievedIndex(missionIndex);

        //            if(mission->missionItemReached.set(itemAchieved))
        //            {
        //                std::shared_ptr<MissionTopic::MissionItemReachedTopic> ptrMissionTopic = std::make_shared<MissionTopic::MissionItemReachedTopic>(itemAchieved);
        //                if(this->m_CB != nullptr)
        //                    m_CB->cbi_VehicleMissionData(systemID, ptrMissionTopic);
        //            }
        //        }
        //        else{
        //            //we have reached the home position, should we do anything here?
        //        }

        break;
    }
    case MAVLINK_MSG_ID_HOME_POSITION:
    {
        mavlink_home_position_t decodedMSG;
        mavlink_msg_home_position_decode(msg,&decodedMSG);

        double power = pow(10,7);

        mace::pose::GeodeticPosition_3D currentHome(decodedMSG.latitude / power,
                                                    decodedMSG.longitude / power,
                                                    decodedMSG.altitude / 1000.0);
        std::cout<<"The new vehicle home is: "<<currentHome<<std::endl;

        currentHome.setCoordinateFrame(GeodeticFrameTypes::CF_GLOBAL_AMSL);
        currentHome.setAltitudeReferenceFrame(AltitudeReferenceTypes::REF_ALT_MSL);

        //check that something has actually changed
        if(state->vehicleGlobalHome.set(currentHome))
        {
            command_item::SpatialHome spatialHome;
            spatialHome.setPosition(&currentHome);

            if(this->m_CB)
                this->m_CB->cbi_VehicleHome(systemID,spatialHome);
        }
        break;
    }
    case MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT:
    {
        mavlink_position_target_global_int_t decodedMSG;
        mavlink_msg_position_target_global_int_decode(msg,&decodedMSG);

        double power = pow(10,7);

        mace::pose::GeodeticPosition_3D targetPosition(GeodeticFrameTypes::CF_GLOBAL_RELATIVE_ALT,
                                                       static_cast<double>(decodedMSG.lat_int/power),
                                                       static_cast<double>(decodedMSG.lon_int/power),
                                                       AltitudeReferenceTypes::REF_ALT_RELATIVE,
                                                       static_cast<double>(decodedMSG.alt), "Target Position");

        MissionTopic::VehicleTargetTopic vehicleTarget(systemID, &targetPosition);
        std::shared_ptr<MissionTopic::VehicleTargetTopic> ptrMissionTopic = std::make_shared<MissionTopic::VehicleTargetTopic>(vehicleTarget);

        if(this->m_CB != nullptr)
            m_CB->cbi_VehicleMissionData(systemID, ptrMissionTopic);

        break;
    }
    case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED:
    {
        mavlink_position_target_local_ned_t decodedMSG;
        mavlink_msg_position_target_local_ned_decode(msg,&decodedMSG);
        std::cout<<"I have received a new local target in the coordinate frame of: "<<decodedMSG.coordinate_frame<<std::endl;
        if(!(decodedMSG.type_mask & IGNORE_POS_TYPE_MASK)) //means the position was to not be ignored
        {
//            mace::pose::CartesianPosition_3D targetPosition(CartesianFrameTypes::CF_GLOBAL_RELATIVE_ALT,
//                                                            static_cast<double>(decodedMSG.x),
//                                                            static_cast<double>(decodedMSG.y),
//                                                            AltitudeReferenceTypes::REF_ALT_RELATIVE,
//                                                            statiatleac_cast<double>(decodedMSG.z), "Target Cartesian Position");
        }



        break;
    }
    case MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN:
    {
        //This position is what the ardupilot uses to reference all of the local commands and position elements
        mavlink_gps_global_origin_t decodedMSG;
        mavlink_msg_gps_global_origin_decode(msg,&decodedMSG);
        double power = pow(10,7);

        mace::pose::GeodeticPosition_3D currentOrigin(decodedMSG.latitude / power,
                                                      decodedMSG.longitude / power,
                                                      decodedMSG.altitude / 1000.0);

        currentOrigin.setCoordinateFrame(GeodeticFrameTypes::CF_GLOBAL_AMSL);
        currentOrigin.setAltitudeReferenceFrame(AltitudeReferenceTypes::REF_ALT_MSL);
        state->vehicleGlobalOrigin.set(currentOrigin);

        break;
    }
    case MAVLINK_MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET:
    {
        mavlink_local_position_ned_system_global_offset_t decodedMSG;
        mavlink_msg_local_position_ned_system_global_offset_decode(msg,&decodedMSG);
        std::cout<<"This is an interesting one."<<std::endl;
        break;
    }

    case MAVLINK_MSG_ID_CONTROL_SYSTEM_STATE:
    {
        std::cout<<"I have seen a new control system state."<<std::endl;
        break;
    }
    default:
    {
        consumed = false;
    }
    } //end of switch statement
    return consumed;
}
