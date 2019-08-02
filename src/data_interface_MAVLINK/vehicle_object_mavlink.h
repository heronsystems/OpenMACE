#ifndef VEHICLE_OBJECT_MAVLINK_H
#define VEHICLE_OBJECT_MAVLINK_H

#include <chrono>
#include <thread>
#include <string>
#include <iostream>

#include "comms/comms_marshaler.h"

#include "command_interface_mavlink.h"

#include "callback_interface_data_mavlink.h"

#include "command_controller_mavlink.h"
#include "guided_controller_mavlink.h"
#include "mission_controller_mavlink.h"

#include "mission_data_mavlink.h"
#include "state_data_mavlink.h"

namespace DataInterface_MAVLINK{

class VehicleObject_MAVLINK : public CommandController_Interface, public GuidedController_Interface, public MissionController_Interface
{
public:

    void async_example(const std::string &loggingPath);

    VehicleObject_MAVLINK(const std::string &loggingPath, const int &vehicleID, const int &transmittingID);

    ~VehicleObject_MAVLINK();

    int getSystemID() const
    {
        return this->systemID;
    }

    void updateCommsInfo(Comms::CommsMarshaler *commsMarshaler, const std::string &linkName, const uint8_t &linkChan);

    void transmitMessage(const mavlink_message_t &msg);

    std::vector<std::shared_ptr<Data::ITopicComponentDataObject>> ParseForVehicleData(const mavlink_message_t* message);

    void parseMessage(const mavlink_message_t *msg);

    void connectCallback(CallbackInterface_DataMAVLINK *cb)
    {
        m_CB = cb;
    }

//The following establish the necessary callback routines

    //The following are as required from the command controller interface
private:
    void cbiCommandController_transmitCommand(const mavlink_command_int_t &cmd);

    void cbiCommandController_transmitCommand(const mavlink_command_long_t &cmd);

    void cbiCommandController_transmitNewMode(const mavlink_set_mode_t &mode);

    void cbiCommandController_CommandACK(const mavlink_command_ack_t &ack);

    //The following are as required from the guided controller interface
private:
    void cbiGuidedController_TransmitMissionItem(const mavlink_mission_item_t &item);

    //The following are as required from the mission controller interface
private:
    void cbiMissionController_TransmitMissionCount(const mavlink_mission_count_t &count);
    void cbiMissionController_TransmitMissionItem(const mavlink_mission_item_t &item);

    void cbiMissionController_TransmitMissionReqList(const mavlink_mission_request_list_t &request);
    void cbiMissionController_TransmitMissionReq(const mavlink_mission_request_t &requestItem);

    void cbiMissionController_ReceviedHome(const command_item::SpatialHome &home);
    void cbiMissionController_ReceivedMission(const MissionItem::MissionList &missionList);

    void cbiMissionController_MissionACK(const mavlink_mission_ack_t &missionACK, const MissionItem::MissionList &missionList);



public:
    static void staticCallbackCMDLongFunction(void *p, mavlink_command_long_t &cmd)
    {
        //((VehicleObject_MAVLINK *)p)->transmitCommandLong(cmd);
    }

    static void staticCallbackState(void *p, DataState::StateGlobalPosition &pos)
    {
        UNUSED(p);
        UNUSED(pos);
    }

    static void staticCallbackTransmitMissionMSG(void *p, mavlink_message_t &msg)
    {
        ((VehicleObject_MAVLINK *)p)->transmitMessage(msg);
    }

    static void staticCallbackTransmitMSG(void *p, mavlink_message_t &msg)
    {
        ((VehicleObject_MAVLINK *)p)->transmitMessage(msg);
    }
private:
    CallbackInterface_DataMAVLINK *m_CB;

    //The following are basic controllers for the MAVLINK vehicle object
public:
    CommandController_MAVLINK *m_CommandController;
    GuidedController_MAVLINK *m_GuidedController;
    MissionController_MAVLINK *m_MissionController;

    //The following are organizational methods to compartmentalize funcitonality
public:
    CommandInterface_MAVLINK *command;
    MissionData_MAVLINK *mission;
    StateData_MAVLINK *state;

    //The following are members describing important details of the vehicle object
private:
    int systemID;
    int commandID;

    //The following are members enabling communications with the vehicle object
private:
    Comms::CommsMarshaler *m_LinkMarshaler;
    std::string m_LinkName;
    uint8_t m_LinkChan;
};

} //end of namespace DataInterface_MAVLINK
#endif // VEHICLE_OBJECT_MAVLINK_H
