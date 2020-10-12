

#ifndef CONTROLLER_PARAMETER_REQUEST_H
#define CONTROLLER_PARAMETER_REQUEST_H

#include <iostream>
#include <string.h>
#include "common/common.h"

#include "controllers/actions/action_send.h"
#include "controllers/actions/action_finish.h"

#include "mavlink.h"
#include "module_vehicle_MAVLINK/mavlink_entity_key.h"
#include "module_vehicle_MAVLINK/controllers/common.h"

namespace MAVLINKUXVControllers {

struct ParameterRequestResult
{
    uint8_t vehicleID = 0;

    int parameterIndex = -1;
    std::string parameterName = "";
    double parameterValue = 0.0;
};

using ParameterRequestSend = Controllers::ActionSend<
    mavlink_message_t,
    MavlinkEntityKey,
    BasicMavlinkController_ModuleKeyed<ParameterRequestResult>,
    MavlinkEntityKey,
    ParameterRequestResult,
    mavlink_param_request_read_t,
    MAVLINK_MSG_ID_PARAM_VALUE
>;

using ParameterRequestFinish = Controllers::ActionFinish<
    mavlink_message_t,
    MavlinkEntityKey,
    BasicMavlinkController_ModuleKeyed<ParameterRequestResult>,
    MavlinkEntityKey,
    ParameterRequestResult,
    mavlink_param_value_t,
    MAVLINK_MSG_ID_PARAM_VALUE
>;

class Controller_ParameterRequest : public BasicMavlinkController_ModuleKeyed<ParameterRequestResult>,
        public ParameterRequestSend,
        public ParameterRequestFinish
{

protected:


    virtual bool Construct_Send(const ParameterRequestResult &userRequest, const MavlinkEntityKey &sender, const MavlinkEntityKey &target, mavlink_param_request_read_t &request, MavlinkEntityKey &queueObj)
    {
        UNUSED(sender);
        queueObj = target;

        if((userRequest.parameterIndex == -1) && (!userRequest.parameterName.empty())) //populate using the string
        {
                char *target;
                if(userRequest.parameterName.length() < 16)
                    target = static_cast<char*>(malloc((1 + userRequest.parameterName.length())));
                else if(userRequest.parameterName.length() == 16)
                    target = static_cast<char*>(malloc(16));
                else
                    return false; //we shouldnt have made this request

                if (target == nullptr) /* check that nothing special happened to prevent tragedy  */
                    return false;
                strcpy(target, request.param_id);
                free(target);
        }
        else if(userRequest.parameterIndex >=0)
        {
            request.param_index = static_cast<int16_t>(userRequest.parameterIndex);
        }
        else //there is no way to process this request
        {
            return false;
        }

        request.target_system = userRequest.vehicleID;
        request.target_component = 0;

        return true;
    }


    virtual bool Finish_Receive(const mavlink_param_value_t &msg, const MavlinkEntityKey &sender, ParameterRequestResult &result, MavlinkEntityKey &queueObj)
    {
        UNUSED(sender);
        UNUSED(queueObj);
        std::cout<<"The parameter value received is: "<<msg.param_value<<std::endl;
        result.parameterValue = static_cast<double>(msg.param_value);
        return true;
    }

public:

    Controller_ParameterRequest(const Controllers::IMessageNotifier<mavlink_message_t, MavlinkEntityKey> *cb, TransmitQueue *queue, int linkChan) :
        BasicMavlinkController_ModuleKeyed<ParameterRequestResult>(cb, queue, linkChan),
        ParameterRequestSend(this, MavlinkEntityKeyToSysIDCompIDConverter<mavlink_param_request_read_t>(mavlink_msg_param_request_read_encode_chan)),
        ParameterRequestFinish(this, mavlink_msg_param_value_decode)
    {

    }

    virtual ~Controller_ParameterRequest() = default;

    void RequestParameterValue(const MavlinkEntityKey &vehicle, const std::string &parameterName)
    {
        ParameterRequestResult currentRequest;
        currentRequest.vehicleID = vehicle;
        currentRequest.parameterName = parameterName;
        ParameterRequestSend::ActionSend(currentRequest, vehicle, vehicle);
    }
};

} //end of namespace MAVLINKVehicleControllers

#endif // CONTROLLER_PARAMETER_REQUEST_H


//#include "controllers/generic_controller.h"

//#include "controllers/actions/action_send.h"
//#include "controllers/actions/action_intermediate_respond.h"
//#include "controllers/actions/action_intermediate_receive.h"
//#include "controllers/actions/action_intermediate.h"
//#include "controllers/actions/action_final_receive_respond.h"
//#include "controllers/actions/action_finish.h"
//#include "controllers/actions/action_request.h"
//#include "controllers/actions/action_unsolicited_receive.h"

//#include "mavlink.h"

//#include "data_interface_MAVLINK/MAVLINK_to_MACE/helper_mission_mavlink_to_mace.h"
//#include "data_interface_MAVLINK/MACE_to_MAVLINK/helper_mission_mace_to_mavlink.h"

//#include "module_vehicle_MAVLINK/mavlink_entity_key.h"
//#include "module_vehicle_MAVLINK/controllers/common.h"

//namespace MAVLINKUXVControllers {

///// Data that is to be downloaded within the parameter list
//using ParameterDownloadResult = std::map<std::string, double>;

///**
// * Definition of the controller that will be used for the paramters
// *
// * [Param1]
// * The controller for mission transmission on ardupilot will communicate using mavlink_message_t messages.
// *
// * [Param2]
// * When request are given to the queue they can be removed by two actions:
// * Expected message type and nothing else (ObjectIntTuple<void*>)
// * Expected message type and mission Key (ObjectIntTuple<MissionItem::MissionKey>)
// *
// * [Param3]
// * When finished a code of type uint8_t will be returned
// *
// * [Param4]
// * The controller can return data of the form MissionDownloadResult with no key
// * No key is needed because we always attached to a single vehicle in this module.
// */
//using CONTROLLER_PARAMETER_TYPE = Controllers::GenericController<
//    mavlink_message_t,
//    MavlinkEntityKey,
//    TransmitQueueWithKeys<MavlinkEntityKey, ObjectIntTuple<void*>, ObjectIntTuple<MissionItem::MissionKey>>,
//    uint8_t,
//    Controllers::DataItem<MavlinkEntityKey, ParameterDownloadResult>
//>;


///**
// * Action that requests a list of the full parameter list on the vehicle
// *
// * This action transmitts a mavlink_parameter_request_list_t
// * Transmission will stop when a MAVLINK_MSG_ID_MISSION_COUNT or MAVLINK_MSG_ID_MISSION_ACK is received.
// * The ACK will be received if there is no mission on the vehicle.
// */
//using ParameterAction_RequestFullList_Initiate = Controllers::ActionRequest<
//    mavlink_message_t,
//    MavlinkEntityKey,
//    CONTROLLER_PARAMETER_TYPE,
//    void*, //there is no parameterization of the request
//    mavlink_param_request_list_t,
//    MAVLINK_MSG_ID_PARAM_VALUE
//>;



///**
// * Action to receive an empty count and finish.
// *
// * This action will only trigger if count received is zero.
// * i.e. there is no parameters on vehicle.
// */
//using ParameterAction_ReceiveEmptyParameters = Controllers::ActionFinish<
//    mavlink_message_t,
//    MavlinkEntityKey,
//    CONTROLLER_PARAMETER_TYPE,
//    void*,
//    uint8_t,
//    mavlink_param_value_t,
//    MAVLINK_MSG_ID_PARAM_VALUE
//>;

///**
// * Action to receive a parameter item and make follow up request for next parameter item.
// */
//using ParameterAction_ReceiveParameterFollowUpNextParameterRequest = Controllers::ActionIntermediate<
//    mavlink_message_t,
//    MavlinkEntityKey,
//    CONTROLLER_PARAMETER_TYPE,
//    void*,
//    void*,
//    mavlink_param_value_t,
//    MAVLINK_MSG_ID_PARAM_VALUE,
//    mavlink_mission_request_t,
//    MAVLINK_MSG_ID_PARAM_VALUE
//>;


///**
// * Action to receive the final mission item and respond with an ACK to signal everything is done
// */
//using MissionAction_ReceiveFinalItem = Controllers::ActionFinalReceiveRespond<
//    mavlink_message_t,
//    MavlinkEntityKey,
//    CONTROLLER_PARAMETER_TYPE,
//    void*,
//    MavlinkEntityKey,
//    MissionDownloadResult,
//    mavlink_mission_item_t,
//    mavlink_mission_ack_t,
//    MAVLINK_MSG_ID_MISSION_ITEM
//>;

//class ControllerMission : public CONTROLLER_MISSION_TYPE,
//        public MissionAction_RequestCurrentMission_Initiate,
//        public MissionAction_ReceiveAckDueToEmptyMission,
//        public MissionAction_ReceiveCountRespondItemRequest,
//        public MissionAction_ReceiveItemFollowUpNextItemRequest,
//        public MissionAction_ReceiveFinalItem,

//        public MissionAction_Upload_Initiate,
//        public MissionAction_Upload_ReceiveRequestSendItem,
//        public MissionAction_Upload_Finish
//{

//private:

//    /// Number of items in the mission being downloaded. Set to -1 if no mission is being downloaded.
//    int m_MissionDownloadCount;

//    std::shared_ptr<MissionDownloadResult> m_MissionDownloading;


//    std::shared_ptr<MissionDownloadResult> m_MissionUploading;

//protected:


//    /**
//     * @brief Build request for mission list
//     * @param sender Unused for this module
//     * @param target Vehicle sending to
//     * @param msg Message to send to vehicle
//     * @param queue Unused for this module, set to zero
//     */
//    void Request_Construct(const MavlinkEntityKey &sender, const MavlinkEntityKey &target, mavlink_mission_request_list_t &msg, void* &queue)
//    {
//        msg.mission_type = 0;
//        msg.target_system = target;
//        msg.target_component = 0;
//        queue = 0;

//        m_MissionDownloading = std::make_shared<std::tuple<command_item::SpatialHome, MissionItem::MissionList>>();
//    }


//    /**
//     * @brief From a mission_count message determine if this is the final transmission.
//     *
//     * In this instance it is only the final transmission if count=0.
//     * i.e. no mission exists on the vehicle.
//     *
//     * @param msg Message received from vehicle
//     * @param sender Vehicle that sent message
//     * @param ack Ack to pass up should this be the final transmission
//     * @param queueObj Unused for this module, set to zero
//     * @return True if count=0 and we are to stop download
//     */
//    virtual bool Finish_Receive(const mavlink_mission_count_t &msg, const MavlinkEntityKey &sender, uint8_t& ack, void* &queueObj)
//    {

//        if(msg.count == 0)
//        {
//            queueObj = 0;
//            ack = -1;

//            return true;
//        }
//        return false;
//    }


//    /**
//     * @brief From a mission_count message determine the next request
//     *
//     * In this instance a follow up request will only be made if count > 1.
//     *
//     * @param msg Message received from vehicle
//     * @param sender Vehicle that sent message
//     * @param cmd Follow up command that can be made
//     * @param vehicleObj Target of followup command
//     * @param receiveQueueObj Unused for this module, set to zero
//     * @param respondQueueObj Unused for this module, set to zero
//     * @return
//     */
//    virtual bool BuildData_Send(const mavlink_mission_count_t &msg, const MavlinkEntityKey &sender, mavlink_mission_request_t &cmd, MavlinkEntityKey &vehicleObj, void* &receiveQueueObj, void* &respondQueueObj)
//    {
//        if(msg.count > 0)
//        {
//            cmd.target_system = msg.target_system;
//            cmd.target_component = msg.target_component;
//            cmd.mission_type = msg.mission_type;
//            cmd.seq = 0;

//            vehicleObj = sender;

//            receiveQueueObj = 0;
//            respondQueueObj = 0;

//            m_MissionDownloadCount = msg.count;
//            std::get<1>(*m_MissionDownloading).initializeQueue(msg.count - 1); // minus one because first item is home

//            return true;
//        }

//        return false;
//    }


//    /**
//     * @brief Method to receive/determine if still transmitting mission items.
//     *
//     * This method is to receive a mission_item, determine if there are more items to request and construct that request
//     *
//     * @param msg Mission Item received
//     * @param sender Vehicle that sent message
//     * @param cmd mission_request to send out if there are more items to request
//     * @param vehicleObj Target of followup command
//     * @param receiveQueueObj Unused for this module, set to zero
//     * @param respondQueueObj Unused for this module, set to zero
//     * @return True if a followup request is to be made. False if no request should be made
//     */
//    virtual bool BuildData_Send(const mavlink_mission_item_t &msg, const MavlinkEntityKey &sender, mavlink_mission_request_t &cmd, MavlinkEntityKey &vehicleObj, void* &receiveQueueObj, void* &respondQueueObj)
//    {
//        //check if activly downloading a mission.
//        if(m_MissionDownloadCount == -1)
//        {
//            return false;
//        }

//        if(msg.seq < m_MissionDownloadCount-1)
//        {
//            ReceiveMissionItem(msg, sender);

//            cmd.target_system = msg.target_system;
//            cmd.target_component = msg.target_component;
//            cmd.mission_type = msg.mission_type;
//            cmd.seq = msg.seq + 1;

//            vehicleObj = sender;

//            receiveQueueObj = 0;
//            respondQueueObj = 0;

//            return true;
//        }

//        return false;
//    }


//    /**
//     * @brief Method to receive the final mission_item
//     *
//     * This method is to receive a mission_item, determine if it is the last item and construct a mission_ack to send back.
//     * Data will also be set indicating MissionList/Home position that was downloaded from vehicle. This is sent back by setting a pointer passed to this function.
//     * The Action will then call onDataReceived function with that data.
//     *
//     * @param msg Mission Item received
//     * @param sender Vehicle that sent message
//     * @param cmd mission_ack that is to be sent out if last item was received
//     * @param data Mission/Home data that was collected during the mission download process.
//     * @param vehicleObj Unused for this module, set to zero
//     * @param queueObj Unused for this module, set to zero
//     * @return True if item received as last item.
//     */
//    virtual bool Construct_FinalObjectAndResponse(const mavlink_mission_item_t &msg, const MavlinkEntityKey &sender, mavlink_mission_ack_t &cmd, MavlinkEntityKey &key, MissionDownloadResult &data, MavlinkEntityKey &vehicleObj, void* &queueObj)
//    {
//        if(msg.seq == m_MissionDownloadCount-1)
//        {
//            ReceiveMissionItem(msg, sender);

//            cmd.target_system = msg.target_system;
//            cmd.target_component = msg.target_component;
//            cmd.type = MAV_MISSION_ACCEPTED;
//            cmd.mission_type = msg.mission_type;

//            queueObj = 0;
//            vehicleObj = sender;


//            key = sender;
//            data = *m_MissionDownloading;

//            m_MissionDownloadCount = -1;
//            m_MissionDownloading = nullptr;

//            return true;
//        }

//        return false;
//    }


//    virtual bool Construct_Send(const MissionDownloadResult &data, const MavlinkEntityKey &sender, const MavlinkEntityKey &target, mavlink_mission_count_t &msg, void* &queue)
//    {
//        m_MissionUploading = std::make_shared<MissionDownloadResult>(data);

//        queue = 0;

//        msg.count = std::get<1>(data).getQueueSize() + 1;
//        msg.target_component = 0;
//        msg.target_system = target;
//        msg.mission_type = 0; //MAV_MISSION_TYPE_MISSION;

//        return true;
//    }


//    virtual bool BuildData_Send(const mavlink_mission_request_t &msg, const MavlinkEntityKey &sender, mavlink_mission_item_t &cmd, MavlinkEntityKey &vehicleObj, void* &receiveQueueObj, void* &respondQueueObj)
//    {
//        if(m_MissionUploading != nullptr)
//        {
//            int index = msg.seq;

//            if(index == 0) //the vehicle requested the home position
//            {
//                cmd = DataMAVLINK::Helper_MissionMACEtoMAVLINK::convertHome(std::get<0>(*m_MissionUploading));
//                cmd.target_system = msg.target_system;
//                cmd.target_component = msg.target_component;
//            }
//            else{
//                std::shared_ptr<command_item::AbstractCommandItem> ptrItem = std::get<1>(*m_MissionUploading).getMissionItem(index - 1);
//                DataMAVLINK::Helper_MissionMACEtoMAVLINK::MACEMissionToMAVLINKMission(ptrItem, index, cmd);
//            }

//            vehicleObj = sender;

//            receiveQueueObj = 0;
//            respondQueueObj = 0;



//            return true;
//        }
//        return false;
//    }

//    virtual bool Finish_Receive(const mavlink_mission_ack_t &msg, const MavlinkEntityKey &sender, uint8_t& ack, void* &queueObj)
//    {
//        m_MissionUploading = nullptr;

//        queueObj = 0;

//        return true;
//    }

//public:

//    ControllerMission(const Controllers::IMessageNotifier<mavlink_message_t, MavlinkEntityKey> *cb, TransmitQueue *queue, int linkChan) :
//        CONTROLLER_MISSION_TYPE(cb, queue, linkChan),
//        MissionAction_RequestCurrentMission_Initiate(this, MavlinkEntityKeyToSysIDCompIDConverter<mavlink_mission_request_list_t>(mavlink_msg_mission_request_list_encode_chan)),
//        MissionAction_ReceiveAckDueToEmptyMission(this, mavlink_msg_mission_count_decode),
//        MissionAction_ReceiveCountRespondItemRequest(this, mavlink_msg_mission_count_decode, MavlinkEntityKeyToSysIDCompIDConverter<mavlink_mission_request_t>(mavlink_msg_mission_request_encode_chan)),
//        MissionAction_ReceiveItemFollowUpNextItemRequest(this, mavlink_msg_mission_item_decode, MavlinkEntityKeyToSysIDCompIDConverter<mavlink_mission_request_t>(mavlink_msg_mission_request_encode_chan)),
//        MissionAction_ReceiveFinalItem(this, mavlink_msg_mission_item_decode, MavlinkEntityKeyToSysIDCompIDConverter<mavlink_mission_ack_t>(mavlink_msg_mission_ack_encode_chan)),

//        MissionAction_Upload_Initiate(this, MavlinkEntityKeyToSysIDCompIDConverter<mavlink_mission_count_t>(mavlink_msg_mission_count_encode_chan)),
//        MissionAction_Upload_ReceiveRequestSendItem(this, mavlink_msg_mission_request_decode, MavlinkEntityKeyToSysIDCompIDConverter<mavlink_mission_item_t>(mavlink_msg_mission_item_encode_chan)),
//        MissionAction_Upload_Finish(this, mavlink_msg_mission_ack_decode)
//    {
//        m_MissionDownloadCount = -1;
//    }

//    virtual ~ControllerMission() = default;


//    void GetMissions(const MavlinkEntityKey &vehicle)
//    {
//        MissionAction_RequestCurrentMission_Initiate::Request(vehicle, vehicle);
//    }

//    void UploadMission(const MissionItem::MissionList &mission, const command_item::SpatialHome &home, const MavlinkEntityKey &vehicle)
//    {
//        MissionDownloadResult homeMissionPair = std::make_tuple(home, mission);
//        MissionAction_Upload_Initiate::Send(homeMissionPair, vehicle, vehicle);
//    }



//private:

//    /**
//     * @brief Process Paramter Received
//     * @param msg Mission Item
//     * @param sender Vehicle that sent mission item
//     */
//    void ReceiveVehicleParameter(const mavlink_mission_item_t &msg, const MavlinkEntityKey &sender)
//    {
//        if(msg.seq == 0)
//        {
//            //Mission exchanges are always going to exist on a geodetic frame and eventually we should check this Ken
//            mace::pose::GeodeticPosition_3D homePosition;
//            homePosition.setCoordinateFrame(GeodeticFrameTypes::CF_GLOBAL_AMSL);
//            homePosition.setAltitudeReferenceFrame(AltitudeReferenceTypes::REF_ALT_MSL);
//            homePosition.setLatitude(static_cast<double>(msg.x));
//            homePosition.setLongitude(static_cast<double>(msg.y));
//            homePosition.setAltitude(static_cast<double>(msg.z));

//            std::get<0>(*m_MissionDownloading).setPosition(&homePosition);
//            std::get<0>(*m_MissionDownloading).setOriginatingSystem(sender);
//            std::get<0>(*m_MissionDownloading).setTargetSystem(sender);
//        }
//        else {
//            int adjustedIndex = msg.seq - 1; //we decrement 1 only here because ardupilot references home as 0 and we 0 index in our mission queue
//            std::shared_ptr<command_item::AbstractCommandItem> newMissionItem = DataMAVLINK::Helper_MissionMAVLINKtoMACE::Convert_MAVLINKTOMACE(sender, msg);
//            std::get<1>(*m_MissionDownloading).replaceMissionItemAtIndex(newMissionItem, adjustedIndex);
//        }
//    }




//};

//} //end of namespace MAVLINKVehicleControllers

