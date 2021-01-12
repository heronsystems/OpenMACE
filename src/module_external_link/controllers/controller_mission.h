#ifndef CONTROLLER_MISSION_H
#define CONTROLLER_MISSION_H

#include <mavlink.h>

#include "controllers/generic_controller.h"

#include "controllers/actions/action_send.h"
#include "controllers/actions/action_intermediate_respond.h"
#include "controllers/actions/action_intermediate_receive.h"
#include "controllers/actions/action_intermediate.h"
#include "controllers/actions/action_final_receive_respond.h"
#include "controllers/actions/action_finish.h"
#include "controllers/actions/action_request.h"
#include "controllers/actions/action_unsolicited_receive_respond.h"

#include "module_external_link/controllers/common.h"

#include "data_generic_mission_item_topic/mission_item_topic_components.h"

#include "data_generic_command_item/mission_items/mission_item_factory.h"
#include "data_generic_command_item/mission_items/mission_ack.h"

namespace ExternalLink {

using CONTROLLER_MISSION_TYPE = Controllers::GenericController<
    mavlink_message_t,
    MaceCore::ModuleCharacteristic,
    TransmitQueueWithKeys<MaceCore::ModuleCharacteristic, ObjectMaceMsgIDTuple<MaceCore::ModuleCharacteristic>, ObjectMaceMsgIDTuple<MissionItem::MissionKey>>,
    uint8_t,
    Controllers::DataItem<MaceCore::ModuleCharacteristic, MissionKey>,
    Controllers::DataItem<MissionKey, MissionList>
    >;

using SendHelper_RequestMissionDownload = Controllers::ActionSend<
    mavlink_message_t, MaceCore::ModuleCharacteristic,
    CONTROLLER_MISSION_TYPE,
    MissionItem::MissionKey,
    MissionItem::MissionKey,
    mavlink_mission_request_list_t,
    MAVLINK_MSG_ID_MISSION_COUNT
>;



using SendHelper_RequestList = Controllers::ActionIntermediate<
    mavlink_message_t, MaceCore::ModuleCharacteristic,
    CONTROLLER_MISSION_TYPE,
    MissionItem::MissionKey,
    MissionItem::MissionKey,
    mavlink_mission_request_list_t,
    MAVLINK_MSG_ID_MISSION_REQUEST_LIST,
    mavlink_mace_mission_count_t,
    MAVLINK_MSG_ID_MISSION_REQUEST_INT
>;



using SendHelper_ReceiveCountRespondItemRequest = Controllers::ActionIntermediate<
    mavlink_message_t, MaceCore::ModuleCharacteristic,
    CONTROLLER_MISSION_TYPE,
    MissionItem::MissionKey,
    MissionItem::MissionKey,
    mavlink_mace_mission_count_t,
    MAVLINK_MSG_ID_MISSION_COUNT,
    mavlink_mace_mission_request_int_t,
    MAVLINK_MSG_ID_MISSION_ITEM_INT
>;



using SendHelper_RequestItem = Controllers::ActionIntermediate<
    mavlink_message_t, MaceCore::ModuleCharacteristic,
    CONTROLLER_MISSION_TYPE,
    MissionItem::MissionKey,
    MissionItem::MissionKey,
    mavlink_mace_mission_request_int_t,
    MAVLINK_MSG_ID_MISSION_REQUEST_INT,
    mavlink_mace_mission_item_int_t,
    MAVLINK_MSG_ID_MISSION_REQUEST_INT,
    MAVLINK_MSG_ID_MACE_MISSION_ACK
>;


using SendHelper_ReceiveItem = Controllers::ActionIntermediateReceive<
    mavlink_message_t, MaceCore::ModuleCharacteristic,
    CONTROLLER_MISSION_TYPE,
    MissionItem::MissionKey,
    MissionItem::MissionKey,
    mavlink_mace_mission_item_int_t,
    MAVLINK_MSG_ID_MISSION_ITEM_INT,
    mavlink_mace_mission_request_int_t
>;


using SendHelper_Final = Controllers::ActionFinalReceiveRespond<
    mavlink_message_t, MaceCore::ModuleCharacteristic,
    CONTROLLER_MISSION_TYPE,
    MissionItem::MissionKey,
    MissionItem::MissionKey,
    MissionItem::MissionList,
    mavlink_mace_mission_item_int_t,
    mavlink_mace_mission_ack_t,
    MAVLINK_MSG_ID_MISSION_ITEM_INT
>;

using SendHelper_FinalFinal = Controllers::ActionFinish<
    mavlink_message_t, MaceCore::ModuleCharacteristic,
    CONTROLLER_MISSION_TYPE,
    MissionItem::MissionKey,
    uint8_t,
    mavlink_mace_mission_ack_t,
    MAVLINK_MSG_ID_MACE_MISSION_ACK
>;



using Action_RequestCurrentMission_Initiate = Controllers::ActionRequest<
    mavlink_message_t, MaceCore::ModuleCharacteristic,
    CONTROLLER_MISSION_TYPE,
    MaceCore::ModuleCharacteristic,
    mavlink_mission_request_list_t,
    MAVLINK_MSG_ID_MISSION_COUNT,
    MAVLINK_MSG_ID_MACE_MISSION_ACK
>;


using Action_RequestCurrentMission_Response = Controllers::ActionIntermediateReceive<
    mavlink_message_t, MaceCore::ModuleCharacteristic,
    CONTROLLER_MISSION_TYPE,
    MissionItem::MissionKey,
    MissionItem::MissionKey,
    mavlink_mission_request_list_t,
    MAVLINK_MSG_ID_MISSION_REQUEST_LIST,
    mavlink_mace_mission_count_t
>;


using Action_RequestCurrentMission_NoMissionResponse = Controllers::ActionIntermediateReceive<
    mavlink_message_t, MaceCore::ModuleCharacteristic,
    CONTROLLER_MISSION_TYPE,
    MissionItem::MissionKey,
    MissionItem::MissionKey,
    mavlink_mission_request_list_t,
    MAVLINK_MSG_ID_MISSION_REQUEST_LIST,
    mavlink_mace_mission_ack_t
>;




using NotifyRemoteOfMission = Controllers::ActionSend<
    mavlink_message_t, MaceCore::ModuleCharacteristic,
    CONTROLLER_MISSION_TYPE,
    MaceCore::ModuleCharacteristic,
    MissionItem::MissionKey,
    mavlink_new_onboard_mission_t,
    MAVLINK_MSG_ID_MACE_MISSION_ACK
>;


using UsolicitedReceiveMissionNotification = Controllers::ActionUnsolicitedReceiveRespond<
    mavlink_message_t, MaceCore::ModuleCharacteristic,
    CONTROLLER_MISSION_TYPE,
    MaceCore::ModuleCharacteristic,
    MissionItem::MissionKey,
    mavlink_new_onboard_mission_t,
    mavlink_mace_mission_ack_t,
    MAVLINK_MSG_ID_NEW_ONBOARD_MISSION
>;


using NotifyRemoteOfMissionFinish = Controllers::ActionFinish<
    mavlink_message_t, MaceCore::ModuleCharacteristic,
    CONTROLLER_MISSION_TYPE,
    MaceCore::ModuleCharacteristic,
    uint8_t,
    mavlink_mace_mission_ack_t,
    MAVLINK_MSG_ID_MACE_MISSION_ACK
>;



class ControllerMission : public CONTROLLER_MISSION_TYPE,
        public SendHelper_RequestMissionDownload,
        public SendHelper_RequestList,
        public SendHelper_ReceiveCountRespondItemRequest,
        public SendHelper_RequestItem,
        public SendHelper_ReceiveItem,
        public SendHelper_Final,
        public SendHelper_FinalFinal,
        public Action_RequestCurrentMission_Initiate,
        public Action_RequestCurrentMission_Response,
        public Action_RequestCurrentMission_NoMissionResponse,
        public NotifyRemoteOfMission,
        public UsolicitedReceiveMissionNotification,
        public NotifyRemoteOfMissionFinish
{

private:

    struct MissionRequestStruct
    {
        MissionItem::MissionList mission;
        MaceCore::ModuleCharacteristic requester;
    };

    OptionalParameter<MaceCore::ModuleCharacteristic> m_GenericRequester;
    std::unordered_map<MissionItem::MissionKey, MissionRequestStruct> m_MissionsBeingFetching;

    std::unordered_map<MaceCore::ModuleCharacteristic, std::unordered_map<MissionItem::MissionKey, MissionItem::MissionList>> m_MissionsUploading;

protected:

    //!
    //! \brief Called when building mavlink packet initial request to a mission
    //! \param data
    //! \param cmd
    //!
    virtual bool Construct_Send(const MissionItem::MissionKey &data, const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target, mavlink_mission_request_list_t &cmd, MissionItem::MissionKey &queueObj);

//    virtual bool BuildData_Send(const mavlink_mission_request_list_t &cmd, const MaceCore::ModuleCharacteristic &sender, mavlink_mace_mission_count_t &rtn, MaceCore::ModuleCharacteristic &vehicleObj, MissionItem::MissionKey &receiveQueueObj, MissionItem::MissionKey &respondQueueObj);


    virtual bool BuildData_Send(const mavlink_mace_mission_count_t &mission, const MaceCore::ModuleCharacteristic &sender, mavlink_mace_mission_request_int_t &request, MaceCore::ModuleCharacteristic &moduleFor, MissionItem::MissionKey &receiveQueueObj, MissionItem::MissionKey &respondQueueObj);


    virtual bool BuildData_Send(const mavlink_mace_mission_request_int_t &missionRequest, const MaceCore::ModuleCharacteristic &sender, mavlink_mace_mission_item_int_t &missionItem, MaceCore::ModuleCharacteristic &moduleFor, MissionItem::MissionKey &receiveQueueObj, MissionItem::MissionKey &respondQueueObj);


    virtual bool BuildData_Send(const mavlink_mace_mission_item_int_t &missionItem, const MaceCore::ModuleCharacteristic &sender, mavlink_mace_mission_request_int_t &request, MaceCore::ModuleCharacteristic &moduleFor, MissionItem::MissionKey &receiveQueueObj, MissionItem::MissionKey &respondQueueObj);



    virtual bool Construct_FinalObjectAndResponse(const mavlink_mace_mission_item_int_t &missionItem, const MaceCore::ModuleCharacteristic &sender, mavlink_mace_mission_ack_t &ackMission, MissionKey &finalKey, MissionList &finalList, MaceCore::ModuleCharacteristic &vehicleObj, MissionItem::MissionKey &queueObj);

    virtual bool Finish_Receive(const mavlink_mace_mission_ack_t &missionItem, const MaceCore::ModuleCharacteristic &sender, uint8_t & ack, MissionItem::MissionKey &queueObj);

    virtual void Request_Construct(const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target, mavlink_mission_request_list_t &msg, MaceCore::ModuleCharacteristic &queueObj);

    virtual bool BuildData_Send(const mavlink_mission_request_list_t &msg, const MaceCore::ModuleCharacteristic &sender, mavlink_mace_mission_count_t &response, MaceCore::ModuleCharacteristic &vehicleObj, MissionItem::MissionKey &receiveQueueObj, MissionItem::MissionKey &responseQueueObj);

    virtual bool BuildData_Send(const mavlink_mission_request_list_t &msg, const MaceCore::ModuleCharacteristic &sender, mavlink_mace_mission_ack_t &response, MaceCore::ModuleCharacteristic &vehicleObj, MissionItem::MissionKey &receiveQueueObj, MissionItem::MissionKey &respondQueueObj);



    virtual bool Construct_Send(const MissionItem::MissionKey &data, const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target, mavlink_new_onboard_mission_t &msg, MaceCore::ModuleCharacteristic &queue);

    virtual bool Construct_FinalObjectAndResponse(const mavlink_new_onboard_mission_t &msg, const MaceCore::ModuleCharacteristic &sender, mavlink_mace_mission_ack_t &ack, MaceCore::ModuleCharacteristic &module_from, MaceCore::ModuleCharacteristic &dataKey, MissionItem::MissionKey &data);

    virtual bool Finish_Receive(const mavlink_mace_mission_ack_t &missionItem, const MaceCore::ModuleCharacteristic &sender, uint8_t & ack, MaceCore::ModuleCharacteristic &queueObj);



public:

    ControllerMission(const Controllers::IMessageNotifier<mavlink_message_t, MaceCore::ModuleCharacteristic> *cb, TransmitQueue *queue, int linkChan);


    void RequestMission(const MissionItem::MissionKey &key, const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target);

    void RequestCurrentMission(const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target);


    void NotifyOfMission(const MissionItem::MissionKey &key, const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target);
};

}

#endif // CONTROLLER_MISSION_H
