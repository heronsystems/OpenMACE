#ifndef CONTROLLER_MISSION_H
#define CONTROLLER_MISSION_H


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
    mace_message_t,
    MaceCore::ModuleCharacteristic,
    TransmitQueueWithKeys<MaceCore::ModuleCharacteristic, ObjectMaceMsgIDTuple<MaceCore::ModuleCharacteristic>, ObjectMaceMsgIDTuple<MissionItem::MissionKey>>,
    uint8_t,
    Controllers::DataItem<MaceCore::ModuleCharacteristic, MissionKey>,
    Controllers::DataItem<MissionKey, MissionList>
    >;





using SendHelper_RequestMissionDownload = Controllers::ActionSend<
    mace_message_t, MaceCore::ModuleCharacteristic,
    CONTROLLER_MISSION_TYPE,
    MissionItem::MissionKey,
    MissionItem::MissionKey,
    mace_mission_request_list_t,
    MACE_MSG_ID_MISSION_COUNT
>;



using SendHelper_RequestList = Controllers::ActionIntermediate<
    mace_message_t, MaceCore::ModuleCharacteristic,
    CONTROLLER_MISSION_TYPE,
    MissionItem::MissionKey,
    MissionItem::MissionKey,
    mace_mission_request_list_t,
    MACE_MSG_ID_MISSION_REQUEST_LIST,
    mace_mission_count_t,
    MACE_MSG_ID_MISSION_REQUEST_ITEM
>;



using SendHelper_ReceiveCountRespondItemRequest = Controllers::ActionIntermediate<
    mace_message_t, MaceCore::ModuleCharacteristic,
    CONTROLLER_MISSION_TYPE,
    MissionItem::MissionKey,
    MissionItem::MissionKey,
    mace_mission_count_t,
    MACE_MSG_ID_MISSION_COUNT,
    mace_mission_request_item_t,
    MACE_MSG_ID_MISSION_ITEM
>;



using SendHelper_RequestItem = Controllers::ActionIntermediate<
    mace_message_t, MaceCore::ModuleCharacteristic,
    CONTROLLER_MISSION_TYPE,
    MissionItem::MissionKey,
    MissionItem::MissionKey,
    mace_mission_request_item_t,
    MACE_MSG_ID_MISSION_REQUEST_ITEM,
    mace_mission_item_t,
    MACE_MSG_ID_MISSION_REQUEST_ITEM,
    MACE_MSG_ID_MISSION_ACK
>;



using SendHelper_ReceiveItem = Controllers::ActionIntermediateReceive<
    mace_message_t, MaceCore::ModuleCharacteristic,
    CONTROLLER_MISSION_TYPE,
    MissionItem::MissionKey,
    MissionItem::MissionKey,
    mace_mission_item_t,
    MACE_MSG_ID_MISSION_ITEM,
    mace_mission_request_item_t
>;

/*
 * Duplicate
typedef Controllers::ActionResponseIntermediate<
        CONTROLLER_MISSION_TYPE,
        mace_mission_request_item_t,
        MACE_MSG_ID_MISSION_ITEM
    >
    SendHelper_RespondItem;
*/




using SendHelper_Final = Controllers::ActionFinalReceiveRespond<
    mace_message_t, MaceCore::ModuleCharacteristic,
    CONTROLLER_MISSION_TYPE,
    MissionItem::MissionKey,
    MissionItem::MissionKey,
    MissionItem::MissionList,
    mace_mission_item_t,
    mace_mission_ack_t,
    MACE_MSG_ID_MISSION_ITEM
>;



using SendHelper_FinalFinal = Controllers::ActionFinish<
    mace_message_t, MaceCore::ModuleCharacteristic,
    CONTROLLER_MISSION_TYPE,
    MissionItem::MissionKey,
    uint8_t,
    mace_mission_ack_t,
    MACE_MSG_ID_MISSION_ACK
>;







using Action_RequestCurrentMission_Initiate = Controllers::ActionRequest<
    mace_message_t, MaceCore::ModuleCharacteristic,
    CONTROLLER_MISSION_TYPE,
    MaceCore::ModuleCharacteristic,
    mace_mission_request_list_generic_t,
    MACE_MSG_ID_MISSION_COUNT,
    MACE_MSG_ID_MISSION_ACK
>;



using Action_RequestCurrentMission_Response = Controllers::ActionIntermediateReceive<
    mace_message_t, MaceCore::ModuleCharacteristic,
    CONTROLLER_MISSION_TYPE,
    MissionItem::MissionKey,
    MissionItem::MissionKey,
    mace_mission_request_list_generic_t,
    MACE_MSG_ID_MISSION_REQUEST_LIST_GENERIC,
    mace_mission_count_t
>;


using Action_RequestCurrentMission_NoMissionResponse = Controllers::ActionIntermediateReceive<
    mace_message_t, MaceCore::ModuleCharacteristic,
    CONTROLLER_MISSION_TYPE,
    MissionItem::MissionKey,
    MissionItem::MissionKey,
    mace_mission_request_list_generic_t,
    MACE_MSG_ID_MISSION_REQUEST_LIST_GENERIC,
    mace_mission_ack_t
>;









using NotifyRemoteOfMission = Controllers::ActionSend<
    mace_message_t, MaceCore::ModuleCharacteristic,
    CONTROLLER_MISSION_TYPE,
    MaceCore::ModuleCharacteristic,
    MissionItem::MissionKey,
    mace_new_onboard_mission_t,
    MACE_MSG_ID_MISSION_ACK
>;


using UsolicitedReceiveMissionNotification = Controllers::ActionUnsolicitedReceiveRespond<
    mace_message_t, MaceCore::ModuleCharacteristic,
    CONTROLLER_MISSION_TYPE,
    MaceCore::ModuleCharacteristic,
    MissionItem::MissionKey,
    mace_new_onboard_mission_t,
    mace_mission_ack_t,
    MACE_MSG_ID_NEW_ONBOARD_MISSION
>;

using NotifyRemoteOfMissionFinish = Controllers::ActionFinish<
    mace_message_t, MaceCore::ModuleCharacteristic,
    CONTROLLER_MISSION_TYPE,
    MaceCore::ModuleCharacteristic,
    uint8_t,
    mace_mission_ack_t,
    MACE_MSG_ID_MISSION_ACK
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
    virtual bool Construct_Send(const MissionItem::MissionKey &data, const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target, mace_mission_request_list_t &cmd, MissionItem::MissionKey &queueObj);

    virtual bool BuildData_Send(const mace_mission_request_list_t &cmd, const MaceCore::ModuleCharacteristic &sender, mace_mission_count_t &rtn, MaceCore::ModuleCharacteristic &vehicleObj, MissionItem::MissionKey &receiveQueueObj, MissionItem::MissionKey &respondQueueObj);


    virtual bool BuildData_Send(const mace_mission_count_t &mission, const MaceCore::ModuleCharacteristic &sender, mace_mission_request_item_t &request, MaceCore::ModuleCharacteristic &moduleFor, MissionItem::MissionKey &receiveQueueObj, MissionItem::MissionKey &respondQueueObj);


    virtual bool BuildData_Send(const mace_mission_request_item_t &missionRequest, const MaceCore::ModuleCharacteristic &sender, mace_mission_item_t &missionItem, MaceCore::ModuleCharacteristic &moduleFor, MissionItem::MissionKey &receiveQueueObj, MissionItem::MissionKey &respondQueueObj);


    virtual bool BuildData_Send(const mace_mission_item_t &missionItem, const MaceCore::ModuleCharacteristic &sender, mace_mission_request_item_t &request, MaceCore::ModuleCharacteristic &moduleFor, MissionItem::MissionKey &receiveQueueObj, MissionItem::MissionKey &respondQueueObj);





    virtual bool Construct_FinalObjectAndResponse(const mace_mission_item_t &missionItem, const MaceCore::ModuleCharacteristic &sender, mace_mission_ack_t &ackMission, MissionKey &finalKey, MissionList &finalList, MaceCore::ModuleCharacteristic &vehicleObj, MissionItem::MissionKey &queueObj);

    virtual bool Finish_Receive(const mace_mission_ack_t &missionItem, const MaceCore::ModuleCharacteristic &sender, uint8_t & ack, MissionItem::MissionKey &queueObj);


    virtual void Request_Construct(const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target, mace_mission_request_list_generic_t &msg, MaceCore::ModuleCharacteristic &queueObj);

    virtual bool BuildData_Send(const mace_mission_request_list_generic_t &msg, const MaceCore::ModuleCharacteristic &sender, mace_mission_count_t &response, MaceCore::ModuleCharacteristic &vehicleObj, MissionItem::MissionKey &receiveQueueObj, MissionItem::MissionKey &responseQueueObj);


    virtual bool BuildData_Send(const mace_mission_request_list_generic_t &msg, const MaceCore::ModuleCharacteristic &sender, mace_mission_ack_t &response, MaceCore::ModuleCharacteristic &vehicleObj, MissionItem::MissionKey &receiveQueueObj, MissionItem::MissionKey &respondQueueObj);





    virtual bool Construct_Send(const MissionItem::MissionKey &data, const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target, mace_new_onboard_mission_t &msg, MaceCore::ModuleCharacteristic &queue);


    virtual bool Construct_FinalObjectAndResponse(const mace_new_onboard_mission_t &msg, const MaceCore::ModuleCharacteristic &sender, mace_mission_ack_t &ack, MaceCore::ModuleCharacteristic &module_from, MaceCore::ModuleCharacteristic &dataKey, MissionItem::MissionKey &data);

    virtual bool Finish_Receive(const mace_mission_ack_t &missionItem, const MaceCore::ModuleCharacteristic &sender, uint8_t & ack, MaceCore::ModuleCharacteristic &queueObj);



public:

    ControllerMission(const Controllers::IMessageNotifier<mace_message_t, MaceCore::ModuleCharacteristic> *cb, TransmitQueue *queue, int linkChan);


    void RequestMission(const MissionItem::MissionKey &key, const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target);

    void RequestCurrentMission(const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target);


    void NotifyOfMission(const MissionItem::MissionKey &key, const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target);
};

}

#endif // CONTROLLER_MISSION_H
