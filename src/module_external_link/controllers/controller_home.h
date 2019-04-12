#ifndef CONTROLLER_HOME_H
#define CONTROLLER_HOME_H

#include "data_generic_command_item/spatial_items/spatial_home.h"

#include "controllers/generic_controller.h"

#include "controllers/actions/action_broadcast.h"
#include "controllers/actions/action_send.h"
#include "controllers/actions/action_final_receive_respond.h"
#include "controllers/actions/action_finish.h"
#include "controllers/actions/action_request.h"
#include "controllers/actions/action_intermediate_receive.h"
#include "controllers/actions/action_intermediate_respond.h"
#include "controllers/actions/action_intermediate.h"
#include "controllers/actions/action_unsolicited_receive.h"

#include "module_external_link/controllers/common.h"

namespace ExternalLink {


using CONTROLLER_HOME_TYPE = Controllers::GenericController<
    mace_message_t, MaceCore::ModuleCharacteristic,
    TransmitQueueWithKeys<MaceCore::ModuleCharacteristic, ObjectIntTuple<MaceCore::ModuleCharacteristic>>,
    uint8_t,
    Controllers::DataItem<MaceCore::ModuleCharacteristic, CommandItem::SpatialHome>
>;

//Broadcast a home position out, send and finish. (No waiting for response)
using ControllerHome_Step_BroadcastHome = Controllers::ActionBroadcast<
    mace_message_t, MaceCore::ModuleCharacteristic,
    CONTROLLER_HOME_TYPE,
    CommandItem::SpatialHome,
    mace_home_position_t
>;

//Receive a broadcasted home position, accept and finish (no response)
using ControllerHome_Step_ReceiveBroadcastedHome = Controllers::ActionUnsolicitedReceive<
    mace_message_t, MaceCore::ModuleCharacteristic,
    CONTROLLER_HOME_TYPE,
    MaceCore::ModuleCharacteristic,
    CommandItem::SpatialHome,
    mace_home_position_t,
    MACE_MSG_ID_HOME_POSITION
>;

//Request a home position, wait to receive the home position
using ControllerHome_Step_RequestHome = Controllers::ActionRequest<
    mace_message_t, MaceCore::ModuleCharacteristic,
    CONTROLLER_HOME_TYPE,
    MaceCore::ModuleCharacteristic,
    mace_mission_request_home_t,
    MACE_MSG_ID_HOME_POSITION
>;

//Receive a request for home, send out the home position, and wait to receive ack
using ControllerHome_Step_ReceiveHomeRequest = Controllers::ActionIntermediate<
    mace_message_t, MaceCore::ModuleCharacteristic,
    CONTROLLER_HOME_TYPE,
    MaceCore::ModuleCharacteristic,
    MaceCore::ModuleCharacteristic,
    mace_mission_request_home_t,
    MACE_MSG_ID_MISSION_REQUEST_HOME,
    mace_home_position_t,
    MACE_MSG_ID_HOME_POSITION_ACK
>;

//Receive home position after requesting for it, send ack out upon reception
using ControllerHome_Step_ReceiveHomePositionSendAck = Controllers::ActionFinalReceiveRespond<
    mace_message_t, MaceCore::ModuleCharacteristic,
    CONTROLLER_HOME_TYPE,
    MaceCore::ModuleCharacteristic,
    MaceCore::ModuleCharacteristic,
    CommandItem::SpatialHome,
    mace_home_position_t,
    mace_home_position_ack_t,
    MACE_MSG_ID_HOME_POSITION
>;

//Receive ack of home position received after sending it
using ControllerHome_Step_ReceiveFinishingAck = Controllers::ActionFinish<
    mace_message_t, MaceCore::ModuleCharacteristic,
    CONTROLLER_HOME_TYPE,
    MaceCore::ModuleCharacteristic,
    uint8_t,
    mace_home_position_ack_t,
    MACE_MSG_ID_HOME_POSITION_ACK
>;

//Set a home position on another controller
using ControllerHome_Step_SendHomePosition = Controllers::ActionSend<
    mace_message_t, MaceCore::ModuleCharacteristic,
    CONTROLLER_HOME_TYPE,
    MaceCore::ModuleCharacteristic,
    CommandItem::SpatialHome,
    mace_set_home_position_t,
    MACE_MSG_ID_HOME_POSITION_ACK
>;

//Receive the set home and send an ack out.
using ControllerHome_Step_ReceiveSetHomeSendACK = Controllers::ActionFinalReceiveRespond<
    mace_message_t, MaceCore::ModuleCharacteristic,
    CONTROLLER_HOME_TYPE,
    MaceCore::ModuleCharacteristic,
    MaceCore::ModuleCharacteristic,
    CommandItem::SpatialHome,
    mace_set_home_position_t,
    mace_home_position_ack_t,
    MACE_MSG_ID_SET_HOME_POSITION
>;

class ControllerHome : public CONTROLLER_HOME_TYPE,
        public ControllerHome_Step_BroadcastHome,
        public ControllerHome_Step_ReceiveBroadcastedHome,
        public ControllerHome_Step_RequestHome,
        public ControllerHome_Step_ReceiveHomeRequest,
        public ControllerHome_Step_ReceiveHomePositionSendAck,
        public ControllerHome_Step_ReceiveFinishingAck,
        public ControllerHome_Step_SendHomePosition,
        public ControllerHome_Step_ReceiveSetHomeSendACK
{

private:

    typedef ActionFinalReceiveRespond<
        mace_message_t, MaceCore::ModuleCharacteristic,
        CONTROLLER_HOME_TYPE,
        MaceCore::ModuleCharacteristic,
        MaceCore::ModuleCharacteristic,
        CommandItem::SpatialHome,
        mace_home_position_t,
        mace_home_position_ack_t,
        MACE_MSG_ID_HOME_POSITION
    >
    ReceiveHomePosition;


    typedef ActionFinalReceiveRespond<
        mace_message_t, MaceCore::ModuleCharacteristic,
        CONTROLLER_HOME_TYPE,
        MaceCore::ModuleCharacteristic,
        MaceCore::ModuleCharacteristic,
        CommandItem::SpatialHome,
        mace_set_home_position_t,
        mace_home_position_ack_t,
        MACE_MSG_ID_SET_HOME_POSITION
    >
    ReceiveSetHomePosition;

    std::unordered_map<MaceCore::ModuleCharacteristic, MaceCore::ModuleCharacteristic> m_ModulesRequestedFrom;


protected:


    virtual void Construct_Broadcast(const CommandItem::SpatialHome &data, const MaceCore::ModuleCharacteristic &sender, mace_home_position_t &msg);


    /**
     * @brief Contruct a SpatialHome object from broadcasted home position
     * @param vehicleObj
     * @return
     */
    virtual bool Construct_FinalObject(const mace_home_position_t &msg, const MaceCore::ModuleCharacteristic &sender, MaceCore::ModuleCharacteristic &key, CommandItem::SpatialHome &data);


    virtual void Request_Construct(const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target, mace_mission_request_home_t &msg, MaceCore::ModuleCharacteristic &queueObj);


    virtual bool BuildData_Send(const mace_mission_request_home_t &msg, const MaceCore::ModuleCharacteristic &sender, mace_home_position_t &rsp, MaceCore::ModuleCharacteristic &vehicleObj, MaceCore::ModuleCharacteristic &receiveQueueObj, MaceCore::ModuleCharacteristic &respondQueueObj);

    virtual bool Construct_FinalObjectAndResponse(const mace_home_position_t &msg, const MaceCore::ModuleCharacteristic &sender, mace_home_position_ack_t &response, MaceCore::ModuleCharacteristic &key, CommandItem::SpatialHome &data, MaceCore::ModuleCharacteristic &vehicleObj, MaceCore::ModuleCharacteristic &queueObj);


    virtual bool Finish_Receive(const mace_home_position_ack_t &ack, const MaceCore::ModuleCharacteristic &sender, uint8_t &ack_code, MaceCore::ModuleCharacteristic &queueObj);


    virtual bool Construct_Send(const CommandItem::SpatialHome &data, const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target, mace_set_home_position_t &msg, MaceCore::ModuleCharacteristic &queueObj);

    virtual bool Construct_FinalObjectAndResponse(const mace_set_home_position_t &msg, const MaceCore::ModuleCharacteristic &sender, mace_home_position_ack_t &ack, MaceCore::ModuleCharacteristic &key, CommandItem::SpatialHome &data, MaceCore::ModuleCharacteristic &vehicleObj, MaceCore::ModuleCharacteristic &queueObj);

public:

    ControllerHome(const Controllers::IMessageNotifier<mace_message_t, MaceCore::ModuleCharacteristic>* cb, TransmitQueue * queue, int linkChan);

};

}

#endif // CONTROLLER_HOME_H
