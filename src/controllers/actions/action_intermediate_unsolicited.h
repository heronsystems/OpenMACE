#ifndef ACTION_INTERMEDIATE_UNSOLICITED_H
#define ACTION_INTERMEDIATE_UNSOLICITED_H

#include "action_base.h"
#include "action_intermediate_unsolicited_receive.h"
#include "action_intermediate_respond.h"

namespace Controllers {


/**
 *
 * An action that kicks off an intermediate step. It doesn't invoke a finished and it isn't instigated by the user.
 * This action contains an "unsolicted receive", meaning it doesn't remove a previous transmission
 *
 * Example Usage:
 *      using SendHelper_RequestItem = ActionIntermediate<
 *          mace_messate_t,                         //Fundemental packet that data is sent out on
 *
 *          CONTROLLER_MISSION_TYPE,                //Controller type
 *          MissionItem::MissionKey,                //Queue that followup message will belong to. Used to add followup transmission to queue
 *          mace_mission_request_item_t,            //Message type to receive
 *          MACE_MSG_ID_MISSION_REQUEST_ITEM,       //ID of received message
 *          mavlink_mission_item_t,                    //Message type to followup with
 *          MACE_MSG_ID_MISSION_REQUEST_ITEM,       // List of message IDs that can unqueue the followup transmission
 *          MACE_MSG_ID_MISSION_ACK
 *      >;
 *
 */
template<typename MESSAGE_TYPE, typename COMPONENT_KEY, typename CONTROLLER_TYPE, typename RESPOND_QUEUE_TYPE, typename MSG_TYPE, const int MESSAGE_REQUEST_ID, typename ACK_TYPE, const int ...MESSAGE_ACK_ID>
class ActionIntermediateUnsolicited :
        public ActionIntermediateUnsolicitedReceive<MESSAGE_TYPE, COMPONENT_KEY, CONTROLLER_TYPE, RESPOND_QUEUE_TYPE, MSG_TYPE, MESSAGE_REQUEST_ID, ACK_TYPE>,
        public ActionIntermediateRespond<MESSAGE_TYPE, COMPONENT_KEY, CONTROLLER_TYPE, ACK_TYPE, MESSAGE_ACK_ID...>
{

public:

    ActionIntermediateUnsolicited(CONTROLLER_TYPE *controller,
                                    const std::function<void(const MESSAGE_TYPE*, MSG_TYPE*)> &decode,
                                    const std::function<void(COMPONENT_KEY, uint8_t, MESSAGE_TYPE*, const ACK_TYPE*)> &encode_ack_chan) :
        ActionIntermediateUnsolicitedReceive<MESSAGE_TYPE, COMPONENT_KEY, CONTROLLER_TYPE, RESPOND_QUEUE_TYPE, MSG_TYPE, MESSAGE_REQUEST_ID, ACK_TYPE>(controller,
                                  [this](const ACK_TYPE &A, const COMPONENT_KEY &B, const RESPOND_QUEUE_TYPE &C, const COMPONENT_KEY &D){ActionIntermediateRespond<MESSAGE_TYPE, COMPONENT_KEY, CONTROLLER_TYPE, ACK_TYPE, MESSAGE_ACK_ID...>::NextTransmission(A,B,C,D);},
                                  decode),
        ActionIntermediateRespond<MESSAGE_TYPE, COMPONENT_KEY, CONTROLLER_TYPE, ACK_TYPE, MESSAGE_ACK_ID...>(controller, encode_ack_chan)
    {
    }
};

}

#endif // ACTION_INTERMEDIATE_UNSOLICITED_H
