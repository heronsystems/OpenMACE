#ifndef ACTION_INTERMEDIATE_H
#define ACTION_INTERMEDIATE_H

#include "action_base.h"
#include "action_intermediate_receive.h"
#include "action_intermediate_respond.h"

namespace Controllers {

//!
//! \brief Sets up an action for an intermediate "handshake" step.
//!
//! This action is not invoked by the user of the controller and does not invoke data delivery.
//! It continues a reliable communication protocol, by removing a previously queued up transmission and sending a new queued transmision.
//!
//!
//! \template MESSAGE_TYPE Underlaying communication datastructure that all communication is done through
//! \template COMPONENT_KEY Type that identifies components on the controller network
//! \template CONTROLLER_TYPE Type of controller being used by this action, will be used to queue transmissions.
//! \template RECEIVE_QUEUE_TYPE Type of object that will identify the object in the queue that needs to be removed
//! \template RESPOND_QUEUE_TYPE Type of object that will establish uniqueness in the queue, for outgoing message
//! \template MSG_TYPE Comm's datatype of incomming message
//! \template MESSAGE_REQUEST_ID integer for the ID of incomming message (what to listen for)
//! \template ACK_TYPE Comm's datatype of outgoing message
//! \template MESSAGE_ACK_ID... array of possible integers that identifies the message identifier that is to stop transmission
//!
template<typename MESSAGE_TYPE, typename COMPONENT_KEY, typename CONTROLLER_TYPE, typename RECEIVE_QUEUE_TYPE, typename RESPOND_QUEUE_TYPE, typename MSG_TYPE, const int MESSAGE_REQUEST_ID, typename ACK_TYPE, const int ...MESSAGE_ACK_ID>
class ActionIntermediate :
        public ActionIntermediateReceive<MESSAGE_TYPE, COMPONENT_KEY, CONTROLLER_TYPE, RECEIVE_QUEUE_TYPE, RESPOND_QUEUE_TYPE, MSG_TYPE, MESSAGE_REQUEST_ID, ACK_TYPE>,
        public ActionIntermediateRespond<MESSAGE_TYPE, COMPONENT_KEY, CONTROLLER_TYPE, ACK_TYPE, MESSAGE_ACK_ID...>
{

public:

    ActionIntermediate(CONTROLLER_TYPE *controller,
                                    const std::function<void(const MESSAGE_TYPE*, MSG_TYPE*)> &decode,
                                    const std::function<void(COMPONENT_KEY, uint8_t, MESSAGE_TYPE*, const ACK_TYPE*)> &encode_ack_chan) :
        ActionIntermediateReceive<MESSAGE_TYPE, COMPONENT_KEY, CONTROLLER_TYPE, RECEIVE_QUEUE_TYPE, RESPOND_QUEUE_TYPE, MSG_TYPE, MESSAGE_REQUEST_ID, ACK_TYPE>(controller,
                                  [this](const ACK_TYPE &A, const COMPONENT_KEY &B, const RESPOND_QUEUE_TYPE &C, const COMPONENT_KEY &D){ActionIntermediateRespond<MESSAGE_TYPE, COMPONENT_KEY, CONTROLLER_TYPE, ACK_TYPE, MESSAGE_ACK_ID...>::NextTransmission(A,B,C,D);},
                                  decode),
        ActionIntermediateRespond<MESSAGE_TYPE, COMPONENT_KEY, CONTROLLER_TYPE, ACK_TYPE, MESSAGE_ACK_ID...>(controller, encode_ack_chan)
    {
    }
};

}

#endif // ACTION_INTERMEDIATE_H
