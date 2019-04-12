#ifndef ACTION_FINISH_H
#define ACTION_FINISH_H

#include "action_base.h"

namespace Controllers {

//!
//! \brief Finishes a previously schedule action, used to receive an ACK
//!
//! Use of this method requires the caller to set a lambda in setLambda_Finished.
//! The lambda set will be called with received ACK_TYPE when appropriate message is received.
//!
//! \template MESSAGE_TYPE Underlaying generic message type that all communication is done through
//! \template COMPONENT_KEY Type that identifies actors on the network
//! \template CONTROLLER_TYPE Type of controller being used by this action, will be used to queue transmissions.
//! \template QUEUE_TYPE Type of object that will establish uniqueness in the queue.
//!   This ultimatly allows a controller to have two identical messages going out to two entities.
//! \template ACK_TYPE Data type to represent acknowledgment code to be received by this action.
//! \template MSG_TYPE Communication message class received by this action
//! \template MESSAGE_REQUEST_ID Integer code for message that is to kick off this action
//!
template<typename MESSAGE_TYPE, typename COMPONENT_KEY, typename CONTROLLER_TYPE, typename QUEUE_TYPE, typename ACK_TYPE, typename MSG_TYPE, const int MESSAGE_REQUEST_ID>
class ActionFinish :
        public ActionBase<CONTROLLER_TYPE>,
        public BaseDecode<COMPONENT_KEY, MESSAGE_TYPE, MSG_TYPE>
{

    typedef ActionBase<CONTROLLER_TYPE> BASE;

protected:

    //!
    //! \brief Method that is to be implimented for this action to act on received ack message.
    //!
    //! Translates a received MSG_TYPE to the ACK_TYPE contained within, and the queue object to identify the transmission that should be removed.
    //!
    //! \param msg Message received over communication network
    //! \param sender Module that sent message received
    //! \param ack Ack code to be generated from given message/sender
    //! \param queueObj Key that is to be generated from message/sender which identifies what queued transmission needs to be removed
    //! \return True if message is to be consumed, false if ignored (and possibly consumed by another action)
    //!
    virtual bool Finish_Receive(const MSG_TYPE &msg, const COMPONENT_KEY &sender, ACK_TYPE& ack, QUEUE_TYPE &queueObj) = 0;

public:


    ActionFinish(CONTROLLER_TYPE *controller,
                           const std::function<void(const MESSAGE_TYPE*, MSG_TYPE*)> &decode) :
        ActionBase<CONTROLLER_TYPE>(controller),
        BaseDecode<COMPONENT_KEY, MESSAGE_TYPE, MSG_TYPE>(decode)
    {


        BASE::m_Controller-> template AddTriggeredLogic<MESSAGE_REQUEST_ID, MSG_TYPE>( BaseDecode<COMPONENT_KEY, MESSAGE_TYPE, MSG_TYPE>::m_DecodeFunc,
                [this](const MSG_TYPE  &msg, const COMPONENT_KEY &sender){

                    QUEUE_TYPE queueObj;
                    ACK_TYPE code;
//                    bool valid = this-> template Finish_Receive(msg, sender, code, queueObj);
                    bool valid = this->Finish_Receive(msg, sender, code, queueObj);

                    if(valid == true)
                    {
                        BASE::m_Controller->onFinished(true, code);
                        BASE::m_Controller->RemoveTransmission(queueObj, MESSAGE_REQUEST_ID, sender);
                    }
                }
        );
    }

protected:
};

}

#endif // ACTION_FINISH_H
