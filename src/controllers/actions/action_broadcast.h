#ifndef ACTION_BROADCAST_H
#define ACTION_BROADCAST_H

#include "action_base.h"
#include <vector>

#include "common/optional_parameter.h"

namespace Controllers {


template<typename DATA_TYPE, typename COMPONENT_KEY>
class IActionBroadcast
{
public:
    virtual void Broadcast(const DATA_TYPE &commandItem, const COMPONENT_KEY &sender) = 0;
};


//!
//! \brief Sets up an action for an unreliable broadcast of data.
//!
//! This communication is  unreliable, so there is no gurantee that it will be received by everyone.
//! This should only be used for streamed data where the cost of setting up reliable communication control is too high.
//!
//! User of this action must impliment Construct_Broadcast method that defines how to translate given data (DATA_TYPE) from a given sender (COMPONENT_KEY) to a target (COMPONENT_KEY)
//!  into relevant data (MSG_TYPE)
//!
//! \template MESSAGE_TYPE Underlaying communication datastructure that all communication is done through
//! \template COMPONENT_KEY Type that identifies components on the controller network
//! \template CONTROLLER_TYPE Type of controller being used by this action, will be used to queue transmissions.
//! \template DATA_TYPE Data type of data that will be given to the action to transmit out.
//! \template MSG_TYPE Datatype that represents the desired message on the comm paradigm.
//!
template<typename MESSAGE_TYPE, typename COMPONENT_KEY, typename CONTROLLER_TYPE, typename DATA_TYPE, typename MSG_TYPE>
class ActionBroadcast :
        public ActionBase<CONTROLLER_TYPE>,
        public BaseEncode<COMPONENT_KEY, MESSAGE_TYPE, MSG_TYPE>,
        public IActionBroadcast<DATA_TYPE, COMPONENT_KEY>
{

    typedef ActionBase<CONTROLLER_TYPE> BASE;

protected:

    //!
    //! \brief Method that is to be implimented for this action to generate broadcasted message
    //! \param data Incomming data to translate, given in the Broadcast function
    //! \param sender Module emitting this action, given in the Send function
    //! \param msg Communications message to send to comms interface
    //!
    virtual void Construct_Broadcast(const DATA_TYPE &data, const COMPONENT_KEY &sender, MSG_TYPE &vec) = 0;

public:


    ActionBroadcast(CONTROLLER_TYPE *controller,
               const std::function<void(COMPONENT_KEY, uint8_t, MESSAGE_TYPE*, const MSG_TYPE*)> &encode_chan) :
        BASE(controller),
        BaseEncode<COMPONENT_KEY, MESSAGE_TYPE, MSG_TYPE>(encode_chan)
    {

    }


    /**
     * @brief Broadcast data
     * @param commandItem Data to broadcast
     * @param sender Module sending
     */
    virtual void Broadcast(const DATA_TYPE &commandItem, const COMPONENT_KEY &sender)
    {
        MSG_TYPE msg;
        Construct_Broadcast(commandItem, sender, msg);

        BASE::m_Controller-> template EncodeMessage(BaseEncode<COMPONENT_KEY, MESSAGE_TYPE, MSG_TYPE>::m_EncodeChanFunc, msg, sender);
    }
};

}

#endif // ACTION_BROADCAST_H
