#ifndef ACTION_RESPONSE_FINAL_DELIVERY_H
#define ACTION_RESPONSE_FINAL_DELIVERY_H

#include "action_base.h"

#include "../base_data_item.h"

namespace Controllers {

template<typename MESSAGE_TYPE, typename COMPONENT_KEY, typename CONTROLLER_TYPE, typename QUEUE_TYPE, typename FINAL_KEY, typename FINAL_TYPE, typename MSG_TYPE, typename ACK_TYPE, const int MESSAGE_REQUEST_ID>
class ActionFinalReceiveRespond :
        public ActionBase<CONTROLLER_TYPE>,
        public BaseDecode<COMPONENT_KEY, MESSAGE_TYPE, MSG_TYPE>,
        public BaseEncode<COMPONENT_KEY, MESSAGE_TYPE, ACK_TYPE>
{

    typedef ActionBase<CONTROLLER_TYPE> BASE;


protected:

    virtual bool Construct_FinalObjectAndResponse(const MSG_TYPE &, const COMPONENT_KEY &sender, ACK_TYPE &, FINAL_KEY &finalKey, FINAL_TYPE &finalData, COMPONENT_KEY &vehicleObj, QUEUE_TYPE &queueObj)= 0;

public:

    ActionFinalReceiveRespond(CONTROLLER_TYPE *controller,
                           const std::function<void(const MESSAGE_TYPE*, MSG_TYPE*)> &decode,
                           const std::function<void(COMPONENT_KEY, uint8_t, MESSAGE_TYPE*, const ACK_TYPE*)> &encode_ack_chan) :
        BASE(controller),
        BaseDecode<COMPONENT_KEY, MESSAGE_TYPE, MSG_TYPE>(decode),
        BaseEncode<COMPONENT_KEY, MESSAGE_TYPE, ACK_TYPE>(encode_ack_chan)
    {


        BASE::m_Controller-> template AddTriggeredLogic<MESSAGE_REQUEST_ID, MSG_TYPE>( BaseDecode<COMPONENT_KEY, MESSAGE_TYPE, MSG_TYPE>::m_DecodeFunc,
                [this, encode_ack_chan](const MSG_TYPE  &msg, const COMPONENT_KEY &sender){

                    COMPONENT_KEY target = sender;

                    COMPONENT_KEY moduleFor;
                    ACK_TYPE ack;
                    FINAL_KEY finalKey;
                    FINAL_TYPE finalObj;
                    QUEUE_TYPE queueObj;

//                    bool valid = this-> template Construct_FinalObjectAndResponse(msg, sender, ack, finalKey, finalObj, moduleFor, queueObj);
                    bool valid = this->Construct_FinalObjectAndResponse(msg, sender, ack, finalKey, finalObj, moduleFor, queueObj);

                    if(valid == true)
                    {
                        BASE::m_Controller->RemoveTransmission(queueObj, MESSAGE_REQUEST_ID, sender);
                        ((Controllers::DataItem<FINAL_KEY, FINAL_TYPE>*)BASE::m_Controller)->onDataReceived(finalKey, finalObj);
//                        this->template FinalResponse(ack, moduleFor, queueObj, target);
                        this->FinalResponse(ack, moduleFor, queueObj, target);
                    }
                }
        );
    }

    void FinalResponse(const ACK_TYPE &cmd, const COMPONENT_KEY &sender, const QUEUE_TYPE &queueObj, const COMPONENT_KEY &target)
    {
        UNUSED(queueObj);
        BASE::m_Controller->template EncodeMessage(BaseEncode<COMPONENT_KEY, MESSAGE_TYPE, ACK_TYPE>::m_EncodeChanFunc, cmd, sender, target);
    }

protected:
};

}

#endif // ACTION_RESPONSE_FINAL_DELIVERY_H
