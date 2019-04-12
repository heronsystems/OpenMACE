#ifndef ACTION_RECEIVE_INTERMEDIATE_H
#define ACTION_RECEIVE_INTERMEDIATE_H

#include "action_base.h"

namespace Controllers {

template<typename MESSAGE_TYPE, typename COMPONENT_KEY, typename CONTROLLER_TYPE, typename RECEIVE_QUEUE_TYPE, typename RESPOND_QUEUE_TYPE, typename MSG_TYPE, const int MESSAGE_REQUEST_ID, typename ACK_TYPE>
class ActionIntermediateReceive :
        public ActionBase<CONTROLLER_TYPE>,
        public BaseDecode<COMPONENT_KEY, MESSAGE_TYPE, MSG_TYPE>
{

    typedef ActionBase<CONTROLLER_TYPE> BASE;

protected:

    virtual bool BuildData_Send(const MSG_TYPE &, const COMPONENT_KEY &sender, ACK_TYPE &, COMPONENT_KEY &vehicleObj, RECEIVE_QUEUE_TYPE &receiveQueueObj, RESPOND_QUEUE_TYPE &respondQueueObj)= 0;

public:

    ActionIntermediateReceive()
    {
        throw std::runtime_error("Default Constructor not supported");
    }

    ActionIntermediateReceive(CONTROLLER_TYPE *controller,
                                  const std::function<void(const ACK_TYPE &, const COMPONENT_KEY &, const RESPOND_QUEUE_TYPE &, const COMPONENT_KEY &)> &nextStep,
                                  const std::function<void(const MESSAGE_TYPE*, MSG_TYPE*)> &decode) :
        ActionBase<CONTROLLER_TYPE>(controller),
        BaseDecode<COMPONENT_KEY, MESSAGE_TYPE, MSG_TYPE>(decode)
    {


        BASE::m_Controller-> template AddTriggeredLogic<MESSAGE_REQUEST_ID, MSG_TYPE>( BaseDecode<COMPONENT_KEY, MESSAGE_TYPE, MSG_TYPE>::m_DecodeFunc,
                [this, nextStep](const MSG_TYPE  &msg, const COMPONENT_KEY &sender){

                    COMPONENT_KEY target = sender;

                    COMPONENT_KEY vehicleFrom;
                    ACK_TYPE ack;
                    RECEIVE_QUEUE_TYPE receiveQueueObj;
                    RESPOND_QUEUE_TYPE respondQueueObj;

//                    bool valid = this-> template BuildData_Send(msg, sender, ack, vehicleFrom, receiveQueueObj, respondQueueObj);
                    bool valid = this->BuildData_Send(msg, sender, ack, vehicleFrom, receiveQueueObj, respondQueueObj);


                    if(valid == true)
                    {
                        BASE::m_Controller->RemoveTransmission(receiveQueueObj, MESSAGE_REQUEST_ID, sender);

                        //BASE::m_Controller->Set(ack, vehicleFrom, queueObj, target);
                        nextStep(ack, vehicleFrom, respondQueueObj, target);
                    }
                }
        );
    }
};

}

#endif // ACTION_RECEIVE_INTERMEDIATE_H
