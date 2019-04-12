#ifndef ACTION_RESPONSE_INTERMEDIATE_H
#define ACTION_RESPONSE_INTERMEDIATE_H

#include "action_base.h"

namespace Controllers {

template<typename MESSAGE_TYPE, typename COMPONENT_KEY, typename CONTROLLER_TYPE, typename MSG_TYPE, const int ...MESSAGE_ACK_ID>
class ActionIntermediateRespond :
        public ActionBase<CONTROLLER_TYPE>,
        public BaseEncode<COMPONENT_KEY, MESSAGE_TYPE, MSG_TYPE>
{

    typedef ActionBase<CONTROLLER_TYPE> BASE;

public:

    ActionIntermediateRespond()
    {
        throw std::runtime_error("Default Constructor not supported");
    }

    ActionIntermediateRespond(CONTROLLER_TYPE *controller,
                                  const std::function<void(COMPONENT_KEY, uint8_t, MESSAGE_TYPE*, const MSG_TYPE*)> &encode_chan) :
        ActionBase<CONTROLLER_TYPE>(controller),
        BaseEncode<COMPONENT_KEY, MESSAGE_TYPE, MSG_TYPE>(encode_chan)
    {
    }

protected:

    template<typename QUEUE_TYPE>
    void NextTransmission(const MSG_TYPE &cmd, const COMPONENT_KEY &sender, const QUEUE_TYPE &queueObj, const COMPONENT_KEY &target)
    {

        std::vector<int> expectedResponses { { MESSAGE_ACK_ID... } };
        BASE::m_Controller-> template QueueTransmission<QUEUE_TYPE>(queueObj, expectedResponses, target, [this, cmd, sender](const std::vector<COMPONENT_KEY> &targets){

            if(targets.size() != 1)
            {
                throw std::runtime_error("Queued transmission with one target, but received multiple to send out");
            }

            BASE::m_Controller-> template EncodeMessage(BaseEncode<COMPONENT_KEY, MESSAGE_TYPE, MSG_TYPE>::m_EncodeChanFunc, cmd, sender, targets.at(0));
        });
    }
};

}

#endif // ACTION_RESPONSE_INTERMEDIATE_H
