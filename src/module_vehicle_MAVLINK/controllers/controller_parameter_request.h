#ifndef CONTROLLER_PARAMETER_REQUEST_H
#define CONTROLLER_PARAMETER_REQUEST_H

#include <iostream>
#include <string.h>
#include "common/common.h"

#include "controllers/actions/action_send.h"
#include "controllers/actions/action_finish.h"

#include "mavlink.h"
#include "module_vehicle_MAVLINK/mavlink_entity_key.h"
#include "module_vehicle_MAVLINK/controllers/common.h"

namespace MAVLINKUXVControllers {

struct MAVLINKStruct_ParamRequest
{
    uint8_t target = 0;

    int paramID = -1;
    std::string paramName = "";
};

using ParameterRequestSend = Controllers::ActionSend<
    mavlink_message_t,
    MavlinkEntityKey,
    BasicMavlinkController_ModuleKeyed<MAVLINKStruct_ParamRequest>,
    MavlinkEntityKey,
    MAVLINKStruct_ParamRequest,
    mavlink_param_request_read_t,
    MAVLINK_MSG_ID_PARAM_VALUE
>;

using ParameterRequestFinish = Controllers::ActionFinish<
    mavlink_message_t,
    MavlinkEntityKey,
    BasicMavlinkController_ModuleKeyed<MAVLINKStruct_ParamRequest>,
    MavlinkEntityKey,
    double,
    mavlink_param_value_t,
    MAVLINK_MSG_ID_PARAM_VALUE
>;

class Controller_ParameterRequest : public BasicMavlinkController_ModuleKeyed<MAVLINKStruct_ParamRequest>,
        public ParameterRequestSend,
        public ParameterRequestFinish
{

protected:


    virtual bool Construct_Send(const MAVLINKStruct_ParamRequest &userRequest, const MavlinkEntityKey &sender, const MavlinkEntityKey &target, mavlink_param_request_read_t &request, MavlinkEntityKey &queueObj)
    {
        UNUSED(sender);
        queueObj = target;

        if((userRequest.paramID == -1) && (!userRequest.paramName.empty())) //populate using the string
        {
                char *target;
                if(userRequest.paramName.length() < 16)
                    target = static_cast<char*>(malloc((1 + userRequest.paramName.length())));
                else if(userRequest.paramName.length() == 16)
                    target = static_cast<char*>(malloc(16));
                else
                    return false; //we shouldnt have made this request

                if (target == nullptr) /* check that nothing special happened to prevent tragedy  */
                    return false;
                strcpy(target, request.param_id);
                free(target);
        }
        else if(userRequest.paramID >=0)
        {
            request.param_index = static_cast<int16_t>(userRequest.paramID);
        }
        else //there is no way to process this request
        {
            return false;
        }

        request.target_system = userRequest.target;
        request.target_component = 0;

        return true;
    }


    virtual bool Finish_Receive(const mavlink_param_value_t &msg, const MavlinkEntityKey &sender, double &value, MavlinkEntityKey &queueObj)
    {
        UNUSED(sender);
        UNUSED(queueObj);
        std::cout<<"The parameter value received is: "<<msg.param_value<<std::endl;
        value = static_cast<double>(msg.param_value);
        return true;
    }

public:

    Controller_ParameterRequest(const Controllers::IMessageNotifier<mavlink_message_t, MavlinkEntityKey> *cb, TransmitQueue *queue, int linkChan) :
        BasicMavlinkController_ModuleKeyed<MAVLINKStruct_ParamRequest>(cb, queue, linkChan),
        ParameterRequestSend(this, MavlinkEntityKeyToSysIDCompIDConverter<mavlink_param_request_read_t>(mavlink_msg_param_request_read_encode_chan)),
        ParameterRequestFinish(this, mavlink_msg_param_value_decode)
    {

    }

    virtual ~Controller_ParameterRequest() = default;


};

} //end of namespace MAVLINKVehicleControllers

#endif // CONTROLLER_PARAMETER_REQUEST_H

