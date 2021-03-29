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
struct ParameterRequestResult
{
    uint8_t vehicleID = 0;
    int parameterIndex = -1;
    std::string parameterName = "";
    double parameterValue = 0.0;
};
using CONTROLLER_PARAMETERREQUEST_TYPE = Controllers::GenericController<
    mavlink_message_t, MavlinkEntityKey,
    TransmitQueueWithKeys<MavlinkEntityKey, ObjectMAVLINKMsgIDTuple<MavlinkEntityKey>>,
    ParameterRequestResult,
    Controllers::DataItem<MaceCore::ModuleCharacteristic, ParameterRequestResult>
>;
using ParameterRequestSend = Controllers::ActionSend<
    mavlink_message_t,
    MavlinkEntityKey,
    CONTROLLER_PARAMETERREQUEST_TYPE,
    MavlinkEntityKey,
    ParameterRequestResult,
    mavlink_param_request_read_t,
    MAVLINK_MSG_ID_PARAM_VALUE
>;
using ParameterRequestFinish = Controllers::ActionFinish<
    mavlink_message_t,
    MavlinkEntityKey,
    CONTROLLER_PARAMETERREQUEST_TYPE,
    MavlinkEntityKey,
    ParameterRequestResult,
    mavlink_param_value_t,
    MAVLINK_MSG_ID_PARAM_VALUE
>;
class Controller_ParameterRequest : public CONTROLLER_PARAMETERREQUEST_TYPE,
        public ParameterRequestSend,
        public ParameterRequestFinish
{
protected:
    virtual bool Construct_Send(const ParameterRequestResult &userRequest, const MavlinkEntityKey &sender, const MavlinkEntityKey &target, mavlink_param_request_read_t &request, MavlinkEntityKey &queueObj)
    {
        UNUSED(sender);
        queueObj = target;
        if((userRequest.parameterIndex == -1) && (!userRequest.parameterName.empty())) //populate using the string
        {
                request.param_index = -1; //this forces the command to pay attention to the string instead
                char *target;
                if(userRequest.parameterName.length() < 16)
                    target = static_cast<char*>(malloc((1 + userRequest.parameterName.length())));
                else if(userRequest.parameterName.length() == 16)
                    target = static_cast<char*>(malloc(16));
                else
                    return false; //we shouldnt have made this request
                if (target == nullptr) /* check that nothing special happened to prevent tragedy  */
                    return false;
                strcpy(target, userRequest.parameterName.c_str());
                strcpy(request.param_id, target);
                free(target);
        }
        else if(userRequest.parameterIndex >=0)
        {
            request.param_index = static_cast<int16_t>(userRequest.parameterIndex);
        }
        else //there is no way to process this request
        {
            return false;
        }
        request.target_system = userRequest.vehicleID;
        request.target_component = 1;
        return true;
    }
    virtual bool Finish_Receive(const mavlink_param_value_t &msg, const MavlinkEntityKey &sender, ParameterRequestResult &result, MavlinkEntityKey &queueObj)
    {
        UNUSED(sender);
        UNUSED(queueObj);
        std::string IDString(msg.param_id);
        result.parameterName = IDString;
        result.parameterValue = static_cast<double>(msg.param_value);
        return true;
    }
public:
    Controller_ParameterRequest(const Controllers::IMessageNotifier<mavlink_message_t, MavlinkEntityKey> *cb, TransmitQueue *queue, int linkChan) :
        CONTROLLER_PARAMETERREQUEST_TYPE(cb, queue, linkChan),
        ParameterRequestSend(this, MavlinkEntityKeyToSysIDCompIDConverter<mavlink_param_request_read_t>(mavlink_msg_param_request_read_encode_chan)),
        ParameterRequestFinish(this, mavlink_msg_param_value_decode)
    {
    }
    virtual ~Controller_ParameterRequest() = default;
    void RequestParameterValue(const MavlinkEntityKey &vehicle, const std::string &parameterName)
    {
        ParameterRequestResult currentRequest;
        currentRequest.vehicleID = vehicle;
        currentRequest.parameterName = parameterName;
        ParameterRequestSend::Send(currentRequest, vehicle, vehicle);
    }
};

} //end of namespace MAVLINKVehicleControllers

#endif // CONTROLLER_PARAMETER_REQUEST_H
