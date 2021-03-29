#ifndef DISTRIBUTE_TEST_PARAMETERIZATION_H
#define DISTRIBUTE_TEST_PARAMETERIZATION_H

#include "mavlink.h"

#include "common/common.h"

#include "controllers/actions/action_broadcast.h"
#include "controllers/actions/action_unsolicited_receive.h"

#include "controllers/controllers_MAVLINK/common.h"

#include "data_generic_item/mace/ai_test_parameterization.h"

namespace MAVLINKUXVControllers {

namespace ModuleController {


using ActionSend_TestParameterizationBroadcast = Controllers::ActionBroadcast<
    mavlink_message_t,
    MaceCore::ModuleCharacteristic,
    BasicExternalLinkController_ModuleKeyed<DataGenericItem::AI_TestParameterization>,
    DataGenericItem::AI_TestParameterization,
    mavlink_ai_test_parameterization_t
>;

//Receive a broadcasted test procedural event position, accept and finish (no response)
using ActionReceive_TestParameterizationBroadcast = Controllers::ActionUnsolicitedReceive<
    mavlink_message_t,
    MaceCore::ModuleCharacteristic,
    BasicExternalLinkController_ModuleKeyed<DataGenericItem::AI_TestParameterization>,
    MaceCore::ModuleCharacteristic,
    DataGenericItem::AI_TestParameterization,
    mavlink_ai_test_parameterization_t,
    MAVLINK_MSG_ID_AI_TEST_PARAMETERIZATION
>;


class Distribute_TestParameterization : public BasicExternalLinkController_ModuleKeyed<DataGenericItem::AI_TestParameterization>,
        public ActionSend_TestParameterizationBroadcast,
        public ActionReceive_TestParameterizationBroadcast
{

protected:

    void Construct_Broadcast(const DataGenericItem::AI_TestParameterization &obj, const MaceCore::ModuleCharacteristic &sender, mavlink_ai_test_parameterization_t &parameterization) override
    {
        UNUSED(sender);
        obj.populateMACECOMMS_TestParameterization(parameterization);
    }

    bool Construct_FinalObject(const mavlink_ai_test_parameterization_t &msg, const MaceCore::ModuleCharacteristic &sender, MaceCore::ModuleCharacteristic &key, DataGenericItem::AI_TestParameterization &data) override
    {
        UNUSED(sender);
        UNUSED(key);
        std::cout<<"A new test parameterization had been received."<<std::endl;
        data.fromMACECOMMS_TestParameterization(msg);
        return true;
    }

public:
    Distribute_TestParameterization(const Controllers::IMessageNotifier<mavlink_message_t, MaceCore::ModuleCharacteristic> *cb, TransmitQueue *queue, int linkChan) :
        BasicExternalLinkController_ModuleKeyed<DataGenericItem::AI_TestParameterization>(cb, queue, linkChan, "TestParamterization", false),
        ActionSend_TestParameterizationBroadcast(this, ModuleToSysIDCompIDConverter<mavlink_ai_test_parameterization_t>(mavlink_msg_ai_test_parameterization_encode_chan)),
        ActionReceive_TestParameterizationBroadcast(this, mavlink_msg_ai_test_parameterization_decode)
    {

    }

};


} //end of namespace ModuleController

} //end of namespace MAVLINKVehicleControllers

#endif // DISTRIBUTE_TEST_PARAMETERIZATION_H
