#ifndef CONTROLLER_TIMESYNC_H
#define CONTROLLER_TIMESYNC_H

#include <mavlink.h>

#include "controllers/generic_controller.h"

#include "controllers/actions/action_send.h"
#include "controllers/actions/action_intermediate_respond.h"
#include "controllers/actions/action_intermediate_receive.h"
#include "controllers/actions/action_intermediate.h"
#include "controllers/actions/action_final_receive_respond.h"
#include "controllers/actions/action_finish.h"
#include "controllers/actions/action_request.h"
#include "controllers/actions/action_unsolicited_receive_respond.h"
#include "controllers/actions/action_unsolicited_receive.h"

#include "controllers/controllers_MAVLINK/common.h"

#include "data_generic_item/data_generic_item_timesync.h"

#include "common/logging/macelog.h"

namespace MAVLINKUXVControllers {

using UnsolicitedReceiveRespondTimesyncNotification = Controllers::ActionUnsolicitedReceiveRespond<
    mavlink_message_t,
    MavlinkEntityKey,
    BasicMavlinkController_ModuleKeyed<DataGenericItem::DataGenericItem_Timesync>,
    MavlinkEntityKey,
    DataGenericItem::DataGenericItem_Timesync,
    mavlink_timesync_t,
    mavlink_timesync_t,
    MAVLINK_MSG_ID_TIMESYNC
>;


class ControllerTimesync : public BasicMavlinkController_ModuleKeyed<DataGenericItem::DataGenericItem_Timesync>,
        public UnsolicitedReceiveRespondTimesyncNotification
{


protected:


    bool Construct_FinalObjectAndResponse(const mavlink_timesync_t &msg, const MavlinkEntityKey &sender, mavlink_timesync_t &cmd, MavlinkEntityKey &componentResponding, MavlinkEntityKey &key, DataGenericItem::DataGenericItem_Timesync &data) override
    {
        key = sender;
        componentResponding = sender;

        if(msg.tc1 == 0) {
            // This is being sent by the autopilot, respond with the current timestamp:
            std::chrono::nanoseconds now_usec = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch());
            cmd.tc1 = now_usec.count();
            cmd.ts1 = msg.ts1; // ts1 is in nanoseconds

            data.setTC1(cmd.tc1);
            data.setTS1(cmd.ts1);
            // TODO-PAT: Do anything with this data???
        }
        else {
            MaceLog::Alert("Received a TIMESYNC reply");
        }

        return true;
    }


public:

    ControllerTimesync(const Controllers::IMessageNotifier<mavlink_message_t, MavlinkEntityKey> *cb, TransmitQueue *queue, int linkChan) :
        BasicMavlinkController_ModuleKeyed<DataGenericItem::DataGenericItem_Timesync>(cb, queue, linkChan, "Timesync", false),
        UnsolicitedReceiveRespondTimesyncNotification(this, mavlink_msg_timesync_decode, MavlinkEntityKeyToSysIDCompIDConverter<mavlink_timesync_t>(mavlink_msg_timesync_encode_chan))
    {

    }

};

} // end namespace MAVLINKUXVControllers

#endif // CONTROLLER_TIMESYNC_H
