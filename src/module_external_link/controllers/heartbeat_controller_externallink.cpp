#include "heartbeat_controller_externallink.h"

namespace ExternalLink {


HeartbeatController_ExternalLink::HeartbeatController_ExternalLink(HeartbeatController_Interface *cb, const int &interval):
    systemID(0), transmittingID(0), responseTimeout(interval)
{
    connectCallback(cb);
}


void HeartbeatController_ExternalLink::updateIDS(const int &targetID, const int &originatingID)
{
    this->systemID = targetID;
    this->transmittingID = originatingID;
}

void HeartbeatController_ExternalLink::run()
{
    while(true)
    {
        if(mToExit == true) {
            clearPendingTasks();
            mTimer.stop();
            break;
        }

        this->RunPendingTasks();

        //The current state we can find out how much time has passed.
        //If one of the lambda expressions has fired the clock shoud
        //be reset right at the end, thus making this value small and
        //improbable the next function will fire
        double timeElapsed = mTimer.elapsedMilliseconds();

        if(timeElapsed > responseTimeout)
        {
            mTimer.stop();
            mTimer.start();
            //formulate the appropriate hearbeat message
            mavlink_mace_heartbeat_t heartbeat;
            heartbeat.autopilot = static_cast<uint8_t>(Data::AutopilotType::AUTOPILOT_TYPE_GENERIC);
            heartbeat.mace_companion = 1;
            heartbeat.mission_state = 0;
            heartbeat.protocol = 0;
            heartbeat.type = static_cast<uint8_t>(Data::SystemType::SYSTEM_TYPE_GCS);
            heartbeat.mavlinkID = static_cast<uint8_t>(this->systemID);
            if(m_CB)
                m_CB->cbiHeartbeatController_transmitCommand(heartbeat);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
}

} //end of namespace DataInterface_ExternalLink

