#include "guided_controller_mavlink.h"

namespace DataInterface_MAVLINK {

GuidedController_MAVLINK::GuidedController_MAVLINK(const int &targetID, const int &originatingID):
    systemID(targetID), transmittingID(originatingID),
    currentRetry(0), maxRetries(5), responseTimeout(500),
    currentCommsState(Data::ControllerCommsState::NEUTRAL),
    m_CB(NULL), prevTransmit(NULL),
    helperMAVtoMACE(targetID),helperMACEtoMAV(originatingID,0)
{
    mLog = spdlog::get("Log_Vehicle" + std::to_string(this->systemID));
}

void GuidedController_MAVLINK::clearPreviousTransmit()
{
    if(prevTransmit)
    {
        delete prevTransmit;
        prevTransmit = NULL;
    }
}

void GuidedController_MAVLINK::updateWaypointTarget(const CommandItem::SpatialWaypoint &target)
{
    std::stringstream buffer;
    buffer << target;

    mLog->debug("Guided Controller receieved a new waypoint target.");
    mLog->info(buffer.str());

    mavlink_mission_item_t request = helperMACEtoMAV.convertWaypoint(target,0);
    //this indicates this is a guided target
    request.current = 2;

    clearPendingTasks();
    clearPreviousTransmit();
    prevTransmit = new PreviousGuided<mavlink_mission_item_t>(guidedItemEnum::WAYPOINT, request);

    currentRetry = 0;
    this->start();
    mTimer.start();

    if(m_CB)
        m_CB->cbiGuidedController_TransmitMissionItem(request);
}

void GuidedController_MAVLINK::receivedMissionACK(const mavlink_mission_ack_t &missionACK)
{
    m_LambdasToRun.push_back([this, missionACK]{
        mTimer.stop();
        currentRetry = 0;
        currentCommsState = Data::ControllerCommsState::NEUTRAL;
    });
}

void GuidedController_MAVLINK::run()
{
    while(true)
    {
        if(mToExit == true) {
            clearPendingTasks();
            clearPreviousTransmit();
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
            guidedItemEnum type = prevTransmit->getType();
            currentRetry++;

            switch(currentCommsState)
            {
            case Data::ControllerCommsState::NEUTRAL:
            {
                //This case we should terminate this because there is nothing we should be doing apparently
                clearPreviousTransmit();
                mTimer.stop();
                mToExit = true;
             break;
            }
            case Data::ControllerCommsState::TRANSMITTING:
            {

                if(type == guidedItemEnum::WAYPOINT)
                {
                    mLog->error("GUIDED Controller is on attempt " + std::to_string(currentRetry) + " for waypoint target transmission.");
                    PreviousGuided<mavlink_mission_item_t> *tmp = static_cast<PreviousGuided<mavlink_mission_item_t>*>(prevTransmit);
                    mavlink_mission_item_t msgTransmit = tmp->getData();
                    mTimer.start();
                    if(m_CB)
                        m_CB->cbiGuidedController_TransmitMissionItem(msgTransmit);
                }
                break;
            }
            }

        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

} //end of namespace DataInterface_MAVLINK
