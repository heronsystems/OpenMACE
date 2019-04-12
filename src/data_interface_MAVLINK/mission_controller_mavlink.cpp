#include "mission_controller_mavlink.h"

namespace DataInterface_MAVLINK {

MissionController_MAVLINK::MissionController_MAVLINK(const int &targetID, const int &originatingID):
    systemID(targetID), transmittingID(originatingID),
    currentRetry(0), maxRetries(5), responseTimeout(800),\
    currentCommsState(Data::ControllerCommsState::NEUTRAL),
    m_CB(NULL), prevTransmit(NULL),
    helperMAVtoMACE(targetID),helperMACEtoMAV(originatingID,0)
{
    mLog = spdlog::get("Log_Vehicle" + std::to_string(this->systemID));
    missionList.setCreatorID(systemID);
    missionList.setVehicleID(systemID);
}


void MissionController_MAVLINK::clearPreviousTransmit()
{
    if(prevTransmit)
    {
        delete prevTransmit;
        prevTransmit = NULL;
    }
}

void MissionController_MAVLINK::requestMission()
{
    mLog->info("Mission Controller has seen a request mission.");
    missionList.setCreatorID(systemID);
    missionList.setVehicleID(systemID);
    this->missionList.clearQueue();
    currentCommsState = Data::ControllerCommsState::RECEIVING;

    mavlink_mission_request_list_t request;
    request.mission_type = MAV_MISSION_TYPE_MISSION;
    request.target_system = systemID;
    request.target_component = 0;

    clearPendingTasks();
    clearPreviousTransmit();
    prevTransmit = new PreviousTransmission<mavlink_mission_request_list_t>(commsItemEnum::ITEM_RXLIST, request);

    currentRetry = 0;
    this->start();
    mTimer.start();

    if(m_CB)
        m_CB->cbiMissionController_TransmitMissionReqList(request);
}

void MissionController_MAVLINK::transmitMission(const MissionItem::MissionList &missionQueue)
{

    mLog->info("Mission Controller has been instructed to transmit a mission.");

    if(missionQueue.getVehicleID() == this->systemID)
    {
        this->missionList = missionQueue;
    }

    currentCommsState = Data::ControllerCommsState::TRANSMITTING;
    mavlink_mission_count_t count;
    count.count = this->missionList.getQueueSize() + 1;
    count.mission_type = MAV_MISSION_TYPE_MISSION;
    count.target_component = 0;
    count.target_system = systemID;

    clearPendingTasks();
    clearPreviousTransmit();
    prevTransmit = new PreviousTransmission<mavlink_mission_count_t>(commsItemEnum::ITEM_TXCOUNT, count);

    currentRetry = 0;
    this->start();
    mTimer.start();

    if(m_CB)
        m_CB->cbiMissionController_TransmitMissionCount(count);
}

void MissionController_MAVLINK::transmitMissionItem(const mavlink_mission_request_t &missionRequest)
{
    m_LambdasToRun.push_back([this, missionRequest]{
        mTimer.stop();
        int index = missionRequest.seq;

        mLog->info("Mission Controller has seen a request for mission item number " + std::to_string(index) + ".");

        currentCommsState = Data::ControllerCommsState::TRANSMITTING;
        mavlink_mission_item_t missionItem;

        if(index == 0) //the vehicle requested the home position
        {
            missionItem = helperMACEtoMAV.convertHome(this->missionHome);
        }
        else{
            std::shared_ptr<CommandItem::AbstractCommandItem> ptrItem = this->missionList.getMissionItem(index - 1);
            helperMACEtoMAV.MACEMissionToMAVLINKMission(ptrItem,index,missionItem);
        }

        clearPreviousTransmit();
        prevTransmit = new PreviousTransmission<mavlink_mission_item_t>(commsItemEnum::ITEM_TXITEM, missionItem);
        currentRetry = 0;
        mTimer.start();

        if(m_CB)
            m_CB->cbiMissionController_TransmitMissionItem(missionItem);
    });
}


void MissionController_MAVLINK::run()
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
            commsItemEnum type = prevTransmit->getType();
            currentRetry++;

            switch(currentCommsState)
            {
            case Data::ControllerCommsState::NEUTRAL:
            {
                //This case we should terminate this because there is nothing we should be doing apparently
                clearPendingTasks();
                clearPreviousTransmit();
                mTimer.stop();
                mToExit = true;
             break;
            }
            case Data::ControllerCommsState::RECEIVING:
            {
                if(type == commsItemEnum::ITEM_RXLIST)
                {
                    mLog->error("Mission Controller is on attempt " + std::to_string(currentRetry) + " for " + getCommsItemEnumString(type) + ".");
                    PreviousTransmission<mavlink_mission_request_list_t> *tmp = static_cast<PreviousTransmission<mavlink_mission_request_list_t>*>(prevTransmit);
                    mavlink_mission_request_list_t msgTransmit = tmp->getData();
                    mTimer.start();
                    if(m_CB)
                        m_CB->cbiMissionController_TransmitMissionReqList(msgTransmit);
                }
                else if(type == commsItemEnum::ITEM_RXITEM)
                {
                    mLog->error("Mission Controller is on attempt " + std::to_string(currentRetry) + " for " + getCommsItemEnumString(type) + ".");
                    PreviousTransmission<mavlink_mission_request_t> *tmp = static_cast<PreviousTransmission<mavlink_mission_request_t>*>(prevTransmit);
                    mavlink_mission_request_t msgTransmit = tmp->getData();
                    mTimer.start();
                    if(m_CB)
                        m_CB->cbiMissionController_TransmitMissionReq(msgTransmit);
                }
                break;
            }
            case Data::ControllerCommsState::TRANSMITTING:
            {

                if(type == commsItemEnum::ITEM_TXCOUNT)
                {
                    mLog->error("Mission Controller is on attempt " + std::to_string(currentRetry) + " for " + getCommsItemEnumString(type) + ".");
                    PreviousTransmission<mavlink_mission_count_t> *tmp = static_cast<PreviousTransmission<mavlink_mission_count_t>*>(prevTransmit);
                    mavlink_mission_count_t msgTransmit = tmp->getData();
                    mTimer.start();
                    if(m_CB)
                        m_CB->cbiMissionController_TransmitMissionCount(msgTransmit);
                }
                else if(type == commsItemEnum::ITEM_TXITEM)
                {
                    mLog->error("Mission Controller is on attempt " + std::to_string(currentRetry) + " for " + getCommsItemEnumString(type) + ".");
                    PreviousTransmission<mavlink_mission_item_t> *tmp = static_cast<PreviousTransmission<mavlink_mission_item_t>*>(prevTransmit);
                    mavlink_mission_item_t msgTransmit = tmp->getData();
                    mTimer.start();
                    if(m_CB)
                        m_CB->cbiMissionController_TransmitMissionItem(msgTransmit);
                }
                break;
            }
            }

        }

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

void MissionController_MAVLINK::receivedMissionCount(const mavlink_mission_count_t &missionCount)
{
    m_LambdasToRun.push_back([this, missionCount]{
        mLog->info("Mission Controller received a mission count of " + std::to_string(missionCount.count));
        mTimer.stop();
        this->missionList.initializeQueue(missionCount.count - 1);

        mavlink_mission_request_t request;
        request.mission_type = MAV_MISSION_TYPE_MISSION;
        request.seq = 0;
        request.target_system = systemID;
        request.target_component = 0;

        mLog->info("Mission Controller is requesting mission item " + std::to_string(0));

        clearPreviousTransmit();
        prevTransmit = new PreviousTransmission<mavlink_mission_request_t>(commsItemEnum::ITEM_RXITEM, request);
        currentRetry = 0;
        mTimer.start();
        if(m_CB)
            m_CB->cbiMissionController_TransmitMissionReq(request);
    });
}


void MissionController_MAVLINK::receivedMissionACK(const mavlink_mission_ack_t &missionACK)
{    
    m_LambdasToRun.push_back([this, missionACK]{
        mTimer.stop();
        currentRetry = 0;
        currentCommsState = Data::ControllerCommsState::NEUTRAL;
        mToExit = true;
        if(m_CB)
            m_CB->cbiMissionController_MissionACK(missionACK, this->missionList);
    });

}

void MissionController_MAVLINK::recievedMissionItem(const mavlink_mission_item_t &missionItem)
{
    m_LambdasToRun.push_back([this, missionItem]{
    mTimer.stop();
    currentRetry = 0;
    int index = missionItem.seq;
    mLog->info("Mission Controller received mission item " + std::to_string(index));

    if(index > (this->missionList.getQueueSize() + 1))
    {
        mTimer.start();
        return;
    }

    if(index == 0) //This implies we recieved the home position according to the mission
    {
        //This is the home position item associated with the vehicle
        CommandItem::SpatialHome newHome;
        newHome.position->setX(missionItem.x);
        newHome.position->setY(missionItem.y);
        newHome.position->setZ(missionItem.z);
        newHome.setOriginatingSystem(systemID);
        newHome.setTargetSystem(systemID);
        m_CB->cbiMissionController_ReceviedHome(newHome);
    }else{
        int adjustedIndex = index - 1; //we decrement 1 only here because ardupilot references home as 0 and we 0 index in our mission queue
        std::shared_ptr<CommandItem::AbstractCommandItem> newMissionItem = helperMAVtoMACE.Convert_MAVLINKTOMACE(missionItem);
        this->missionList.replaceMissionItemAtIndex(newMissionItem,adjustedIndex);
    }

    MissionItem::MissionList::MissionListStatus status = this->missionList.getMissionListStatus();
    if(status.state == MissionItem::MissionList::INCOMPLETE)
    {
        int indexRequest = status.remainingItems.at(0)+1;
        mLog->info("Mission Controller is requesting mission item " + std::to_string(indexRequest));
        mavlink_mission_request_t request;
        request.mission_type = MAV_MISSION_TYPE_MISSION;
        request.seq = indexRequest;
        request.target_system = systemID;
        request.target_component = 0;

        clearPreviousTransmit();
        prevTransmit = new PreviousTransmission<mavlink_mission_request_t>(commsItemEnum::ITEM_RXITEM, request);
        currentRetry = 0;
        mTimer.start();

        if(m_CB)
            m_CB->cbiMissionController_TransmitMissionReq(request);
    }else{
        mLog->info("Mission Controller has received the entire mission of " + std::to_string(this->missionList.getQueueSize()));
        clearPendingTasks();
        mToExit = true;
        currentCommsState = Data::ControllerCommsState::NEUTRAL;
        m_CB->cbiMissionController_ReceivedMission(this->missionList);
    }
    });
}

} //end of namespace DataInterface_MAVLINK
