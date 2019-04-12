#ifndef MISSION_CONTROLLER_MAVLINK_H
#define MISSION_CONTROLLER_MAVLINK_H

#include <iostream>
#include <QDate>

#include "mavlink.h"

#include "data/controller_comms_state.h"
#include "common/thread_manager.h"
#include "data/timer.h"

#include "data_generic_mission_item_topic/mission_item_topic_components.h"
#include "MACE_to_MAVLINK/helper_mission_mace_to_mavlink.h"
#include "MAVLINK_to_MACE/helper_mission_mavlink_to_mace.h"

#include "generic/comms_item.h"
#include "generic/helper_previous_transmission.h"

#include "spdlog/spdlog.h"

//typedef void(*CallbackFunctionPtr_MisCount)(void*, mavlink_message_t &);

class MissionController_Interface
{
public:
    virtual void cbiMissionController_TransmitMissionCount(const mavlink_mission_count_t &count) = 0;
    virtual void cbiMissionController_TransmitMissionItem(const mavlink_mission_item_t &item) = 0;

    virtual void cbiMissionController_TransmitMissionReqList(const mavlink_mission_request_list_t &request) = 0;
    virtual void cbiMissionController_TransmitMissionReq(const mavlink_mission_request_t &requestItem) = 0;

    virtual void cbiMissionController_ReceviedHome(const CommandItem::SpatialHome &home) = 0;
    virtual void cbiMissionController_ReceivedMission(const MissionItem::MissionList &missionList) = 0;

    virtual void cbiMissionController_MissionACK(const mavlink_mission_ack_t &missionACK, const MissionItem::MissionList &missionList) = 0;
};

namespace DataInterface_MAVLINK {

class MissionController_MAVLINK : public Thread
{
public:
    MissionController_MAVLINK(const int &targetID, const int &originatingID);

    ~MissionController_MAVLINK() {
        std::cout << "Destructor on the mavlink mission controller" << std::endl;
        mToExit = true;
    }

    void run();

    void requestMission();

    void transmitMission(const MissionItem::MissionList &missionQueue);

    void transmitMissionItem(const mavlink_mission_request_t &missionRequest);

    void recievedMissionItem(const mavlink_mission_item_t &missionItem);

    void receivedMissionCount(const mavlink_mission_count_t &missionCount);

    void receivedMissionACK(const mavlink_mission_ack_t &missionACK);

    void connectCallback(MissionController_Interface *cb)
    {
        m_CB = cb;
    }

    Data::ControllerCommsState getCommsState() const
    {
        return this->currentCommsState;
    }
private:
    void clearPreviousTransmit();

private:
    int systemID;
    int transmittingID;

    Timer mTimer;
    int currentRetry;
    int maxRetries;
    int responseTimeout;

private:
    std::shared_ptr<spdlog::logger> mLog;

    MissionController_Interface *m_CB;
    PreviousTransmissionBase<commsItemEnum> *prevTransmit;

    DataMAVLINK::Helper_MissionMAVLINKtoMACE helperMAVtoMACE;
    DataMAVLINK::Helper_MissionMACEtoMAVLINK helperMACEtoMAV;

    Data::ControllerCommsState currentCommsState;

    MissionItem::MissionList missionList;
    CommandItem::SpatialHome missionHome;

protected:
    std::list<std::function<void()>> m_LambdasToRun;

    void clearPendingTasks()
    {
        m_LambdasToRun.clear();
    }

    void RunPendingTasks() {
        while(m_LambdasToRun.size() > 0) {
            auto lambda = m_LambdasToRun.front();
            m_LambdasToRun.pop_front();
            lambda();
        }
    }

};


} //end of namespace DataInterface_MAVLINK

#endif // MISSION_CONTROLLER_MAVLINK_H
