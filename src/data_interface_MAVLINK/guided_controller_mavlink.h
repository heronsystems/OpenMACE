#ifndef GUIDED_CONTROLLER_MAVLINK_H
#define GUIDED_CONTROLLER_MAVLINK_H

#include <iostream>
#include <QDate>

#include "mavlink.h"

#include "data/controller_comms_state.h"
#include "common/thread_manager.h"
#include "data/timer.h"

#include "data_generic_mission_item_topic/mission_item_topic_components.h"
#include "MACE_to_MAVLINK/helper_mission_mace_to_mavlink.h"
#include "MAVLINK_to_MACE/helper_mission_mavlink_to_mace.h"

#include "generic/helper_previous_guided_mavlink.h"

#include "spdlog/spdlog.h"

class GuidedController_Interface
{
public:
    virtual void cbiGuidedController_TransmitMissionItem(const mavlink_mission_item_t &item) = 0;

};


namespace DataInterface_MAVLINK{

class GuidedController_MAVLINK : public Thread
{
public:
    GuidedController_MAVLINK(const int &targetSystem, const int &targetComp);

    void updateWaypointTarget(const command_item::SpatialWaypoint &target);

    void receivedMissionACK(const mavlink_mission_ack_t &missionACK);

    ~GuidedController_MAVLINK() {
        std::cout << "Destructor on the mavlink guided controller" << std::endl;
        mToExit = true;
    }

    void run();

    Data::ControllerCommsState getCommsState() const
    {
        return this->currentCommsState;
    }

    void connectCallback(GuidedController_Interface *cb)
    {
        m_CB = cb;
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

    GuidedController_Interface *m_CB;
    PreviousTransmissionBase<guidedItemEnum> *prevTransmit;

    Data::ControllerCommsState currentCommsState;

    DataMAVLINK::Helper_MissionMAVLINKtoMACE helperMAVtoMACE;
    DataMAVLINK::Helper_MissionMACEtoMAVLINK helperMACEtoMAV;

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

#endif // GUIDED_CONTROLLER_MAVLINK_H
