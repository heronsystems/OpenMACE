#include "ardupilot_target_progess.h"

ArdupilotTargetProgess::ArdupilotTargetProgess()
{
    distanceThresholdAchieved = 0.5;
    distanceThresholdHunting = 10.0;
    maxDuration_Hunting = 20.0;
    maxDuration_Routing = std::numeric_limits<double>::max();
    state = Data::ControllerState::TRACKING;
    initializeTargetState();
}

ArdupilotTargetProgess::ArdupilotTargetProgess(const double &achievedDistance, const double &huntingDistance, const double &maxHuntingDuration) :
    distanceThresholdAchieved(achievedDistance), distanceThresholdHunting(huntingDistance), maxDuration_Hunting(maxHuntingDuration)
{
    state = Data::ControllerState::TRACKING;
    maxDuration_Routing = std::numeric_limits<double>::max();
    initializeTargetState();
}

Data::ControllerState ArdupilotTargetProgess::newTargetItem(const double &distance)
{
    initializeTargetStart();
    Data::ControllerState rtnState = updateTargetState(distance);
    return rtnState;
}

//Data::ControllerState ArdupilotTargetProgess::updateTargetTimes()
//{
//}

Data::ControllerState ArdupilotTargetProgess::updateTargetState(const double &distance)
{
    if(distance > distanceThresholdHunting)
    {
        //Really nothing to do in this case as the item is far away.
        state = Data::ControllerState::TRACKING;
    }
    else if((distance <= distanceThresholdHunting) && (distance > distanceThresholdAchieved))
    {
        //This case starts the interesting case where we are getting close to the item.
        //At this stage we are hunting for the target. If we remain in this state for a
        //period of time we can abort the item and move on by considering the item achieved.
        huntingStart = std::chrono::system_clock::now();
        state = Data::ControllerState::HUNTING;
    }
    else if(distance <= distanceThresholdAchieved)
    {
        //This case we have achieved the waypoint
        state = Data::ControllerState::ACHIEVED;
    }

    return state;
}

////////////////////////////////////////////////////////////////////////////
/// TIME EVENTS: These events are used to trigger a timelapse of the current
/// state of the mission/target/hunting times.
////////////////////////////////////////////////////////////////////////////

void ArdupilotTargetProgess::initializeTargetState()
{
    targetStart = std::chrono::system_clock::now();
    huntingStart = std::chrono::system_clock::now();
}

void ArdupilotTargetProgess::initializeTargetStart()
{
    targetStart = std::chrono::system_clock::now();
}

float ArdupilotTargetProgess::getCurrentTargetTime()
{
    std::chrono::time_point<std::chrono::system_clock> now;
    now = std::chrono::system_clock::now();
    std::chrono::duration<float> elapsed_seconds = now - targetStart;
    return elapsed_seconds.count();
}

float ArdupilotTargetProgess::getHuntingTime()
{
    std::chrono::time_point<std::chrono::system_clock> now;
    now = std::chrono::system_clock::now();
    std::chrono::duration<float> elapsed_seconds = now - huntingStart;
    return elapsed_seconds.count();
}


