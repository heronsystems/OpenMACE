#ifndef ARDUPILOT_TARGET_PROGRESS_H
#define ARDUPILOT_TARGET_PROGRESS_H

#include <limits>
#include <chrono>

#include "data/controller_state.h"

class ArdupilotTargetProgess
{
public:
    ArdupilotTargetProgess();

    ArdupilotTargetProgess(const double &achievedDistance, const double &huntingDistance, const double &maxHuntingDuration);


    void updateMaxDurationRouting(const double &value)
    {
        maxDuration_Routing = value;
    }

    void updateMaxDurationHunting(const double &value)
    {
        maxDuration_Hunting = value;
    }

    void updateHuntingThreshold(const double &value)
    {
        distanceThresholdHunting = value;
    }

    void updateAchievedThreshold(const double &value)
    {
        distanceThresholdAchieved = value;
    }

    double getMaxDurationRouting() const
    {
        return maxDuration_Routing;
    }

    double getMaxDurationHunting() const
    {
        return maxDuration_Hunting;
    }

    double getDistanceThresholdHunting() const
    {
        return distanceThresholdHunting;
    }

    double getDistanceThresholdAchieved() const
    {
        return distanceThresholdAchieved;
    }

    Data::ControllerState newTargetItem(const double &distance);

    Data::ControllerState updateTargetState(const double &distance);

//    Data::ControllerState updateTargetTimes();

    float getCurrentMissionTime();
    float getCurrentTargetTime();
    float getHuntingTime();

private:
    void initializeTargetState();
    void initializeTargetStart();

private:
    std::chrono::time_point<std::chrono::system_clock> missionStart;
    std::chrono::time_point<std::chrono::system_clock> targetStart;
    std::chrono::time_point<std::chrono::system_clock> huntingStart;

private:
    Data::ControllerState state;

    double distanceThresholdAchieved;
    double distanceThresholdHunting;

    double maxDuration_Routing;
    double maxDuration_Hunting;
};

#endif // ARDUPILOT_TARGET_PROGRESS_H
