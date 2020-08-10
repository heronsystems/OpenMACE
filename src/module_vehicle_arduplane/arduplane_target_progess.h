#ifndef ARDUPLANE_TARGET_PROGRESS_H
#define ARDUPLANE_TARGET_PROGRESS_H

#include <limits>
#include <chrono>

#include "data/controller_state.h"

class ArduplaneTargetProgess
{
public:
    ArduplaneTargetProgess();

    ArduplaneTargetProgess(const double &achievedDistance, const double &huntingDistance, const double &maxHuntingDuration);


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

#endif // ARDUPLANE_TARGET_PROGRESS_H
