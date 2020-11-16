#ifndef ACTION_WRITETOVEHICLELOG_H
#define ACTION_WRITETOVEHICLELOG_H

#include <string>

class Action_WriteToVehicleLog
{
public:
    enum AI_LogEvent : uint8_t
    {
        TEST_START,
        TRIAL_START,
        TRIAL_ABORTED,
        TRIAL_END,
        TEST_END
    };

public:
    Action_WriteToVehicleLog();

private:
    AI_LogEvent _eventType;
    std::string _stringDetail;
};

#endif // ACTION_WRITETOVEHICLELOG_H
