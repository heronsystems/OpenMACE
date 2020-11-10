#ifndef ACTION_WRITETOVEHICLELOG_H
#define ACTION_WRITETOVEHICLELOG_H


class Action_WriteToVehicleLog
{
public:
    enum AI_LogEvent: uint8_t
    {
        TEST_START,
        TRIAL_START,
        TRIAL_ABORTED,
        TRIAL_END,
        TEST_END
    };

public:
    Action_WriteToVehicleLog();
};

#endif // ACTION_WRITETOVEHICLELOG_H
