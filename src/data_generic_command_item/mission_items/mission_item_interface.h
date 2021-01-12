#ifndef MISSION_ITEM_INTERFACE_H
#define MISSION_ITEM_INTERFACE_H

#include <mavlink.h>

class Interface_MissionItem
{
public:
    Interface_MissionItem() = default;

    virtual ~Interface_MissionItem() = default;

public:
    virtual bool toMACEComms_MissionItem(mavlink_mission_item_t &obj) const = 0;

    virtual mavlink_message_t toMACEComms_MACEMsg(mavlink_mission_item_t &obj, const uint8_t &chan) const = 0;

protected:
    void initializeMissionItem(mavlink_mission_item_t &obj) const
    {
        obj.param1 = 0.0;
        obj.param2 = 0.0;
        obj.param3 = 0.0;
        obj.param4 = 0.0;

        obj.x = 0.0;
        obj.y = 0.0;
        obj.z = 0.0;

        obj.seq = 0;
        obj.command = 0;

        obj.target_system = 0;
        obj.mission_type = 0;
        obj.frame = 3;
        obj.current = 0;
        obj.autocontinue = 1;

    }
};

#endif // MISSION_ITEM_INTERFACE_H
