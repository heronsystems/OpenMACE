#ifndef MISSION_ITEM_INTERFACE_H
#define MISSION_ITEM_INTERFACE_H

#include "mace.h"

class Interface_MissionItem
{
public:
    Interface_MissionItem() = default;

    virtual ~Interface_MissionItem() = default;

public:
    virtual bool toMACEComms_MissionItem(mace_mission_item_t &obj) const = 0;

    virtual mace_message_t toMACEComms_MSG(mace_mission_item_t &obj) const = 0;

protected:
    void initialMissionItem(mace_mission_item_t &obj)
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
        obj.mission_system = 0;
        obj.mission_creator = 0;
        obj.mission_id = 0;
        obj.mission_type = 0;
        obj.mission_state = 0;
        obj.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
        obj.current = 0;
        obj.autocontinue = 1;

    }
};

#endif // MISSION_ITEM_INTERFACE_H
