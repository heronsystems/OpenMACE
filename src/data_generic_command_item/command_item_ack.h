#ifndef COMMAND_ITEM_ACK_H
#define COMMAND_ITEM_ACK_H

#include <iostream>

#include "data/command_ack_type.h"

#include "command_item_type.h"

namespace command_item {

class CommandItemACK
{
public:
    CommandItemACK();
    CommandItemACK(const MAV_CMD &cmdType, const  MAV_CMD_ACK &codeType);
    CommandItemACK(const int &systemOrigin, const MAV_CMD &cmdType, const MAV_CMD_ACK &codeType, const int &systemTarget = 0);

public:
    MAV_CMD getCommandType() const
    {
        return this->cmd;
    }
     MAV_CMD_ACK getACKType() const
    {
        return this->code;
    }

    int getOriginatingSystem() const
    {
        return this->originatingSystem;
    }
    int getTargetSystem() const
    {
        return this->targetSystem;
    }

public:
    void setCommandType(const MAV_CMD &cmdType)
    {
        this->cmd = cmdType;
    }
    void setACKType(const  MAV_CMD_ACK &ackType)
    {
        this->code = ackType;
    }
    void setOriginatingSystem(const int &systemOrigin)
    {
        this->originatingSystem = systemOrigin;
    }
    void setTargetSystem(const int &systemTarget)
    {
        this->targetSystem = systemTarget;
    }

public:
    void operator = (const CommandItemACK &rhs)
    {
        this->cmd = rhs.cmd;
        this->code = rhs.code;
        this->originatingSystem = rhs.originatingSystem;
        this->targetSystem = rhs.targetSystem;
    }

    bool operator == (const CommandItemACK &rhs) {
        if(this->cmd != rhs.cmd){
            return false;
        }
        if(this->code != rhs.code){
            return false;
        }
        if(this->originatingSystem != rhs.originatingSystem){
            return false;
        }
        if(this->targetSystem != rhs.targetSystem){
            return false;
        }
        return true;
    }

    bool operator != (const CommandItemACK &rhs) {
        return !(*this == rhs);
    }


public:
    //KEN TODO: add and originating component in order to direct this information rather than broadcast
    MAV_CMD cmd;
    MAV_CMD_ACK code;
    int originatingSystem;
    int targetSystem;
};

} //end of namespace CommandItem

#endif // COMMAND_ITEM_ACK_H
