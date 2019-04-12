#ifndef COMMAND_ITEM_ACK_H
#define COMMAND_ITEM_ACK_H

#include <iostream>

#include "data/command_ack_type.h"

#include "command_item_type.h"

namespace CommandItem {

class CommandItemACK
{
public:
    CommandItemACK();
    CommandItemACK(const COMMANDITEM &cmdType, const Data::CommandACKType &codeType);
    CommandItemACK(const int &systemOrigin, const COMMANDITEM &cmdType, const Data::CommandACKType &codeType, const int &systemTarget = 0);

public:
    COMMANDITEM getCommandType() const
    {
        return this->cmd;
    }
    Data::CommandACKType getACKType() const
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
    void setCommandType(const COMMANDITEM &cmdType)
    {
        this->cmd = cmdType;
    }
    void setACKType(const Data::CommandACKType &ackType)
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
    COMMANDITEM cmd;
    Data::CommandACKType code;
    int originatingSystem;
    int targetSystem;
};

} //end of namespace CommandItem

#endif // COMMAND_ITEM_ACK_H
