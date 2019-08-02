#ifndef COMMAND_INTERFACE_MAVLINK_H
#define COMMAND_INTERFACE_MAVLINK_H

#include "mavlink.h"

#include "data_generic_command_item/command_item_components.h"

typedef void(*CallbackFunctionPtr_CmdLng)(void*, mavlink_command_long_t&);

namespace DataInterface_MAVLINK{

class CommandInterface_MAVLINK
{
public:
    CommandInterface_MAVLINK(const int &targetSystem, const int &targetComp);

    virtual ~CommandInterface_MAVLINK();

    virtual void getSystemHome(const int &compID = 0);
    
    virtual mavlink_message_t setNewMode(const int &newMode, const int &originatingSystemID, const int &chan);
    virtual mavlink_message_t setHomePosition(const command_item::SpatialHome &commandItem, const int &compID);
    
    virtual void setSystemArm(const command_item::ActionArm &commandItem, const int &compID = 0);
    virtual void setSystemTakeoff(const command_item::SpatialTakeoff &commandItem, const int &compID = 0);
    virtual void setSystemLand(const command_item::SpatialLand &commandItem, const int &compID = 0);
    virtual void setSystemRTL(const command_item::SpatialRTL &commandItem, const int &compID = 0);

    
    virtual mavlink_command_long_t initializeCommandLong();
    virtual mavlink_message_t packLongMessage(const mavlink_command_long_t &cmdLong, const int &originatingSystemID, const int &originatingCompID, const int &chan);

    //establish callback connections
public:
    void connectCallback_CommandLong(CallbackFunctionPtr_CmdLng cb, void *p);

private:
    int targetSystemID;
    int targetCompID;

private:
    CallbackFunctionPtr_CmdLng m_CBCmdLng;
    void *m_p;

};

} //end of namespace DataInterface_MAVLINK

#endif // COMMAND_INTERFACE_MAVLINK_H
