#ifndef INTERFACE_COMMAND_HELPER_H
#define INTERFACE_COMMAND_HELPER_H

#include "mace.h"
#include "command_item_type.h"

template<class T>
class Interface_CommandHelper
{
public:
    virtual ~Interface_CommandHelper() = default;
protected:
    void initializeCommandItem(T &obj) const;

    virtual void populateCommandItem(T &obj) const = 0;

    virtual void fromCommandItem(const T &obj) = 0;

    void transferToMissionItem(const T &cmdObj, mace_mission_item_t &misObj) const;
};


#endif // INTERFACE_COMMAND_HELPER_H
