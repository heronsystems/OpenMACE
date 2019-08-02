#ifndef COMMAND_ITEM_INTERFACE_H
#define COMMAND_ITEM_INTERFACE_H

#include "mace.h"

template <class T>
class Interface_CommandItem
{
public:
    Interface_CommandItem() = default;

    virtual ~Interface_CommandItem() = default;

public:
    virtual bool toMACEComms_CommandItem(T &obj) const = 0;

    virtual mace_message_t toMACEComms_MSG(T &obj) const = 0;

protected:
    void initializeCommandItem(mace_command_long_t &obj)
    {

    }

    void initializeCommandItem(mace_command_short_t &obj)
    {

    }
};

#endif // COMMAND_ITEM_INTERFACE_H
