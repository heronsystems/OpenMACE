#ifndef ABSTRACT_ROOT_STATE_H
#define ABSTRACT_ROOT_STATE_H

#include <iostream>
#include <thread>
#include <mavlink.h>

#include "abstract_state_ardupilot.h"

#include "data_generic_command_item/abstract_command_item.h"
#include "data_generic_command_item/command_item_type.h"

#include "module_vehicle_MAVLINK/controllers/controller_system_mode.h"
#include "module_vehicle_MAVLINK/controllers/commands/command_home_position.h"

namespace ardupilot{
namespace state{

class AbstractRootState : public AbstractStateArdupilot
{
public:
    AbstractRootState();

    AbstractRootState(const Data::MACEHSMState &copy);

    AbstractRootState(const AbstractRootState &copy);

    /**
      */
    virtual ~AbstractRootState() = default;

public:
    /**
     *
     */
    template <class T>
    const T *as() const
    {
        //ensure that we are attempting to cast it to a type of state
        return static_cast<const T *>(this);
    }

    /**
     *
     */
    template <class T>
    T *as()
    {
        //ensure that we are attempting to cast it to a type of state
        return static_cast<T *>(this);
    }

public:

    bool handleCommand(const std::shared_ptr<AbstractCommandItem> command);

};

} //end of namespace ardupilot
} //end of namespace state

#endif // ABSTRACTPARENTSTATE_H
