#ifndef I_CONTROLLER_H
#define I_CONTROLLER_H

#include <vector>
#include "common/common.h"

#include "actions/action_base.h"

namespace Controllers {


enum class Actions
{
    SEND,
    REQUEST,
    BROADCAST
};


//!
//! \brief Interface for all controllers to take.
//!
//! \template MESSAGETYPE the type of message the controller is to injest/output
//! \template COMPONENT_KEY object that keys individual resources on the mavlink protocol
//!
template <typename MESSAGETYPE, typename COMPONENT_KEY>
class IController
{

public:

    //!
    //! \brief Receive a message for the controller
    //! \param message Message to receive
    //! \return True if action was taken, false if this module didnt' care about message
    //!
    virtual bool ReceiveMessage(const MESSAGETYPE* message, const COMPONENT_KEY &sender) = 0;

    virtual ~IController() = default;

    //!
    //! \brief Query to be given to determin if controller has the given action
    //! \param action Action to ask if controller contains
    //! \return True if contains
    //!
    virtual bool ContainsAction(const Actions action)
    {
        UNUSED(action);
        //no time to fix controllers that will be ultimatly removed. Adding implimentation for now.
        //We want this to eventually be pure
        printf("DEPRECATED!!! Function should be pure. Other controllers should impliment this");
    }


    //!
    //! \brief Remove all action tied to the given host
    //! \param ptr Pointer to host that actions attributed to are to be removed
    //!
    virtual void RemoveHost(void * ptr) = 0;
};

}

#endif // I_CONTROLLER_H
