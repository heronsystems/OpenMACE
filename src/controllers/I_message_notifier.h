#ifndef I_MESSAGE_NOTIFIER_H
#define I_MESSAGE_NOTIFIER_H

#include <vector>

#include "mace_core/module_characteristics.h"
#include "common/optional_parameter.h"

namespace Controllers {

//!
//! \brief Interface is to be given to controllers.
//!
//! The controller is to use this interface as a callback to transmit messages and to asks about the local topography of the network.
//!
template<typename MESSAGETYPE, typename COMPONENT_KEY>
class IMessageNotifier
{
public:

    //!
    //! \brief TransmitMessage
    //! \param msg Message to transmit
    //! \param target Target to transmitt to. Broadcast if not set.
    //!
    virtual void TransmitMessage(const MESSAGETYPE &msg, const OptionalParameter<COMPONENT_KEY> &target) const = 0;


    //!
    //! \brief It may be that the communication paradigm contains its own set of ID's that is seperate from the network the controllers are operating on.
    //! This method provides a means to link those two.
    //!
    //! For example in MACE the instances all are keyed based on ModuleCharacteristic. Yet vehicles are all keyed based on their own mavlink ID
    //!   At times it is nessessary to determine what module a given mavlink vehicle exists on, which this method would be used to do.
    //!
    //! \param ID ID on secondary network the controller operates on
    //! \return Primary key identifying entity on the controller's network
    //!
    virtual COMPONENT_KEY GetKeyFromSecondaryID(int ID) const = 0;

    //!
    //! \brief Get the object identifying the component that is hosting the controller
    //! \return Key
    //!
    virtual COMPONENT_KEY GetHostKey() const = 0;
};

}

#endif // I_MESSAGE_NOTIFIER_H
