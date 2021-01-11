#ifndef I_PROTOCOL_EVENTS_MACE_H
#define I_PROTOCOL_EVENTS_MACE_H

#include <string>
#include <memory>

#include "i_link_mace.h"


namespace CommsMACE
{

//!
//! \brief Interface that it to be implemented by users of MavlinkComms to listen for any events it fired
//!
class IProtocolEvents
{
public:

    //!
    //! \brief A message about protocol has been generated
    //! \param linkName Link identifier which generated call
    //! \param title
    //! \param message
    //!
    virtual void ProtocolStatusMessage(const ILink* link_ptr, const std::string &title, const std::string &message) const = 0;

    virtual void ReceiveLossPercentChanged(const ILink* link_ptr, int uasId, float lossPercent) const = 0;
    virtual void ReceiveLossTotalChanged(const ILink* link_ptr, int uasId, int totalLoss) const = 0;
};


} //END MAVLINKComms

#endif // I_PROTOCOL_EVENTS_H
