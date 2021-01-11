#ifndef I_PROTOCOL_MACE_H
#define I_PROTOCOL_MACE_H

#include <memory>
#include <vector>


namespace CommsMACE
{

class ILink;

class IProtocol
{
public:
    virtual void ResetMetadataForLink(const ILink* link) = 0;

    virtual void ReceiveData(ILink *link, const std::vector<uint8_t> &buffer) = 0;

    //!
    //! \brief Get the protocol channel being used for a specific link
    //! \param link Link to check
    //! \return Channel of the protocol being used
    //!
    virtual uint8_t GetChannel(ILink *link) const = 0;


    //!
    //! \brief Set the channel being used for a specific link on the protocol
    //! \param link Link to set
    //! \param channel Channel to use
    //!
    virtual void SetChannel(ILink *link, uint8_t channel) = 0;
};


} //END MAVLINKComms

#endif // I_PROTOCOL_H
