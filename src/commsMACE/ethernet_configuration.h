#ifndef ETHERNET_CONFIGURATION_H
#define ETHERNET_CONFIGURATION_H

#include <string>

#include "link_configuration_mace.h"

#include "commsmace_global.h"
#include "common/common.h"

#include "mace_digimesh_wrapper.h"

namespace CommsMACE
{

class COMMSMACESHARED_EXPORT EthernetConfiguration : public LinkConfiguration
{
public:

    EthernetConfiguration();
    EthernetConfiguration(const uint32_t portNumber);
    EthernetConfiguration(EthernetConfiguration* copy);

    ~EthernetConfiguration();


    const uint32_t portNumber() const  { return _portNumber; }

    void setPortName        (const uint32_t& portNumber);

private:

    uint32_t _portNumber;
};

}

#endif // ETHERNET_CONFIGURATION_H
