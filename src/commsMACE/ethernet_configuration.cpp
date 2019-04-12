#include "ethernet_configuration.h"

#include <iostream>

namespace CommsMACE
{

EthernetConfiguration::EthernetConfiguration()
{
}

EthernetConfiguration::EthernetConfiguration(const uint32_t portNumber)
{
    this->_portNumber = portNumber;
}


EthernetConfiguration::EthernetConfiguration(EthernetConfiguration* copy)
{
    this->_portNumber = copy->_portNumber;
}

EthernetConfiguration::~EthernetConfiguration()
{

}

void EthernetConfiguration::setPortName        (const uint32_t &portNumber)
{
    this->_portNumber = portNumber;
}

}



