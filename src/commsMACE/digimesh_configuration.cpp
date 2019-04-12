#include "digimesh_configuration.h"

#include <iostream>

namespace CommsMACE
{

DigiMeshConfiguration::DigiMeshConfiguration(const std::string& name)
{
    UNUSED(name);
    _baud = DigiMeshBaudRates::Baud9600;
}




DigiMeshConfiguration::DigiMeshConfiguration(DigiMeshConfiguration* copy)
{
    _baud = copy->baud();
}

DigiMeshConfiguration::~DigiMeshConfiguration()
{

}

void DigiMeshConfiguration::setBaud            (int baud)
{
    this->_baud = (DigiMeshBaudRates)baud;
}

void DigiMeshConfiguration::setPortName        (const std::string& portName)
{
    this->_portName = portName;
}

}



