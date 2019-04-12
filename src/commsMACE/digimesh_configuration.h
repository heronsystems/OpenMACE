#ifndef DIGIMESH_CONFIGURATION_H
#define DIGIMESH_CONFIGURATION_H

#include <string>

#include "link_configuration_mace.h"

#include "commsmace_global.h"
#include "common/common.h"

#include "mace_digimesh_wrapper.h"

namespace CommsMACE
{

class COMMSMACESHARED_EXPORT DigiMeshConfiguration : public LinkConfiguration
{
public:

    DigiMeshConfiguration(const std::string& name);
    DigiMeshConfiguration(DigiMeshConfiguration* copy);

    ~DigiMeshConfiguration();


    DigiMeshBaudRates  baud() const                   { return _baud; }
    const std::string portName() const  { return _portName; }

    void setBaud            (int baud);
    void setPortName        (const std::string& portName);

private:

    DigiMeshBaudRates _baud;
    std::string _portName;
};

}

#endif // DIGIMESH_CONFIGURATION_H
