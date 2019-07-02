#ifndef TRANSMIT_STATUS_H
#define TRANSMIT_STATUS_H

#include "I_AT_data.h"
#include <string>

#include "digi_common/transmit_status_types.h"
#include "digi_common/discovery_status_types.h"

namespace ATData
{

class TransmitStatus : public IATData {

public:

    TransmitStatus()
    {

    }

    TransmitStatus(const std::vector<uint8_t> &data)
    {
        DeSerialize(data);
    }

    uint8_t retries;
    TransmitStatusTypes status;
    DiscoveryTypes disoveryRequired;

private:

    virtual void DeSerialize(const std::vector<uint8_t> &data){
        retries = data[4];
        status = (TransmitStatusTypes)data[5];
        disoveryRequired = (DiscoveryTypes)data[6];
    }

    virtual std::vector<uint8_t> Serialize() const {
        throw "Output Only Data";
    }
};

}

#endif // TRANSMIT_STATUS_H
