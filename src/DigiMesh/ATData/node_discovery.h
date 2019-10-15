#ifndef NODE_H
#define NODE_H

#include "I_AT_data.h"
#include <string>

namespace ATData
{

class NodeDiscovery : public IATData {

public:

    NodeDiscovery()
    {

    }

    NodeDiscovery(const std::vector<uint8_t> &data)
    {
        DeSerialize(data);
    }

    uint16_t network_addr;
    uint64_t addr;
    std::string NI;
    uint16_t parent_net_addr;
    uint8_t device_type;
    uint16_t profile_id;
    uint16_t manufacturer_id;

private:

    virtual void DeSerialize(const std::vector<uint8_t> &data){
        network_addr = data[0] << 8 | data[1];

        addr = 0;
        for(int i = 0 ; i < 8 ; i++) {
            addr |= (((uint64_t)data[2+i]) << (8*(7-i)));
        }

        NI = "";
        size_t ni_end;
        for(size_t i = 10; ; i++) {
            if(data[i] == '\0') {
                ni_end = i;
                break;
            }
            NI += char(data[i]);
        }

        parent_net_addr = data[ni_end+1] << 8 | data[ni_end+2];
        device_type = data[ni_end+3];
        // one byte reserved for status
        profile_id = data[ni_end+5] << 8 | data[ni_end+6];
        manufacturer_id = data[ni_end+7] << 8 | data[ni_end+8];
    }

    virtual std::vector<uint8_t> Serialize() const {
        throw "Output Only Data";
    }
};

}

#endif // NODE_H
