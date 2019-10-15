#ifndef MESSAGE_H
#define MESSAGE_H

#include "I_AT_data.h"
#include <string>

namespace ATData
{

class Message : public IATData {

public:

    Message(const std::vector<uint8_t> &data, const bool &explicitFrame = false)
    {
        if(!explicitFrame)
            DeSerialize(data);
        else
            DeSerializeExplicit(data);
    }

    uint64_t addr;
    bool broadcast;
    std::vector<uint8_t> data;

private:

    virtual void DeSerialize(const std::vector<uint8_t> &msg){
        addr = 0;
        for(int i = 0 ; i < 8 ; i++) {
            addr |= (((uint64_t)msg[1+i]) << (8*(7-i)));
        }

        broadcast = (msg[11]& 0x03) == 0x02;


        for(size_t i = 12 ; i < msg.size() ; i++) {
            data.push_back(msg.at(i));
        }
    }

    virtual void DeSerializeExplicit(const std::vector<uint8_t> &msg){
        addr = 0;
        for(int i = 0 ; i < 8 ; i++) {
            addr |= (((uint64_t)msg[1+i]) << (8*(7-i)));
        }

        broadcast = msg[17] == 0x02;

        for(size_t i = 18 ; i < msg.size() ; i++) {
            data.push_back(msg.at(i));
        }
    }


    virtual std::vector<uint8_t> Serialize() const {
        throw "Output Only Data";
    }
};

}
#endif // MESSAGE_H
