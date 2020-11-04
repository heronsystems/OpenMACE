#ifndef VOID_H
#define VOID_H


#include "I_AT_data.h"
#include "common/common.h"

namespace ATData
{

class Void : public IATData {
private:

public:

    Void(const std::vector<uint8_t> &data)
    {
        UNUSED(data);
    }

    Void()
    {
    }


    virtual void DeSerialize(const std::vector<uint8_t> &data)
    {
        UNUSED(data);
    }

    virtual std::vector<uint8_t> Serialize() const {
        std::vector<uint8_t> buf;
        return buf;
    }
};

}

#endif // VOID_H
