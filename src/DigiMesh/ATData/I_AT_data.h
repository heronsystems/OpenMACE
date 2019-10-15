#ifndef IATDATA_H
#define IATDATA_H

#include <vector>
#include <stdint.h>

namespace ATData
{

class IATData {
public:

    virtual void DeSerialize(const std::vector<uint8_t> &) = 0;

    virtual std::vector<uint8_t> Serialize() const = 0;

private:
};

}

#endif // IATDATA_H
