#ifndef INTEGER_H
#define INTEGER_H

#include "I_AT_data.h"

namespace ATData
{

template <typename T>
class Integer : public IATData {
private:

    T mInt;

public:

    Integer(const std::vector<uint8_t> &data)
    {
        DeSerialize(data);
    }

    Integer(const int &data)
    {
        mInt = data;
    }

    operator int&(){return mInt;}


    virtual void DeSerialize(const std::vector<uint8_t> &data){
        throw "Not Implimented";
        std::string str = "";
        for(size_t i = 0 ; i < data.size()-1 ; i++) {
            str += data[i];
        }

        this->mInt = std::stoi(str);
    }

    virtual std::vector<uint8_t> Serialize() const {
        int numBytes = sizeof(T);

        std::vector<uint8_t> buf;
        for(int i = 0 ; i < numBytes ; i++) {
            uint8_t a = (mInt & (0xFF << 8*i)) >> 8*i;
            buf.push_back(a);
        }

        return buf;
    }
};

}

#endif // INTEGER_H
