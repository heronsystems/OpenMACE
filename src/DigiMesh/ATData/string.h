#ifndef STRING_H
#define STRING_H

#include "I_AT_data.h"
#include <string>

namespace ATData
{

class String : public std::string, public IATData {

public:

    String(const std::vector<uint8_t> &data)
    {
        DeSerialize(data);
    }

    String(const char* str) {
        this->assign(str);
    }

    virtual void DeSerialize(const std::vector<uint8_t> & data){
        std::string str = "";
        for(size_t i = 0 ; i < data.size() ; i++) {
            str += data[i];
        }
        this->assign(str);
    }

    virtual std::vector<uint8_t> Serialize() const {
        std::vector<uint8_t> buf;
        for(size_t i = 0 ; i < length() ; i++) {
            buf.push_back(this->at(i));
        }
        buf.push_back('\0');
        return buf;
    }
};

}

#endif // STRING_H
