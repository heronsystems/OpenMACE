#ifndef CALLBACK_H
#define CALLBACK_H

#include "ATData/index.h"

class ICallback {
public:
    virtual bool IsSet() const = 0;

    virtual void Call(int frame_id, const std::vector<uint8_t> &data) = 0;
};

template <typename T>
class Callback : public ICallback {
public:
    Callback() {
        static_assert(std::is_base_of<ATData::IATData, T>::value, "T must be a descendant of ATDATA::IATDATA");
        m_Func = NULL;
    }

    Callback(const std::function<void(int, const T&)> &func) {
        static_assert(std::is_base_of<ATData::IATData, T>::value, "T must be a descendant of ATDATA::IATDATA");
        m_Func = new std::function<void(int, const T&)>(func);
    }

    ~Callback() {
        delete m_Func;
    }

    virtual void Call(int frame_id, const std::vector<uint8_t> &data) {
        (*m_Func)(frame_id, T(data));
    }

    virtual bool IsSet() const {
        if(m_Func == NULL) {
            return false;
        }
        return true;
    }


    std::function<void(int, const T&)> *m_Func;
};

#endif // CALLBACK_H
