#ifndef ACTION_BASE_H
#define ACTION_BASE_H

#include <functional>

#include <mavlink.h>

#include "mace_core/module_characteristics.h"

namespace Controllers {

template<typename COMPONENT_KEY, typename MESSAGE_TYPE, typename MSG_TYPE>
class BaseEncode
{
public:
    std::function<void(COMPONENT_KEY, uint8_t, MESSAGE_TYPE*, const MSG_TYPE*)> m_EncodeChanFunc;

    BaseEncode(const std::function<void(COMPONENT_KEY, uint8_t, MESSAGE_TYPE*, const MSG_TYPE*)> &encode) :
        m_EncodeChanFunc(encode)
    {

    }
};


template<typename COMPONENT_KEY, typename MESSAGE_TYPE, typename MSG_TYPE>
class BaseDecode
{
public:
    std::function<void(const MESSAGE_TYPE*, MSG_TYPE*)> m_DecodeFunc;

    BaseDecode(const std::function<void(const MESSAGE_TYPE*, MSG_TYPE*)> &decode) :
        m_DecodeFunc(decode)
    {

    }
};





template<typename CONTROLLER_TYPE>
class ActionBase
{
protected:

    CONTROLLER_TYPE *m_Controller;

    /*
    std::function<void(COMPONENT_KEY, uint8_t, MESSAGE_TYPE*, const MSG_TYPE*)> m_EncodeChanFunc;
    std::function<void(const MESSAGE_TYPE*, MSG_TYPE*)> m_DecodeFunc;
    */

protected:


public:

    ActionBase()
    {
        throw std::runtime_error("Default Constructor not supported");
    }

    ActionBase(CONTROLLER_TYPE *controller) :
        m_Controller(controller)
    {
    }



};

}

#endif // ACTION_BASE_H
