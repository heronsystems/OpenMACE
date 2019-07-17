#ifndef STATE_H
#define STATE_H

#include <string>
#include "base/base_global.h"

#include "common/class_forward.h"

#include "data/environment_time.h"

//This class is intended to be abstract as well

namespace mace {
namespace state_space {

MACE_CLASS_FORWARD(State);

class BASESHARED_EXPORT State{

public:
    /**
      */
    State() = default;

    State(const State &copy) = default;

    /**
      */
    virtual ~State() = default;

public:
    virtual std::string printInfo() const
    {
        std::string rtn = "";
        return rtn;
    }

public:
    /**
     *
     */
    template <class T>
    const T *stateAs() const
    {
        //ensure that we are attempting to cast it to a type of state
        return static_cast<const T *>(this);
    }

    /**
     *
     */
    template <class T>
    T *stateAs()
    {
        //ensure that we are attempting to cast it to a type of state
        return static_cast<T *>(this);
    }

public:
    //an option to avoid the virtual may be something like this
//    template <class T>
//    State* getClone() const
//    {
//        return new T(*this->as<T>());
//    }

    /**
     * @brief getClone
     * @return
     */
    virtual State* getStateClone() const = 0;

    /**
     * @brief getClone
     * @param state
     */
    virtual void getStateClone(State** state) const = 0;

};

MACE_CLASS_FORWARD(State_TimeExpanded);

class BASESHARED_EXPORT State_TimeExpanded
{
public:
    State_TimeExpanded():
        m_State(nullptr)
    {

    }

    State_TimeExpanded(const State* state)
    {
        this->setState(state);
    }

    ~State_TimeExpanded()
    {
        delete m_State; //first free the memory
        m_State = nullptr;
    }

    State_TimeExpanded(const State_TimeExpanded &copy)
    {
        this->m_State = copy.m_State->getStateClone();
        this->m_Time = copy.m_Time;
    }

    void setState(const State* state)
    {
        m_State = state->getStateClone();
    }


public:
    State* m_State;
    Data::EnvironmentTime m_Time;
};

} //end of namespace state_space
} //end of namespace mace

#endif // STATE_H
