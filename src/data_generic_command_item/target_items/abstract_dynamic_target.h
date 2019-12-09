#ifndef ABSTRACT_DYNAMIC_TARGET_H
#define ABSTRACT_DYNAMIC_TARGET_H

#include <string>
#include <limits>
#include <cmath>

namespace command_target {

class DynamicTarget
{
public:
    enum class TargetTypes: uint8_t
    {
        KINEMATIC = 0,
        ORIENTATION = 1
    };

public:
    DynamicTarget();

    virtual ~DynamicTarget();

public:
    virtual TargetTypes getTargetType() const = 0;

public:
    /**
         *
         */
    template <class T>
    const T *targetAs() const
    {
        //ensure that we are attempting to cast it to a type of state
        return static_cast<const T *>(this);
    }

    /**
         *
         */
    template <class T>
    T *targetAs()
    {
        //ensure that we are attempting to cast it to a type of state
        return static_cast<T *>(this);
    }

    /**
         * @brief getClone
         * @return
         */
    virtual DynamicTarget* getDynamicTargetClone() const = 0;

    /**
         * @brief getClone
         * @param state
         */
    virtual void getDynamicTargetClone(DynamicTarget** target) const = 0;

};

} //end of namespace command_target

#endif // ABSTRACT_DYNAMIC_TARGET_H
