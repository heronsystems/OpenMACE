#ifndef ABSTRACT_STATE_ARDUCOPTER_H
#define ABSTRACT_STATE_ARDUCOPTER_H

#include <iostream>
#include <thread>

#include "common/class_forward.h"
#include "data_generic_command_item/abstract_command_item.h"

#include "arducopter_hsm.h"
#include "arducopter_state_types.h"

#include "../vehicle_object/arducopter_vehicle_object.h"

#include "common/logging/macelog.h"

//forward declaration of the class

namespace arducopter{
namespace state{


class AbstractStateArducopter : public hsm::StateWithOwner<ArducopterVehicleObject>
{
public:
    AbstractStateArducopter();

    AbstractStateArducopter(const AbstractStateArducopter &copy);

    /**
      */
    virtual ~AbstractStateArducopter();

public:
    /**
     *
     */
    template <class T>
    const T *as() const
    {
        //ensure that we are attempting to cast it to a type of state
        return static_cast<const T *>(this);
    }

    /**
     *
     */
    template <class T>
    T *as()
    {
        //ensure that we are attempting to cast it to a type of state
        return static_cast<T *>(this);
    }

    /**
     * @brief getClone
     * @return
     */
    virtual AbstractStateArducopter* getClone() const = 0;

    /**
     * @brief getClone
     * @param state
     */
    virtual void getClone(AbstractStateArducopter** state) const = 0;

public:

    virtual bool handleMAVLINKMessage(const mavlink_message_t &msg);

    void setCurrentCommand(const std::shared_ptr<AbstractCommandItem> command);

    virtual bool handleCommand(const std::shared_ptr<AbstractCommandItem> command);

public:
    virtual void OnExit();

public:
    virtual void OnEnter(const std::shared_ptr<AbstractCommandItem> command) = 0;

protected:
    void clearCommand();

protected:

    std::shared_ptr<command_item::AbstractCommandItem> currentCommand;
    bool currentCommandSet;

};

} //end of namespace arducopter
} //end of namespace state

#endif // ABSTRACT_STATE_ARDUCOPTER_H
