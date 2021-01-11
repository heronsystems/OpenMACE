#ifndef ABSTRACT_STATE_ARDUCOPTER_H
#define ABSTRACT_STATE_ARDUCOPTER_H

#include <iostream>
#include <thread>

#include "common/class_forward.h"
#include "data_generic_command_item/abstract_command_item.h"

#include "common/hsm.h"
#include "data/mace_hsm_state.h"

#include "../vehicle_object/vehicle_object_ardupilot.h"

#include "common/logging/macelog.h"

#include "module_vehicle_MAVLINK/controllers/controller_write_event_to_log.h"

//forward declaration of the class

namespace ardupilot{
namespace state{

class AbstractStateArdupilot : public hsm::StateWithOwner<VehicleObject_Ardupilot>
{
public:
    AbstractStateArdupilot();

    AbstractStateArdupilot(const Data::MACEHSMState &enteredState);

    AbstractStateArdupilot(const AbstractStateArdupilot &copy);

    /**
      */
    virtual ~AbstractStateArdupilot();

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
    virtual AbstractStateArdupilot* getClone() const = 0;

    /**
     * @brief getClone
     * @param state
     */
    virtual void getClone(AbstractStateArdupilot** state) const = 0;

public:

    virtual bool handleMAVLINKMessage(const mavlink_message_t &msg);

    void setCurrentCommand(const std::shared_ptr<AbstractCommandItem> command);

    virtual bool handleCommand(const std::shared_ptr<AbstractCommandItem> command);

public:
    virtual void initializeForTestEvaluation(const command_item::Action_InitializeTestSetup &initialization)
    {
        UNUSED(initialization);
    }

    virtual void handleTestProcedural(const command_item::Action_ProceduralCommand &command)
    {
        UNUSED(command);
    }

public:
    virtual void OnExit();

public:
    virtual void OnEnter(const std::shared_ptr<AbstractCommandItem> command) = 0;

public:

public:
    Data::MACEHSMState getCurrentStateEnum() const
    {
        return _currentState;
    }

    Data::MACEHSMState getDesiredStateEnum() const
    {
        return _desiredState;
    }

    void setCurrentStateEnum(const Data::MACEHSMState &state)
    {
        _currentState = state;
    }

    void setDesiredStateEnum(const Data::MACEHSMState &state)
    {
        _desiredState = state;
    }

public:
    virtual bool shouldExecuteModeTransition(const uint8_t &mode)
    {
        UNUSED(mode);

        return true;
    }

protected:
    void clearCommand();

    virtual void updateOwner_ProgressionOfHSM();

protected:
    MAVLINKUXVControllers::ControllerSystemMode* prepareModeController();
    std::mutex m_mutex_ModeController;
    std::condition_variable m_condition_ModeController;
    bool m_oldModeControllerShutdown = false;

protected:
    std::shared_ptr<command_item::AbstractCommandItem> currentCommand;
    bool currentCommandSet;

protected:
    Data::MACEHSMState _currentState;
    Data::MACEHSMState _desiredState;

};

} //end of namespace ardupilot
} //end of namespace state

#endif // ABSTRACT_STATE_ARDUCOPTER_H
