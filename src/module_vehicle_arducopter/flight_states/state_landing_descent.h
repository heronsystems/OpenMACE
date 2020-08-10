#ifndef STATE_LANDING_DESCENT_H
#define STATE_LANDING_DESCENT_H

#include <mavlink.h>

#include "abstract_state_arducopter.h"

#include "../arducopter_target_progess.h"

#include "module_vehicle_MAVLINK/controllers/controller_guided_mission_item.h"

namespace arducopter{
namespace state{

class State_LandingComplete;

class State_LandingDescent : public AbstractStateArducopter
{
public:
    State_LandingDescent();

public:
    AbstractStateArducopter* getClone() const override;

    void getClone(AbstractStateArducopter** state) const override;

public:
    hsm::Transition GetTransition() override;

public:
    bool handleCommand(const std::shared_ptr<AbstractCommandItem> command) override;

    void Update() override;

    void OnEnter() override;

    void OnEnter(const std::shared_ptr<AbstractCommandItem> command) override;

    void OnExit() override;

private:
    ArducopterTargetProgess guidedProgress;
};

} //end of namespace arducopter
} //end of namespace state

#endif // STATE_LANDING_DESCENT_H
