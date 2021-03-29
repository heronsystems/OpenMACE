#ifndef AP_STATE_FLIGHT_AI_INITIALIZE_ROUTE_H
#define AP_STATE_FLIGHT_AI_INITIALIZE_ROUTE_H

#include <iostream>

#include <mavlink.h>

#include "base/math/frame_tf.h"
#include "base/trajectory/agent_parameters.h"
#include "base/trajectory/dubins.h"
#include "base/vehicle/vehicle_path_linear.h"

#include "data_generic_command_item/command_item_components.h"

#include "module_vehicle_ardupilot/flight_states/abstract_state_ardupilot.h"
#include "controllers/controllers_MAVLINK/controller_guided_target_item_waypoint.h"
#include "controllers/controllers_MAVLINK/commands/command_change_speed.h"
#include "controllers/controllers_MAVLINK/controller_mission.h"

#include "module_vehicle_arduplane/vehicle_object/vehicle_object_arduplane.h"

namespace ardupilot {
namespace state{

class AP_State_FlightAI_Initialize_ABORT;

class AP_State_FlightAI_Initialize_ROUTE : public AbstractStateArdupilot
{
public:
    AP_State_FlightAI_Initialize_ROUTE();

    void OnExit() override;

public:
    AbstractStateArdupilot* getClone() const override;

    void getClone(AbstractStateArdupilot** state) const override;

public:
    hsm::Transition GetTransition() override;

public:
    bool handleCommand(const std::shared_ptr<command_item::AbstractCommandItem> command) override;

    void Update() override;

    void OnEnter() override;

    void OnEnter(const std::shared_ptr<command_item::AbstractCommandItem> command) override;

    void OnEnter(const command_item::Action_InitializeTestSetup &initialization);

public:
    void handleTestProcedural(const command_item::Action_ProceduralCommand &command) override;

private:
    void receivedTargetState(const pose::CartesianPosition_3D &target);

private:
    void setupTrackingControllers();

    void setupAircraftSpeed();

    void setupAircraftRoute();

    void setupForAutoMission();
    bool runInAutoMode = false;

    void executeFrameComparison();

    MissionList routeViaDubinsSpline();

    MissionItem::MissionList routeStraightVectors();

public:

    bool notifyOfImpendingModeChange(const uint8_t &mode) override
    {
        UNUSED(mode);
        if((mode == PLANE_MODE::PLANE_MODE_AUTO) && (runInAutoMode))
        {
            //If the plane has moved to auto, this was intentional by this state and therefore nothing should be done.
            return false;
        }
        else
            setDesiredStateEnum(Data::MACEHSMState::STATE_FLIGHT_AI_INITIALIZE_ABORT);

        return false;
    }

private:
    static void staticCallbackFunction_VehicleTarget(void *p, const pose::GeodeticPosition_3D &projection, const pose::GeodeticPosition_3D &actual)
    {
        ((AP_State_FlightAI_Initialize_ROUTE *)p)->callbackFunction_VehicleTarget(projection, actual);
    }

    void callbackFunction_VehicleTarget(const pose::GeodeticPosition_3D &projection, const pose::GeodeticPosition_3D &actual);

private:
    pose::GeodeticPosition_3D _actualTarget;

    MAVLINKUXVControllers::ControllerGuidedTargetItem_WP* m_GeodeticCommsController;

private:
    command_item::Action_InitializeTestSetup m_SetupConditions;
};

} //end of namespace state
} //end of namespace ardupilot

#endif // AP_STATE_FLIGHT_AI_INITIALIZE_ROUTE_H
