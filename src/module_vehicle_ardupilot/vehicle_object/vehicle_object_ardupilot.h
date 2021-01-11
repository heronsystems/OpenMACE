#ifndef VEHICLE_OBJECT_ARDUPILOT_H
#define VEHICLE_OBJECT_ARDUPILOT_H

#include "module_vehicle_MAVLINK/vehicle_object/mavlink_vehicle_object.h"

#include "ardupilot_component_operating_mode.h"

#include "base/trajectory/agent_parameters.h"
#include "base/vehicle/vehicle_path_linear.h"

#include "common/transmit_queue.h"
#include "data/mace_hsm_state.h"
#include "data_generic_mission_item_topic/vehicle_target_topic.h"

#include "mace_core/module_characteristics.h"

#include "trajectory_control/virtual_target_controller.h"


typedef void(*CallbackFunctionPtr_VehicleTarget)(void*, MissionTopic::VehicleTargetTopic&);

class VehicleObject_Ardupilot : public MavlinkVehicleObject
{
public:
    VehicleObject_Ardupilot(CommsMAVLINK* commsObj, const MaceCore::ModuleCharacteristic &module, const int &mavlinkID);

    virtual ~VehicleObject_Ardupilot() = default;

    CallbackInterface_MAVLINKVehicleObject* getMAVLINKCallback()
    {
        return m_CB;
    }


    void connectTargetCallback(CallbackFunctionPtr_VehicleTarget cb, void *p)
    {
        m_CBTarget = cb;
        m_FunctionTarget = p;
    }

    void callTargetCallback(MissionTopic::VehicleTargetTopic &topic)
    {
        m_CBTarget(m_FunctionTarget,topic);
    }

    void publishVehicleTrajectory(const int &systemID, const VehiclePath_Linear &trajectory)
    {
        if(m_CB != nullptr)
            m_CB->cbi_VehicleTrajectory(systemID, trajectory);
    }

    void publishVehicleHome(const int &systemID, const command_item::SpatialHome &home)
    {
        if(m_CB != nullptr)
            m_CB->cbi_VehicleHome(systemID, home);
    }

    void publishVehicleAutoMission(const int &systemID, const MissionItem::MissionList &missionList)
    {
        if(m_CB != nullptr)
            m_CB->cbi_VehicleMission(systemID, missionList);
    }

protected:
    CallbackFunctionPtr_VehicleTarget m_CBTarget;
    void *m_FunctionTarget;

public:
    ARDUPILOTComponent_OperatingMode* m_ArdupilotMode;
    AgentParams m_AgentParams;
};

#endif // ARDUPILOT_VEHICLE_OBJECT_H
