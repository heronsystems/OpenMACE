#ifndef ARDUPLANE_VEHICLE_OBJECT_H
#define ARDUPLANE_VEHICLE_OBJECT_H

#include "module_vehicle_MAVLINK/vehicle_object/mavlink_vehicle_object.h"

#include "arduplane_component_flight_mode.h"
#include "../flight_states/arduplane_state_types.h"

#include "data_generic_mission_item_topic/vehicle_target_topic.h"

#include "common/transmit_queue.h"
#include "mace_core/module_characteristics.h"

typedef void(*CallbackFunctionPtr_VehicleTarget)(void*, MissionTopic::VehicleTargetTopic&);

class ArduplaneVehicleObject : public MavlinkVehicleObject
{
public:
    ArduplaneVehicleObject(CommsMAVLINK* commsObj, const MaceCore::ModuleCharacteristic &module, const int &mavlinkID);

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


protected:
    CallbackFunctionPtr_VehicleTarget m_CBTarget;
    void *m_FunctionTarget;

public:
    ARDUPLANEComponent_FlightMode arduplaneMode;


};

#endif // ARDUPLANE_VEHICLE_OBJECT_H
