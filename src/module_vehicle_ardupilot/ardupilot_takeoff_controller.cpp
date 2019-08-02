#include "ardupilot_takeoff_controller.h"

Ardupilot_TakeoffController::Ardupilot_TakeoffController(std::shared_ptr<DataInterface_MAVLINK::VehicleObject_MAVLINK> vehicleData) :
    Ardupilot_GeneralController(vehicleData),
    currentStateLogic(DISARMED)
{
    controllerType = CONTROLLER_TAKEOFF;
    vehicleMissionState = ArdupilotTargetProgess(2,10,10);
//    this->vehicleDataObject->state->vehicleAttitude.AddNotifier(this,[this]
//    {
//        m_LambdasToRun.push_back([this]{
//            std::cout<<"The attitude is functioning"<<std::endl;
//        });
//    });

//    this->vehicleDataObject->state->vehicleGlobalPosition.AddNotifier(this,[this]
//    {
//        m_LambdasToRun.push_back([this]{
//            std::cout<<"The position is functioning"<<std::endl;
//        });
//    });
}

Ardupilot_TakeoffController::~Ardupilot_TakeoffController() {

}

void Ardupilot_TakeoffController::initializeTakeoffSequence(const command_item::SpatialTakeoff &takeoff)
{
    missionItem_Takeoff = takeoff;
    std::string mode = vehicleDataObject->state->vehicleFlightMode.get().getFlightModeString();
    bool armed = vehicleDataObject->state->vehicleArm.get().getSystemArm();
    int vehicleID = vehicleDataObject->getSystemID();

    if((armed) && (mode == "GUIDED"))
    {
        currentStateLogic = ARMED_RIGHT_MODE;
        vehicleDataObject->m_CommandController->setSystemTakeoff(takeoff);
        //the vehicle is already armed and we can send the initial takeoff command

    }else if(armed){
        //the vehicle is armed and we should switch to guided
        currentStateLogic = ARMED_WRONG_MODE;
        int requiredMode = vehicleDataObject->state->vehicleFlightMode.get().getFlightModeFromString("GUIDED");
        vehicleDataObject->m_CommandController->setNewMode(requiredMode);
    }else
    {
        //we are in a mode that we can request the aircraft to arm
        currentStateLogic = DISARMED;
        command_item::ActionArm itemArm;
        itemArm.setTargetSystem(vehicleID);
        itemArm.setVehicleArm(true);
        vehicleDataObject->m_CommandController->setSystemArm(itemArm);
    }
}

void Ardupilot_TakeoffController::updateCommandACK(const mavlink_command_ack_t &cmdACK)
{
    if((cmdACK.command == 22) && (cmdACK.result == MAV_RESULT_ACCEPTED))
    {
        currentStateLogic = ALTITUDE_TRANSITION;
        //The takeoff sequence is being performed
    }
}

double Ardupilot_TakeoffController::distanceToTarget(){
    double distance = 0.0;
    switch(currentStateLogic)
    {
    case(ALTITUDE_TRANSITION):
    {
        DataState::StateGlobalPosition currentPosition = vehicleDataObject->state->vehicleGlobalPosition.get();
        DataState::StateGlobalPosition targetPosition(currentPosition.getX(),currentPosition.getY(),missionItem_Takeoff.position->getZ());
        distance  = fabs(currentPosition.deltaAltitude(targetPosition));
        MissionTopic::VehicleTargetTopic vehicleTarget(vehicleDataObject->getSystemID(), targetPosition, distance);
        m_CBTarget(m_FunctionTarget,vehicleTarget);
        break;
    }
    case(HORIZONTAL_TRANSITION):
    {
        DataState::StateGlobalPosition targetPosition(missionItem_Takeoff.position->getX(),missionItem_Takeoff.position->getY(),missionItem_Takeoff.position->getZ());
        distance = vehicleDataObject->state->vehicleGlobalPosition.get().distanceBetween3D(targetPosition);
        MissionTopic::VehicleTargetTopic vehicleTarget(vehicleDataObject->getSystemID(), targetPosition, distance);
        m_CBTarget(m_FunctionTarget,vehicleTarget);
        break;
    }
    default:
        break;
    }

    return distance;
}

void Ardupilot_TakeoffController::generateControl(const Data::ControllerState &currentState)
{
    switch(currentState){
    case Data::ControllerState::TRACKING:
    {
        //std::cout<<"I am still routing to the waypoint"<<std::endl;
        break;
    }
    case Data::ControllerState::HUNTING:
    {
        std::cout<<"I am hunting for the waypoint"<<std::endl;
        break;
    }
    case Data::ControllerState::ACHIEVED:
    {
        if((currentStateLogic == ALTITUDE_TRANSITION) && (missionItem_Takeoff.position->has3DPositionSet()))
        {
            currentStateLogic = HORIZONTAL_TRANSITION;
            command_item::SpatialWaypoint target;
            target.setTargetSystem(missionItem_Takeoff.getTargetSystem());
            target.setOriginatingSystem(missionItem_Takeoff.getOriginatingSystem());
            target.setPosition(missionItem_Takeoff.getPosition());
            vehicleDataObject->m_GuidedController->updateWaypointTarget(target);
        }
        else
            mToExit = true;

    }
    } //end of switch statement
}


void Ardupilot_TakeoffController::run()
{
    while(true)
    {
        this->RunPendingTasks();

        if(mToExit == true) {
            break;
        }

        std::string mode = vehicleDataObject->state->vehicleFlightMode.get().getFlightModeString();
        bool armed = vehicleDataObject->state->vehicleArm.get().getSystemArm();

        switch(currentStateLogic)
        {
        case DISARMED:
        {
            if(armed)
            {
                if(mode == "GUIDED")
                {
                    currentStateLogic = ARMED_RIGHT_MODE;
                    vehicleDataObject->m_CommandController->setSystemTakeoff(missionItem_Takeoff);
                }else{
                    currentStateLogic = ARMED_WRONG_MODE;
                    int requiredMode = vehicleDataObject->state->vehicleFlightMode.get().getFlightModeFromString("GUIDED");
                    vehicleDataObject->m_CommandController->setNewMode(requiredMode);
                }

            }
            break;
        }
        case ARMED_WRONG_MODE:
        {
            if(armed == false)
            {
                //for some reason we have taken a step backwards
                //the vehicle is no longer armed and therefore
                //we dont want to proceed
                mToExit = true;
            }
            if(mode == "GUIDED")
            {
                //we have made a good progression if we are in this phase
                currentStateLogic = ARMED_RIGHT_MODE;
                vehicleDataObject->m_CommandController->setSystemTakeoff(missionItem_Takeoff);
            }
            break;
        }
        case ALTITUDE_TRANSITION:
        {
            if((!armed) || (mode != "GUIDED"))
            {
                //for some reason we have taken a step backwards
                //the vehicle is no longer armed and therefore
                //we dont want to proceed
                mToExit = true;
            }
            controlSequence();
            break;
        }
        case HORIZONTAL_TRANSITION:
        {
            if((!armed) || (mode != "GUIDED"))
            {
                //for some reason we have taken a step backwards
                //the vehicle is  no longer armed and therefore
                //we dont want to proceed
                mToExit = true;
            }
            controlSequence();
            break;
        }

        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void Ardupilot_TakeoffController::controlSequence()
{
    //let us see how close we are to our target
//    double distance = distanceToTarget();
//    Data::ControllerState currentState = vehicleMissionState.updateMissionState(distance);
//    generateControl(currentState);
}
