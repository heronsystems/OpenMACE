#ifndef TASKING_STATE_MACHINE_H
#define TASKING_STATE_MACHINE_H

#include <functional>
#include <vector>
#include <memory>
#include <unordered_set>

#include "data_auctioneer/state/vehicle_state.h"
#include "data_auctioneer/assignment_task_queue.h"

#include "data_tasks/task_descriptor.h"

#include "data_generic_state_item/state_local_position.h"
#include "data_generic_state_item/state_global_position.h"

#include "data_generic_command_item/abstract_command_item.h"

#include "data/environment_time.h"

#include "task_decomposer.h"

class ModuleTaskManagement;

typedef enum class TaskingState
{
    Idle,
    Refuel,
    Transition,
    WaitForAuction_Start,
    PerformingTask,
    WaitForAuction_Complete,
    Complete,
    WaitForAuction_Abort,
    Abort
} TaskingState;

class TaskingStateMachine
{
public:
    TaskingStateMachine(ModuleTaskManagement *callback);

    TaskingState getCurrentState() const;

    void NewAssignmentQueue(const std::vector<TaskDescriptorPtr> &assignmentQueue);

    void AdvanceStateMachine(const VehicleState &vehicleState, bool &newState, bool &newCommand);

    CommandItem::AbstractCommandItemPtr GetNextVehicleCommand();


    TaskDescriptorPtr getCurrentTask() const;

    // If a task was aborted, this function is used to possibly create a new task to ensure it gets finished.
    // The new task may be a copy of the previous task, a task representing the uncompleted portion of the
    // previous task, or the task may be aborted entirely.
    TaskDescriptorPtr CreateNewTaskFromAbortedTask(uint64_t creatorID, uint8_t taskID);

    void setOrigin(const DataState::StateGlobalPosition &origin);

    void ReceiveAuctionSignal();

    void setRequiredDistanceToTarget(double requiredDistanceToTarget);
    void setMinTurnRadius(double minTurnRadius);
    void setNominalSpeed(double nominalSpeed);
    void setNominalClimbRate(double nominalClimbRate);
    void setNominalAcceleration(double nominalAcceleration);


    void setDefaultAltitude(double defaultAltitude);
    void setTakeoffAltitude(double takeoffAltitude);

    void setCanFly(bool canFly);
    void setCanBeStationary(bool canBeStationary);

private:



    // The tasks may be re-ordered from the order set by the auction. Does not include the current task.
    std::vector<TaskDescriptorPtr> m_queuedTasks;

    // Set on advancing the state machine from Idle to Transition. Reset to null on advancing to Idle
    TaskDescriptorPtr m_currentTask = nullptr;


    DataState::StateLocalPosition m_transitionDestination;
    bool m_transitionDestinationIs2D;

// List of commands - commands needed to perform a task, index representing current command.
    std::shared_ptr<CommandList> m_currentTaskCommands;

    TaskingState m_currentState = TaskingState::Idle;

    DataState::StateGlobalPosition m_origin;

    bool m_auctionHasSignaled = false;

    bool RefuelNeeded(const VehicleState &vehicleState);

    bool TaskCompleted();

    ModuleTaskManagement *m_moduleCallback;
    TaskDecomposer m_taskDecomposer;

    CommandData m_transitionCommand;
    Data::EnvironmentTime m_lastTransitionCommand;
};


#endif // TASKING_STATE_MACHINE_H
