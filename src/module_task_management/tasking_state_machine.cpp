#include "tasking_state_machine.h"

#include "data_generic_state_item/positional_aid.h"

#include "data_generic_command_item/spatial_items/spatial_rtl.h"
#include "data_generic_command_item/spatial_items/spatial_loiter_time.h"

#include "data_tasks/task_loiter_descriptor.h"
#include "data_tasks/task_survey_descriptor.h"

#include "module_task_management.h"

TaskingStateMachine::TaskingStateMachine(ModuleTaskManagement *callback) :
    m_moduleCallback(callback),
    m_taskDecomposer(m_moduleCallback)
{

}

TaskingState TaskingStateMachine::getCurrentState() const
{
    return m_currentState;
}

/*!
 * \brief Sets a new assignment queue
 * \param assignmentQueue Assignment queue
 */
void TaskingStateMachine::NewAssignmentQueue(const std::vector<TaskDescriptorPtr> &assignmentQueue)
{
    m_queuedTasks = assignmentQueue;

    if (m_queuedTasks.empty())
    {
        return;
    }

    if (m_currentTask == nullptr)
    {
        return;
    }

    if (m_queuedTasks.front()->getTaskKey() != m_currentTask->getTaskKey())
    {
        std::cout << "CANCEL CURRENT TASK" << std::endl;
        m_currentTask = nullptr;
        m_currentTaskCommands = nullptr;
    }
    else
    {
        m_queuedTasks.erase(m_queuedTasks.begin());
    }
}

/*!
 * \brief Advances the state machine
 * \param vehicleState Vehicle state
 * \param newState Whether the state machine advanced to a new state
 * \param newCommand Whether a new command needs to be issued
 */
void TaskingStateMachine::AdvanceStateMachine(const VehicleState &vehicleState, bool &newState, bool &newCommand)
{
    newState = false;
    newCommand = false;

    switch (m_currentState)
    {
        case TaskingState::Idle:
        {
            if (RefuelNeeded(vehicleState))
            {
//                std::cout << "Transitioned to REFUEL" << std::endl;
                m_currentState = TaskingState::Refuel;

                newState = true;
                newCommand = true;
            }
            else if (!m_queuedTasks.empty())
            {
//                std::cout << "Transitioned to TRANSITION" << std::endl;
                m_currentState = TaskingState::Transition;
                m_currentTask = m_queuedTasks.front();
                m_queuedTasks.erase(m_queuedTasks.begin());

                m_taskDecomposer.GetTransitionCommand(vehicleState, m_currentTask, m_transitionCommand);

                newState = true;
                newCommand = true;
            }
            break;
        }
        case TaskingState::Transition:
        {
            if (m_currentTask == nullptr)
            {
//                std::cout << "Transitioned to IDLE" << std::endl;
                m_currentState = TaskingState::Idle;
                newState = true;
                break;
            }
            if (m_currentTaskCommands == nullptr)
            {
                m_currentTaskCommands = m_taskDecomposer.Decompose(vehicleState, m_currentTask);
                Data::EnvironmentTime now;
                Data::EnvironmentTime::CurrentTime(Data::SYSTEMCLOCK, now);
                m_currentTaskCommands->lastCommandIssued = now;
                break;
            }
            if (m_currentTaskCommands->abortLambda != nullptr
                    && m_currentTaskCommands->abortLambda(vehicleState)) // Abort due to transition failure
            {
//                std::cout << "Transitioned to WAIT_FOR_ABORT" << std::endl;
                m_currentState = TaskingState::WaitForAuction_Abort;
                newState = true;
                break;
            }
            if (m_transitionCommand.commandDone(vehicleState))
            {
//                std::cout << "Transitioned to WAIT_FOR_START" << std::endl;
                m_currentState = TaskingState::WaitForAuction_Start;
                newState = true;
                m_currentTaskCommands->commandIndex = 0;
            }

            break;
        }
        case TaskingState::PerformingTask:
        {
            auto &command = m_currentTaskCommands->commands.at(m_currentTaskCommands->commandIndex);
            if (command.commandDone(vehicleState))
            {
                ++m_currentTaskCommands->commandIndex;
                if (m_currentTaskCommands->commandIndex >= m_currentTaskCommands->commands.size())
                {
//                    std::cout << "Transitioned to WAIT_FOR_COMPLETE" << std::endl;
                    m_currentState = TaskingState::WaitForAuction_Complete;
                    newState = true;
                }
                else
                {
                    newCommand = true;
                }

                Data::EnvironmentTime now;
                Data::EnvironmentTime::CurrentTime(Data::SYSTEMCLOCK, now);
                m_currentTaskCommands->lastCommandIssued = now;
            }
            else if (m_currentTaskCommands->abortLambda != nullptr && m_currentTaskCommands->abortLambda(vehicleState))
            {
//                std::cout << "Transitioned to WAIT_FOR_ABORT" << std::endl;
                m_currentState = TaskingState::WaitForAuction_Abort;
                newState = true;
            }
            break;
        }
        case TaskingState::Complete:
        case TaskingState::Abort:
        {
//            std::cout << "Transitioned to IDLE" << std::endl;
            m_currentState = TaskingState::Idle;
            m_currentTask = nullptr;

            m_currentTaskCommands = nullptr;
            newState = true;
            break;
        }
        case TaskingState::Refuel:
        {
            if (!RefuelNeeded(vehicleState))
            {
//                std::cout << "Transitioned to IDLE" << std::endl;
                m_currentState = TaskingState::Idle;
                newState = true;
            }
            break;
        }
        case TaskingState::WaitForAuction_Start:
        {
            if (m_auctionHasSignaled)
            {
//                std::cout << "Transitioned to PERFORMING TASK" << std::endl;
                m_currentState = TaskingState::PerformingTask;
                newState = true;
                newCommand = true;
                m_auctionHasSignaled = false;
            }
            if (m_currentTask == nullptr)
            {
//                std::cout << "Transitioned to IDLE" << std::endl;
                m_currentState = TaskingState::Idle;
                newState = true;
            }
            break;
        }
        case TaskingState::WaitForAuction_Complete:
        {
            if (m_auctionHasSignaled)
            {
//                std::cout << "Transitioned to COMPLETE" << std::endl;
                m_currentState = TaskingState::Complete;
                newState = true;
                m_auctionHasSignaled = false;
            }
            if (m_currentTask == nullptr)
            {
//                std::cout << "Transitioned to IDLE" << std::endl;
                m_currentState = TaskingState::Idle;
                newState = true;
            }
            break;
        }
        case TaskingState::WaitForAuction_Abort:
        {
            if (m_auctionHasSignaled)
            {
//                std::cout << "Transitioned to ABORT" << std::endl;
                m_currentState = TaskingState::Abort;
                newState = true;
                m_auctionHasSignaled = false;
            }
            if (m_currentTask == nullptr)
            {
//                std::cout << "Transitioned to IDLE" << std::endl;
                m_currentState = TaskingState::Idle;
                newState = true;
            }
            break;
        }
    }
}

/*!
 * \brief Retrieves the next vehicle command.
 * \details Except for if the refuel state is being used, this will current always be a GoTo command.
 * \return Vehicle command
 */
CommandItem::AbstractCommandItemPtr TaskingStateMachine::GetNextVehicleCommand()
{
    switch (m_currentState)
    {
        case TaskingState::Transition:
            if (m_currentTask == nullptr)
                return nullptr;
            return m_transitionCommand.command;
        case TaskingState::PerformingTask:
            if (m_currentTaskCommands->commandIndex >= m_currentTaskCommands->commands.size())
                return nullptr;
            return m_currentTaskCommands->commands.at(m_currentTaskCommands->commandIndex).command;
        case TaskingState::Refuel:
        {
            std::shared_ptr<CommandItem::SpatialRTL> rtl = std::make_shared<CommandItem::SpatialRTL>();
            return rtl;
        }
        case TaskingState::Abort:
        case TaskingState::Complete:
        case TaskingState::Idle:
        case TaskingState::WaitForAuction_Start:
        case TaskingState::WaitForAuction_Complete:
        case TaskingState::WaitForAuction_Abort:
            return nullptr;
    }
}
void TaskingStateMachine::setOrigin(const DataState::StateGlobalPosition &origin)
{
    m_origin = origin;
}

/*!
 * \brief Called when the auction has signaled that the status update was seen
 */
void TaskingStateMachine::ReceiveAuctionSignal()
{
    m_auctionHasSignaled = true;
}

void TaskingStateMachine::setRequiredDistanceToTarget(double requiredDistanceToTarget)
{
    m_taskDecomposer.setRequiredDistanceToTarget(requiredDistanceToTarget);
}

void TaskingStateMachine::setMinTurnRadius(double minTurnRadius)
{
    m_taskDecomposer.setMinTurnRadius(minTurnRadius);
}

void TaskingStateMachine::setNominalSpeed(double nominalSpeed)
{
    m_taskDecomposer.setNominalSpeed(nominalSpeed);
}

void TaskingStateMachine::setNominalClimbRate(double nominalClimbRate)
{
    m_taskDecomposer.setNominalClimbRate(nominalClimbRate);
}

void TaskingStateMachine::setNominalAcceleration(double nominalAcceleration)
{
    m_taskDecomposer.setNominalAcceleration(nominalAcceleration);
}

void TaskingStateMachine::setDefaultAltitude(double defaultAltitude)
{
    m_taskDecomposer.setDefaultAltitude(defaultAltitude);
}

void TaskingStateMachine::setTakeoffAltitude(double takeoffAltitude)
{
    m_taskDecomposer.setTakeoffAltitude(takeoffAltitude);
}

void TaskingStateMachine::setCanFly(bool canFly)
{
    m_taskDecomposer.setCanFly(canFly);
}

void TaskingStateMachine::setCanBeStationary(bool canBeStationary)
{
    m_taskDecomposer.setCanBeStationary(canBeStationary);
}

TaskDescriptorPtr TaskingStateMachine::getCurrentTask() const
{
    return m_currentTask;
}

/*!
 * \brief Creates a new task from the recently aborted task.
 * \details Allows an aborted task to be either recreated entirely, recreated partially (due to partial completion),
 * or abandoned.
 *
 * NOTE: This currently unused and unimplemented.
 *
 * \param creatorID Creator ID
 * \param taskID Task ID
 * \return New task, or nullptr if no new task should be created
 */
TaskDescriptorPtr TaskingStateMachine::CreateNewTaskFromAbortedTask(uint64_t creatorID, uint8_t taskID)
{
    // TODO Used to generate a new task based on the current task, which was aborted but potentially partially completed.
    // Currently, an aborted task will always be ignored.
    return nullptr;
}

/*!
 * \brief Whether the vehicle needs to be refueled.
 * \details Currently unused. When used, will allow the vehicle to go into the Refuel state, where a RTL command
 * would be issued in order for the vehicle to refuel.
 * \param vehicleState Vehicle state
 * \return Whether the vehicle needs to be refueled
 */
bool TaskingStateMachine::RefuelNeeded(const VehicleState &vehicleState)
{
    return false;
}

/*!
 * \return Whether the current task has been completed (all commands finished)
 */
bool TaskingStateMachine::TaskCompleted()
{
    return (m_currentTaskCommands->commands.size() == m_currentTaskCommands->commandIndex);
}

