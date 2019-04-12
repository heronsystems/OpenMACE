#ifndef TASK_DECOMPOSER_H
#define TASK_DECOMPOSER_H

#include <vector>
#include "data_tasks/task_descriptor.h"
#include "data_tasks/task_loiter_descriptor.h"
#include "data_tasks/task_survey_descriptor.h"
#include "data_tasks/task_key.h"

#include "data/loiter_direction.h"

#include "data_auctioneer/state/vehicle_state.h"

#include "data_generic_state_item/state_local_position.h"
#include "data_generic_state_item/state_global_position.h"

#include "data_generic_command_item/abstract_command_item.h"

class ModuleTaskManagement;

/*!
 * \brief The CommandData struct stores data for a command to be issued to a vehicle
 */
typedef struct CommandData
{
    CommandItem::AbstractCommandItemPtr command; /*!< Command */
    double expectedDuration; /*<! Expected duration of the command */
    std::function<bool(const VehicleState &vehicleState)> commandDone = nullptr; /*!< Function which returns if a command is done */
} CommandData;

/*!
 * \brief The CommandList struct stores data for a list of commands needed to complete a task
 */
typedef struct CommandList
{
    std::vector<CommandData> commands; /*!< List of data for each command */
    int commandIndex = -1; /*!< Current command index. -1 if transitioning to the task */
    double expectedTransitionTime; /*!< Expected duration to transition to the task start */
    double expectedDuration; /*!< Expected duration for the task */
    Data::EnvironmentTime lastCommandIssued; /*!< When the last command was issued */
    std::function<bool(const VehicleState &vehicleState)> abortLambda = nullptr; /*!< Function which returns whether the task should be aborted */
} CommandList;

/*!
 * \brief The TaskDecomposer class decomposes tasks into commands
 */
class TaskDecomposer
{
public:
    TaskDecomposer(ModuleTaskManagement *callback);


    double getMinTurnRadius() const;
    void setMinTurnRadius(double minTurnRadius);

    double getNominalSpeed() const;
    void setNominalSpeed(double nominalSpeed);

    double getNominalAcceleration() const;
    void setNominalAcceleration(double nominalAcceleration);

    bool getCanFly() const;
    void setCanFly(bool canFly);

    double getDefaultAltitude() const;
    void setDefaultAltitude(double defaultAltitude);

    bool getCanBeStationary() const;
    void setCanBeStationary(bool canBeStationary);

    double getRequiredDistanceToTarget() const;
    void setRequiredDistanceToTarget(double requiredDistanceToTarget);

    DataState::StateGlobalPosition getOrigin() const;
    void setOrigin(const DataState::StateGlobalPosition &origin);

    double getNominalClimbRate() const;
    void setNominalClimbRate(double nominalClimbRate);




    void GetTransitionCommand(const VehicleState &vehicleState, const TaskDescriptorPtr &task, CommandData &commandData);
    std::shared_ptr<CommandList> Decompose(const VehicleState &vehicleState, const TaskDescriptorPtr &task);

    double getTakeoffAltitude() const;
    void setTakeoffAltitude(double takeoffAltitude);

private:
    typedef struct TaskDataCache
    {
        TaskKey key;
        DataState::StateGlobalPosition startPosition;
        bool startIs2D;
        double expectedTransitionTime;
    } TaskDataCache;

    typedef struct TaskLoiterCache : public TaskDataCache
    {
        Data::LoiterDirection direction;
    } TaskLoiterCache;

    typedef struct TaskSurveyCache : public TaskDataCache
    {
        std::vector<DataState::StateGlobalPosition> boundary;
        int startIndex;
    } TaskSurveyCache;

    //Vehicle constants, used to estim
    double m_minTurnRadius = 0; // Minimum turn radius when the vehicle turns at its maximum rate.
    double m_nominalSpeed = 5; // Used to estimate times
    double m_nominalAcceleration = 2;
    double m_nominalClimbRate = 2;
    bool m_canFly = true;
    double m_defaultAltitude = 10;
    bool m_canBeStationary = true; // False indicates that the vehicle must be in motion during operation, such as a fixed wing UAV

    double m_requiredDistanceToTarget = 2.0;

    double m_takeoffAltitude = 2;

    std::unique_ptr<TaskDataCache> m_taskDataCache = nullptr;

    ModuleTaskManagement *m_moduleCallback = nullptr;

    void GetTransitionCommand_Loiter(const VehicleState &vehicleState, const TaskDescriptorPtr &task, CommandData &commandData);
    void GetTransitionCommand_Survey(const VehicleState &vehicleState, const TaskDescriptorPtr &task, CommandData &CommandData);

    template <typename PolygonType>
    void GetTransitionCommand_Survey(const VehicleState &vehicleState,
                                     const AbstractTaskSurveyDescriptor *abstractSurvey,
                                     CommandData &CommandData,
                                     TaskSurveyCache *surveyCache);

    std::shared_ptr<CommandList> Decompose_Loiter(const VehicleState &vehicleState, const AbstractTaskLoiterDescriptor *task);
    std::shared_ptr<CommandList> Decompose_Survey(const VehicleState &vehicleState, const AbstractTaskSurveyDescriptor *task);



    double EstimateTravelTime(const DataState::StateLocalPosition &dst, const VehicleState &startState, VehicleState &endState);

    bool ConvertTaskPositionToGlobalPosition(const DataState::StateGlobalPosition &globalOrigin, state_space::State *state, DataState::StateGlobalPosition &position);
};

#endif // TASK_DECOMPOSER_H
