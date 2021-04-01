#include "task_decomposer.h"

#include "data_generic_state_item/positional_aid.h"


#include "data_generic_command_item/do_items/command_goto.h"

#include "module_task_management.h"

TaskDecomposer::TaskDecomposer(ModuleTaskManagement *callback) :
    m_moduleCallback(callback)
{

}


double TaskDecomposer::getMinTurnRadius() const
{
    return m_minTurnRadius;
}

void TaskDecomposer::setMinTurnRadius(double minTurnRadius)
{
    m_minTurnRadius = minTurnRadius;
}

double TaskDecomposer::getNominalSpeed() const
{
    return m_nominalSpeed;
}

void TaskDecomposer::setNominalSpeed(double nominalSpeed)
{
    m_nominalSpeed = nominalSpeed;
}

double TaskDecomposer::getNominalAcceleration() const
{
    return m_nominalAcceleration;
}

void TaskDecomposer::setNominalAcceleration(double nominalAcceleration)
{
    m_nominalAcceleration = nominalAcceleration;
}

bool TaskDecomposer::getCanFly() const
{
    return m_canFly;
}

void TaskDecomposer::setCanFly(bool canFly)
{
    m_canFly = canFly;
}

double TaskDecomposer::getDefaultAltitude() const
{
    return m_defaultAltitude;
}

void TaskDecomposer::setDefaultAltitude(double defaultAltitude)
{
    m_defaultAltitude = defaultAltitude;
}

bool TaskDecomposer::getCanBeStationary() const
{
    return m_canBeStationary;
}

void TaskDecomposer::setCanBeStationary(bool canBeStationary)
{
    m_canBeStationary = canBeStationary;
}

double TaskDecomposer::getRequiredDistanceToTarget() const
{
    return m_requiredDistanceToTarget;
}

void TaskDecomposer::setRequiredDistanceToTarget(double requiredDistanceToTarget)
{
    m_requiredDistanceToTarget = std::max(requiredDistanceToTarget, 2.0);
}

double TaskDecomposer::getNominalClimbRate() const
{
    return m_nominalClimbRate;
}

void TaskDecomposer::setNominalClimbRate(double nominalClimbRate)
{
    m_nominalClimbRate = nominalClimbRate;
}

/*!
 * \brief Constructs a command to transition to the specified task
 * \param vehicleState Vehicle state
 * \param task Task descriptor
 * \param commandData Transition command data
 */
void TaskDecomposer::GetTransitionCommand(const VehicleState &vehicleState, const TaskDescriptorPtr &task, CommandData &commandData)
{
    switch (task->getTaskKey().type)
    {
        case TaskDescriptor::TaskType::LOITER: // Starts at loiter location.
            GetTransitionCommand_Loiter(vehicleState, task, commandData);
            break;
        case TaskDescriptor::TaskType::SURVEY: // Starts at closest vertex
            GetTransitionCommand_Survey(vehicleState, task, commandData);
            break;
        case TaskDescriptor::TaskType::UNKNOWN_TYPE:
        default:
            throw std::runtime_error("Unsupported task type");
    }
}

/*!
 * \brief Decomposes a task into commands
 * \param vehicleState Vehicle State
 * \param task Task descriptor
 * \return List of commands and associated data for the task
 */
std::shared_ptr<CommandList> TaskDecomposer::Decompose(const VehicleState &vehicleState, const TaskDescriptorPtr &task)
{
    if (m_taskDataCache == nullptr || m_taskDataCache->key != task->getTaskKey())
    {
        CommandData transitionCommand;
        GetTransitionCommand(vehicleState, task, transitionCommand);
    }

    switch (task->getTaskKey().type)
    {
        case TaskDescriptor::TaskType::LOITER: // Starts at loiter location.
            return Decompose_Loiter(vehicleState, task->GetTaskAs<AbstractTaskLoiterDescriptor>());
        case TaskDescriptor::TaskType::SURVEY: // Starts at closest vertex
            return Decompose_Survey(vehicleState, task->GetTaskAs<AbstractTaskSurveyDescriptor>());
        case TaskDescriptor::TaskType::UNKNOWN_TYPE:
            break;
    }
    return nullptr;
}

double TaskDecomposer::getTakeoffAltitude() const
{
    return m_takeoffAltitude;
}

void TaskDecomposer::setTakeoffAltitude(double takeoffAltitude)
{
    m_takeoffAltitude = takeoffAltitude;
}

/*!
 * \brief Constructs a command to transition to the specified loiter task
 * \param vehicleState Vehicle state
 * \param task Task descriptor
 * \param commandData Transition command data
 */
void TaskDecomposer::GetTransitionCommand_Loiter(const VehicleState &vehicleState,
                                                 const TaskDescriptorPtr &task,
                                                 CommandData &commandData)
{
    auto loiterCache = new TaskLoiterCache();
    loiterCache->key = task->getTaskKey();
    auto loiter = task->GetTaskAs<AbstractTaskLoiterDescriptor>();

    mace::state_space::State *loiterPosition;

    loiter->getLoiterState(loiterPosition);

    DataState::StateGlobalPosition globalOrigin(m_moduleCallback->getDataObject()->GetGlobalOrigin());

    loiterCache->startIs2D = ConvertTaskPositionToGlobalPosition(globalOrigin, loiterPosition, loiterCache->startPosition);
    if (loiterCache->startIs2D)
        loiterCache->startPosition.setAltitude(m_defaultAltitude);

    double requiredDistance = m_requiredDistanceToTarget;
    if (m_canBeStationary)
    {
        if (vehicleState.hasPosition() && vehicleState.hasAttitude())
        {
            DataState::StateLocalPosition startPosition;
            DataState::PositionalAid::GlobalPositionToLocal(globalOrigin, loiterCache->startPosition, startPosition);
            double bearingBetween = vehicleState.getPosition().bearingBetween(startPosition); // degrees
            double vehicleHeading = vehicleState.getAttitude().yaw;

            double deltaHeading = fmod(360.0 + bearingBetween - vehicleHeading, 360.0);
            if (deltaHeading > 180.0)
                loiterCache->direction = Data::LoiterDirection::CCW;
            else
                loiterCache->direction = Data::LoiterDirection::CW;
        }
        else
            loiterCache->direction = Data::LoiterDirection::CW;

        requiredDistance = std::max(m_minTurnRadius, m_requiredDistanceToTarget);
    }
    else
    {
        loiterCache->direction = Data::LoiterDirection::CW;
    }

    commandData.commandDone = [=](const VehicleState &state)
    {
        DataState::StateGlobalPosition vehiclePosition;
        DataState::PositionalAid::LocalPositionToGlobal(m_moduleCallback->getDataObject()->GetGlobalOrigin(),
                                                        state.getPosition(), vehiclePosition);
        return vehiclePosition.distanceBetween3D(loiterCache->startPosition) < requiredDistance;
    };

    CommandItem::CommandGoToPtr transitionCmd = std::make_shared<CommandItem::CommandGoTo>(m_moduleCallback->getParentMaceInstanceID(), m_moduleCallback->getVehicleID());
    CommandItem::SpatialWaypointPtr waypoint = std::make_shared<CommandItem::SpatialWaypoint>(m_moduleCallback->getParentMaceInstanceID(), m_moduleCallback->getVehicleID());

    Base3DPosition startPos;
    startPos.setPosition3D(loiterCache->startPosition.getLongitude(), loiterCache->startPosition.getLatitude(), loiterCache->startPosition.getAltitude());
    if (startPos.getZ() < 0.5)
        startPos.setZ(m_defaultAltitude);

    startPos.setCoordinateFrame(Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT);

    waypoint->setPosition(startPos);
    transitionCmd->setSpatialCommand(waypoint);

    commandData.command = transitionCmd;

    VehicleState endState;
    DataState::StateLocalPosition startPosition;
    DataState::PositionalAid::GlobalPositionToLocal(globalOrigin, loiterCache->startPosition, startPosition);

    loiterCache->expectedTransitionTime = EstimateTravelTime(startPosition, vehicleState, endState);

    commandData.expectedDuration = loiterCache->expectedTransitionTime;

    m_taskDataCache.reset(loiterCache);
}

/*!
 * \brief Constructs a command to transition to the specified survey task
 * \param vehicleState Vehicle state
 * \param task Task descriptor
 * \param commandData Transition command data
 */
void TaskDecomposer::GetTransitionCommand_Survey(const VehicleState &vehicleState, const TaskDescriptorPtr &task, CommandData &commandData)
{
    auto surveyCache = new TaskSurveyCache();

    surveyCache->startIndex = 0;
    surveyCache->key = task->getTaskKey();
    surveyCache->startIs2D = true;

    auto abstractSurvey = task->GetTaskAs<AbstractTaskSurveyDescriptor>();
    if (abstractSurvey->getCoordinateType() == TaskDescriptor::CoordinateType::CARTESIAN_2D)
        GetTransitionCommand_Survey<mace::geometry::Polygon_2DC>(vehicleState, abstractSurvey, commandData, surveyCache);
    else // 2D Geodetic
        GetTransitionCommand_Survey<mace::geometry::Polygon_2DG>(vehicleState, abstractSurvey, commandData, surveyCache);

    surveyCache->startPosition.setAltitude(vehicleState.getPosition().getPositionZ());

    DataState::StateGlobalPosition globalOrigin(m_moduleCallback->getDataObject()->GetGlobalOrigin());

    VehicleState endState;
    DataState::StateLocalPosition startPosition;
    DataState::PositionalAid::GlobalPositionToLocal(globalOrigin, surveyCache->startPosition, startPosition);
    surveyCache->expectedTransitionTime = EstimateTravelTime(startPosition, vehicleState, endState);

    commandData.expectedDuration = surveyCache->expectedTransitionTime;
    commandData.commandDone = [=](const VehicleState &vehicleState)
    {
        DataState::StateGlobalPosition vehiclePosition;
        DataState::PositionalAid::LocalPositionToGlobal(globalOrigin, vehicleState.getPosition(), vehiclePosition);
        return vehiclePosition.distanceBetween3D(surveyCache->startPosition) < m_requiredDistanceToTarget;
    };

    CommandItem::CommandGoToPtr transitionCmd = std::make_shared<CommandItem::CommandGoTo>(m_moduleCallback->getParentMaceInstanceID(), m_moduleCallback->getVehicleID());
    CommandItem::SpatialWaypointPtr waypoint = std::make_shared<CommandItem::SpatialWaypoint>(m_moduleCallback->getParentMaceInstanceID(), m_moduleCallback->getVehicleID());

    Base3DPosition startPos;
    startPos.setPosition3D(surveyCache->startPosition.getLongitude(), surveyCache->startPosition.getLatitude(), surveyCache->startPosition.getAltitude());
    startPos.setCoordinateFrame(Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT);
    if (startPos.getZ() < 0.5)
        startPos.setZ(m_defaultAltitude);

    waypoint->setPosition(startPos);
    transitionCmd->setSpatialCommand(waypoint);

    commandData.command = transitionCmd;

    m_taskDataCache.reset(surveyCache);
}

template <typename PolygonType>
void TaskDecomposer::GetTransitionCommand_Survey(const VehicleState &vehicleState,
                                                 const AbstractTaskSurveyDescriptor *abstractSurvey,
                                                 CommandData &CommandData,
                                                 TaskSurveyCache *surveyCache)
{
    DataState::StateGlobalPosition globalOrigin(m_moduleCallback->getDataObject()->GetGlobalOrigin());
    auto survey = abstractSurvey->GetTaskAs<TaskSurveyDescriptor<PolygonType>>();
    auto boundary = survey->getSurveyBoundary().getVector();
    DataState::StateGlobalPosition vehiclePosition, closest, next;
    DataState::PositionalAid::LocalPositionToGlobal(globalOrigin, vehicleState.getPosition(), vehiclePosition);
    closest = vehiclePosition;

    double closestDistance = std::numeric_limits<double>::max();
    double nextDistance;

    int index = 0;
    auto it = boundary.begin();
    while (it != boundary.end())
    {
        ConvertTaskPositionToGlobalPosition(globalOrigin, &(*it), next);

        nextDistance = vehiclePosition.distanceBetween2D(next);
        if (nextDistance < closestDistance)
        {
            closestDistance = nextDistance;
            closest = next;
            surveyCache->startIndex = index;
        }
        surveyCache->boundary.push_back(next);

        ++it;
        ++index;
    }
    if (index > 0)
        surveyCache->startPosition = surveyCache->boundary.at(surveyCache->startIndex);
}

/*!
 * \brief Decomposes a loiter task into commands
 * \param vehicleState Vehicle State
 * \param task Task descriptor
 * \return List of commands and associated data for the loiter task
 */
std::shared_ptr<CommandList> TaskDecomposer::Decompose_Loiter(const VehicleState &vehicleState, const AbstractTaskLoiterDescriptor *task)
{
    auto loiterCache = static_cast<TaskLoiterCache *>(m_taskDataCache.get());
    std::shared_ptr<CommandList> commandList = std::make_shared<CommandList>();
    CommandData data;


    double duration = task->getDuration();


//    auto loiterCommand = std::make_shared<CommandItem::SpatialWaypoint>(m_moduleCallback->getParentMaceInstanceID(), m_moduleCallback->getVehicleID());
    auto loiterCommand = std::make_shared<CommandItem::SpatialLoiter_Time>(m_moduleCallback->getParentMaceInstanceID(), m_moduleCallback->getVehicleID());
    auto loiterTask = task->GetTaskAs<AbstractTaskLoiterDescriptor>();

    Base3DPosition startPos;
    startPos.setPosition3D(loiterCache->startPosition.getLongitude(), loiterCache->startPosition.getLatitude(), loiterCache->startPosition.getAltitude());
    startPos.setCoordinateFrame(Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT);

    loiterCommand->setPosition(startPos);
    loiterCommand->duration = duration;
    loiterCommand->direction = loiterCache->direction;
    loiterCommand->radius = m_minTurnRadius;

    data.command = loiterCommand;
    data.expectedDuration = duration;

    auto gotoCommand = std::make_shared<CommandItem::CommandGoTo>(m_moduleCallback->getParentMaceInstanceID(), m_moduleCallback->getVehicleID());
    gotoCommand->setSpatialCommand(loiterCommand);

    data.command = gotoCommand;

    data.commandDone = [commandList, duration](const VehicleState &vehicleState)
    {
        const auto &currentTime = vehicleState.getTimestamp();
        auto expectedEnd = commandList->lastCommandIssued + (duration * 1000);
        return (expectedEnd < currentTime);
    };

    commandList->expectedDuration = duration;
    commandList->commands.push_back(data);

    // TODO make this configurable. User could choose to never abort, or a different threshold over the expected transition time
    // to abort the loiter.
    Data::EnvironmentTime requiredTransitionDone = vehicleState.getTimestamp() + (m_taskDataCache->expectedTransitionTime * 1000) * 2;
    commandList->abortLambda = [=](const VehicleState &vehicleState)
    {
        return commandList->commandIndex < 0 && requiredTransitionDone < vehicleState.getTimestamp();
    };

    return commandList;
}

/*!
 * \brief Decomposes a survey task into commands
 * \param vehicleState Vehicle State
 * \param task Task descriptor
 * \return List of commands and associated data for the survey task
 */
std::shared_ptr<CommandList> TaskDecomposer::Decompose_Survey(const VehicleState &vehicleState, const AbstractTaskSurveyDescriptor *task)
{
    // TODO Initial decomposition just follows vertices.
    auto surveyCache = static_cast<TaskSurveyCache *>(m_taskDataCache.get());
    std::shared_ptr<CommandList> commandList = std::make_shared<CommandList>();
    CommandData nextCommandData;

    int index = surveyCache->startIndex;
    index = (index + 1) % surveyCache->boundary.size(); // Already visited the first vertex

    CommandItem::CommandGoToPtr command;
    CommandItem::SpatialWaypointPtr waypoint;

    DataState::StateGlobalPosition currentPosition = surveyCache->startPosition;
    double distance;

    VehicleState currentVehicleState = vehicleState;
    VehicleState nextVehicleState;

    DataState::StateGlobalPosition globalOrigin(m_moduleCallback->getDataObject()->GetGlobalOrigin());
    commandList->expectedDuration = 0;
    while (index != surveyCache->startIndex)
    {
        nextVehicleState = VehicleState();
        const DataState::StateGlobalPosition &nextPosition = surveyCache->boundary.at(index);
        DataState::StateLocalPosition nextPositionLocal;
        DataState::PositionalAid::GlobalPositionToLocal(globalOrigin, nextPosition, nextPositionLocal);
        nextCommandData.expectedDuration = EstimateTravelTime(nextPositionLocal, currentVehicleState, nextVehicleState);


        command = std::make_shared<CommandItem::CommandGoTo>(m_moduleCallback->getParentMaceInstanceID(), m_moduleCallback->getVehicleID());
        waypoint = std::make_shared<CommandItem::SpatialWaypoint>(m_moduleCallback->getParentMaceInstanceID(), m_moduleCallback->getVehicleID());

        Base3DPosition nextCmdPos;
        nextCmdPos.setPosition3D(nextPosition.getLongitude(), nextPosition.getLatitude(), nextPosition.getAltitude());
        nextCmdPos.setCoordinateFrame(Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT);
        if (nextCmdPos.getZ() < 0.5)
            nextCmdPos.setZ(m_defaultAltitude);

        waypoint->setPosition(nextCmdPos);

        nextCommandData.commandDone = [=](const VehicleState &vehicleState)
        {
            DataState::StateGlobalPosition vehiclePosition;
            DataState::PositionalAid::LocalPositionToGlobal(globalOrigin, vehicleState.getPosition(), vehiclePosition);
            return nextPosition.distanceBetween3D(vehiclePosition) < m_requiredDistanceToTarget;
        };

        command->setSpatialCommand(waypoint);
        currentPosition = surveyCache->boundary.at(index);
        currentVehicleState = nextVehicleState;

        nextCommandData.command = command;

        commandList->expectedDuration += nextCommandData.expectedDuration;
        commandList->commands.push_back(nextCommandData);

        index = (index + 1) % surveyCache->boundary.size();
    }

    commandList->expectedTransitionTime = surveyCache->expectedTransitionTime;

    commandList->abortLambda = [=](const VehicleState &vehicleState)
    {
//        return false;
        Data::EnvironmentTime requiredEnd;
        if (commandList->commandIndex < 0)
        {
            requiredEnd = commandList->lastCommandIssued + commandList->expectedTransitionTime * 1000 * 1.5;

            if (requiredEnd < vehicleState.getTimestamp())
            {
                return true; // Took too long to transition to the new task
            }

            return false;
        }

        const auto &currentCommand = commandList->commands.at(commandList->commandIndex);
        requiredEnd = commandList->lastCommandIssued + (currentCommand.expectedDuration * 1000 * 1.5);

        if (requiredEnd < vehicleState.getTimestamp())
        {
            return true; // One of the commands took too long to complete
        }

        return false;
    };


    return commandList;
}

/*!
 * \brief Estimates travel time between start and destination
 * \param dst Destination
 * \param startState Start state
 * \param endState Updated to reflect ending state
 * \return Estimated travel time
 */
double TaskDecomposer::EstimateTravelTime(const DataState::StateLocalPosition &dst, const VehicleState &startState, VehicleState &endState)
{
    double distance = dst.distanceBetween3D(startState.getPosition());
    double bearingBetween = startState.getPosition().bearingBetween(dst); // degrees
    if (startState.hasAttitude())
    {
        double vehicleHeading = startState.getAttitude().yaw;

        double deltaHeading = fmod(360.0 + bearingBetween - vehicleHeading, 360.0);
        if (deltaHeading > 180)
            deltaHeading -= 180;

        distance += m_minTurnRadius * deltaHeading / (2 * M_PI);
    }
    // TODO Ignoring acceleration/climbing for now

    endState.setPosition(dst);
    DataState::StateAttitude attitude;
    attitude.pitch = 0;
    attitude.roll = 0;
    attitude.yaw = bearingBetween;
    endState.setAttitude(attitude);

    return distance / m_nominalSpeed;
}

/*!
 * \brief Converts a State pointer to a StateGlobalPosition
 * \param globalOrigin Origin
 * \param state State
 * \param position Converted position
 * \return Whether the position is 2D
 */
bool TaskDecomposer::ConvertTaskPositionToGlobalPosition(const DataState::StateGlobalPosition &globalOrigin, state_space::State *state, DataState::StateGlobalPosition &position)
{
    auto pos3DC = dynamic_cast<mace::pose::CartesianPosition_3D *>(state);
    auto pos2DC = dynamic_cast<mace::pose::CartesianPosition_2D *>(state);
    auto pos3DG = dynamic_cast<mace::pose::GeodeticPosition_3D *>(state);
    auto pos2DG = dynamic_cast<mace::pose::GeodeticPosition_2D *>(state);

    if (pos3DC)
    {
        DataState::StateLocalPosition local;
        local.setPosition(pos3DC->getXPosition(), pos3DC->getYPosition(), pos3DC->getZPosition());
        DataState::PositionalAid::LocalPositionToGlobal(globalOrigin, local, position);
        return false;
    }

    if (pos2DC)
    {
        DataState::StateLocalPosition local;
        local.setPosition(pos2DC->getXPosition(), pos2DC->getYPosition(), m_defaultAltitude);
        DataState::PositionalAid::LocalPositionToGlobal(globalOrigin, local, position);
        return true;
    }

    if (pos3DG)
    {
        position.setPosition(pos3DG->getLatitude(), pos3DG->getLongitude(), pos3DG->getAltitude());
        return false;
    }

    if (pos2DG)
    {
        position.setPosition(pos2DG->getLatitude(), pos2DG->getLongitude(), m_defaultAltitude);
        return true;
    }

    throw std::runtime_error("Unsupported coordinate system");
}
