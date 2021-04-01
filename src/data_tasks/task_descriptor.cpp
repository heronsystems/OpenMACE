#include "task_descriptor.h"

#include "task_key.h"

#include <ostream>

/*!
 * \brief Layout of values from the Print() function
 * \param out Output stream
 * \param separator Value separator
 */
void TaskDescriptor::PrintLayout(std::ostream &out, bool printDescription, const std::string &separator)
{
    if (printDescription)
        out << "TaskGenericDescriptor" << separator;
    out << "creatorID" << separator
        << "taskID" << separator
        << "type" << separator
        << "timeCreated" << separator
        << "timePenalty" << separator
        << "requiredStart" << separator
        << "requiredEnd";
}

/*!
 * \brief Converts task type enum value to a string
 * \param type Task type
 * \return String representation
 */
std::string TaskDescriptor::TaskTypeToString(TaskDescriptor::TaskType type)
{
    switch (type)
    {
        case TaskType::LOITER:
            return "Loiter";
        case TaskType::SURVEY:
            return "Survey";
        case TaskType::UNKNOWN_TYPE:
        default:
            return "Unknown";
    }
}


/*!
 * \brief Converts coordinate type enum value to a string
 * \param type Coordinate type
 * \return String representation
 */
std::string TaskDescriptor::CoordinateTypeToString(TaskDescriptor::CoordinateType type)
{
    switch (type)
    {
        case CoordinateType::CARTESIAN_2D:
            return "Cartesian_2D";
        case CoordinateType::CARTESIAN_3D:
            return "Cartesian_3D";
        case CoordinateType::GEODETIC_2D:
            return "Geodetic_2D";
        case CoordinateType::GEODETIC_3D:
            return "Geodetic_3D";
        case CoordinateType::UNKNOWN_COORD:
        default:
            return "Unknown";
    }
}

/*!
 * \brief Default constructor
 */
TaskDescriptor::TaskDescriptor()
{

}

/*!
 * \brief Constructor
 * \param creatorID Creator ID
 * \param taskID Task ID (local)
 * \param type Task type
 */
TaskDescriptor::TaskDescriptor(uint64_t creatorID, uint8_t taskID, TaskType type) :
    m_creatorID(creatorID),
    m_taskID(taskID),
    m_type(type)
{

}

/*!
 * \brief Prints to stream
 * \param out Output stream
 * \param displayValueNames Whether variable names are printed
 * \param valueSeparator Value separator
 * \param namesSeparator Separator between variable name and value
 */
void TaskDescriptor::Print(std::ostream &out,
                           bool displayValueNames,
                           const std::string &valuesSeparator,
                           const std::string &namesSeparator,
                           const std::string &subtypeStartMarker,
                           const std::string &subtypeEndMarker) const
{
    switch (m_type)
    {
        case TaskType::LOITER:
            out << "TaskLoiterDescriptor" << valuesSeparator;
            break;
        case TaskType::SURVEY:
            out << "TaskSurveyDescriptor" << valuesSeparator;
            break;
        case TaskType::UNKNOWN_TYPE:
        default:
            out <<"TaskGenericDescriptor" << valuesSeparator;
    }

    if (displayValueNames)
        out << "creatorID" << namesSeparator;
    out << m_creatorID << valuesSeparator;

    if (displayValueNames)
        out << "taskID" << namesSeparator;
    out << (int) m_taskID << valuesSeparator;

    if (displayValueNames)
        out << "timeCreated" << namesSeparator;
    out << m_timeCreated.ToString().toStdString() << valuesSeparator;

    if (displayValueNames)
        out << "timePenalty" << namesSeparator;
    out << m_timePenalty << valuesSeparator;

    if (displayValueNames)
        out << "requiredStart" << namesSeparator;
    out << m_requiredStart.ToString().toStdString()  << valuesSeparator;

    if (displayValueNames)
        out << "requiredEnd" << namesSeparator;
    out << m_requiredEnd.ToString().toStdString();
}

/*!
 * \brief Returns the task key identifying this task
 * \return Task key
 */
TaskKey TaskDescriptor::getTaskKey() const
{
    TaskKey key = { m_creatorID, m_taskID, m_timeCreated, m_type };
    return key;
}

/*!
 * \brief Gets time created
 * \return Time created
 */
const Data::EnvironmentTime &TaskDescriptor::getTimeCreated() const
{
    return m_timeCreated;
}

/*!
 * \param timeCreated Time created
 */
void TaskDescriptor::setTimeCreated(const Data::EnvironmentTime &timeCreated)
{
    m_timeCreated = timeCreated;
}

/*!
 * \return Time penalty
 */
double TaskDescriptor::getTimePenalty() const
{
    return m_timePenalty;
}

/*!
 * \param timePenalty Time penalty
 */
void TaskDescriptor::setTimePenalty(double timePenalty)
{
    m_timePenalty = timePenalty;
}

/*!
 * \return Required start
 */
const Data::EnvironmentTime &TaskDescriptor::getRequiredStart() const
{
    return m_requiredStart;
}

/*!
 * \param requiredStart Required start
 */
void TaskDescriptor::setRequiredStart(const Data::EnvironmentTime &requiredStart)
{
    m_requiredStart = requiredStart;
}

/*!
 * \return Required end
 */
const Data::EnvironmentTime &TaskDescriptor::getRequiredEnd() const
{
    return m_requiredEnd;
}

/*!
 * \param requiredEnd Required end
 */
void TaskDescriptor::setRequiredEnd(const Data::EnvironmentTime &requiredEnd)
{
    m_requiredEnd = requiredEnd;
}

/*!
 * \return Task timing
 * \sa TaskDescriptor::TaskTiming
 */
TaskDescriptor::TaskTiming TaskDescriptor::getTaskTiming() const
{
    return m_taskTiming;
}

/*!
 * \param taskTiming Sets task timing
 * \sa TaskDescriptor::TaskTiming
 */
void TaskDescriptor::setTaskTiming(const TaskTiming &taskTiming)
{
    m_taskTiming = taskTiming;
}

void TaskDescriptor::UpdateID(uint64_t creatorID, uint8_t taskID)
{
    m_creatorID = creatorID;
    m_taskID = taskID;
}

