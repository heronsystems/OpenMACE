#include "abstract_task_loiter_descriptor.h"

#include <ostream>

void AbstractTaskLoiterDescriptor::PrintLayout(std::ostream &out, bool printDescription, const std::string &separator)
{
    if (printDescription)
        out << "TaskLoiterDescriptor" << separator;
    TaskDescriptor::PrintLayout(out, false, separator);

    out << separator
        << "duration" << separator
        << "coordinateType" << separator
        << "loiterPosition.x" << separator
        << "loiterPosition.y" << separator
        << "loiterPosition.z";
}

AbstractTaskLoiterDescriptor::AbstractTaskLoiterDescriptor(CoordinateType coordinateType) :
    TaskDescriptor(),
    m_coordinateType(coordinateType)
{

}

AbstractTaskLoiterDescriptor::AbstractTaskLoiterDescriptor(uint64_t creatorID,
                                                           uint8_t taskID,
                                                           CoordinateType coordinateType) :
    TaskDescriptor(creatorID, taskID, TaskType::LOITER),
    m_coordinateType(coordinateType)
{

}


void AbstractTaskLoiterDescriptor::Print(std::ostream &out,
                                      bool displayValueNames,
                                      const std::string &valueSeparator,
                                      const std::string &namesSeparator,
                                      const std::string &subtypeStartMarker,
                                      const std::string &subtypeEndMarker) const
{
    TaskDescriptor::Print(out, displayValueNames, valueSeparator, namesSeparator, subtypeStartMarker, subtypeEndMarker);
    out << valueSeparator;
    if (displayValueNames)
        out << "duration" << namesSeparator;
    out << m_duration << valueSeparator;

    if (displayValueNames)
        out << "coordinateType" << namesSeparator;
    out << CoordinateTypeToString(getCoordinateType()) << valueSeparator;

    if (displayValueNames)
        out << "loiterPosition.x" << namesSeparator;
    out << getLoiterX() << valueSeparator;

    if (displayValueNames)
        out << "loiterPosition.y" << namesSeparator;
    out << getLoiterY() << valueSeparator;

    if (displayValueNames)
        out << "loiterPosition.z" << namesSeparator;
    out << getLoiterZ();
}


/*!
 * \return Loiter duration
 */
uint32_t AbstractTaskLoiterDescriptor::getDuration() const
{
    return m_duration;
}

/*!
 * \param duration Loiter duration
 */
void AbstractTaskLoiterDescriptor::setDuration(const uint32_t &duration)
{
    m_duration = duration;
}

/*!
 * \brief Returns the type of coordinate system being used.
 * \sa TaskDescriptor::CoordinateType
 * \return coordinate system type
 */
TaskDescriptor::CoordinateType AbstractTaskLoiterDescriptor::getCoordinateType() const
{
    return m_coordinateType;
}

/*!
 * \brief Retrieves x coordinate of underlying coordinate system
 * \return x coordinate
 */
double AbstractTaskLoiterDescriptor::getLoiterX() const
{
    return 0.0;
}

/*!
 * \brief Retrieves y coordinate of underlying coordinate system
 * \return y coordinate
 */
double AbstractTaskLoiterDescriptor::getLoiterY() const
{
    return 0.0;
}

/*!
 * \brief Retrieves z coordinate of underlying coordinate system
 * \return z coordinate
 */
double AbstractTaskLoiterDescriptor::getLoiterZ() const
{
    return 0.0;
}


