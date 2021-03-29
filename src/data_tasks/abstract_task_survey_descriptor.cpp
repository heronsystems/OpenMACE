#include "abstract_task_survey_descriptor.h"

#include <ostream>

void AbstractTaskSurveyDescriptor::PrintLayout(std::ostream &out, bool printDescription, const std::string &separator, const std::string &subtypeStartMarker, const std::string &subtypeEndMarker, const std::string &varSizeMarker)
{
    if (printDescription)
        out << "TaskSurveyDescriptor" << separator;
    TaskDescriptor::PrintLayout(out, false, separator);

    out << separator
        << "sensorResolution" << separator
        << "overlapHorizontal" << separator
        << "overlapVertical" << separator
        << "vertexCoordinateType" << separator
        << "numVertices" << separator
        << "vertexList" << subtypeStartMarker
        << "vertex.x" << separator
        << "vertex.y" << subtypeEndMarker
        << separator << varSizeMarker;
}

AbstractTaskSurveyDescriptor::AbstractTaskSurveyDescriptor(CoordinateType coordinateType) :
    TaskDescriptor(),
    m_coordinateType(coordinateType)
{

}

AbstractTaskSurveyDescriptor::AbstractTaskSurveyDescriptor(uint64_t creatorID, uint8_t taskID, CoordinateType coordinateType) :
    TaskDescriptor(creatorID, taskID, TaskType::SURVEY),
    m_coordinateType(coordinateType)
{

}

void AbstractTaskSurveyDescriptor::Print(std::ostream &out,
                                      bool displayValueNames,
                                      const std::string &valueSeparator,
                                      const std::string &namesSeparator,
                                      const std::string &subtypeStartMarker,
                                      const std::string &subtypeEndMarker) const
{
    TaskDescriptor::Print(out,
                          displayValueNames,
                          valueSeparator,
                          namesSeparator,
                          subtypeStartMarker,
                          subtypeEndMarker);

    out << valueSeparator;

    if (displayValueNames)
        out << "sensorResolution" << namesSeparator;
    out << m_sensorResolution << valueSeparator;

    if (displayValueNames)
        out << "overlapHorizontal" << namesSeparator;
    out << m_overlapHorizontal << valueSeparator;

    if (displayValueNames)
        out << "overlapVertical" << namesSeparator;
    out << m_overlapVertical << valueSeparator;

    if (displayValueNames)
        out << "vertexCoordinateType" << namesSeparator;
    out << CoordinateTypeToString(getCoordinateType()) << valueSeparator;

    int size = getNumVertices();
    if (displayValueNames)
        out << "numVertices" << namesSeparator;
    out << size << valueSeparator;

    if (displayValueNames)
        out << "vertexList" << namesSeparator;

    int vertexIndex = 0;
    for (const auto &vertex : getVertices())
    {
        out << subtypeStartMarker;

        if (displayValueNames)
            out << "vertex.x" << namesSeparator;
        out << vertex.first << valueSeparator;

        if (displayValueNames)
            out << "vertex.y" << namesSeparator;
        out << vertex.second;

        out << subtypeEndMarker;

        if (vertexIndex < size - 1)
            out << valueSeparator;

        ++vertexIndex;
    }
}

/*!
 * \return Sensor resolution
 */
double AbstractTaskSurveyDescriptor::getSensorResolution() const
{
    return m_sensorResolution;
}

/*!
 * \param sensorResolution Sensor resolution
 */
void AbstractTaskSurveyDescriptor::setSensorResolution(double sensorResolution)
{
    m_sensorResolution = sensorResolution;
}

/*!
 * \return Horizontal overlap
 */
double AbstractTaskSurveyDescriptor::getOverlapHorizontal() const
{
    return m_overlapHorizontal;
}

/*!
 * \param overlapHorizontal Horizontal overlap
 */
void AbstractTaskSurveyDescriptor::setOverlapHorizontal(double overlapHorizontal)
{
    m_overlapHorizontal = overlapHorizontal;
}

/*!
 * \return Vertical overlap
 */
double AbstractTaskSurveyDescriptor::getOverlapVertical() const
{
    return m_overlapVertical;
}

/*!
 * \param overlapVertical Vertical overlap
 */
void AbstractTaskSurveyDescriptor::setOverlapVertical(double overlapVertical)
{
    m_overlapVertical = overlapVertical;
}

/*!
 * \return Underlying coordinate type
 */
TaskDescriptor::CoordinateType AbstractTaskSurveyDescriptor::getCoordinateType() const
{
    return m_coordinateType;
}
