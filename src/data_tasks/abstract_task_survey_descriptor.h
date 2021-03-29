#ifndef ABSTRACT_TASK_SURVEY_DESCRIPTOR_H
#define ABSTRACT_TASK_SURVEY_DESCRIPTOR_H

#include "task_descriptor.h"

#include "base/pose/base_position.h"

#include "base/geometry/base_polygon.h"

MACE_CLASS_FORWARD(AbstractTaskSurveyDescriptor);

class AbstractTaskSurveyDescriptor : public TaskDescriptor
{
public:
    static void PrintLayout(std::ostream &out,
                            bool printDescription = true,
                            const std::string &separator = ",",
                            const std::string &subtypeStartMarker = "{",
                            const std::string &subtypeEndMarker = "}",
                            const std::string &varSizeMarker = "...");

    AbstractTaskSurveyDescriptor(CoordinateType coordinateType = CoordinateType::UNKNOWN_COORD);
    AbstractTaskSurveyDescriptor(uint64_t creatorID, uint8_t taskID, CoordinateType coordinateType = CoordinateType::UNKNOWN_COORD);

    virtual void Print(std::ostream &out,
                       bool displayValueNames = false,
                       const std::string &valueSeparator = ",",
                       const std::string &namesSeparator = ":",
                       const std::string &subtypeStartMarker = "{",
                       const std::string &subtypeEndMarker = "}") const;

    /*!
     * \brief Returns the number of vertices in the boundary
     * \return Number of vertices
     */
    virtual int getNumVertices() const = 0;

    /*!
     * \brief Returns the vertices as a vector of a pair of doubles (x, y)
     * \return Vector of vertex pairs
     */
    virtual std::vector<std::pair<double, double>> getVertices() const = 0;

    double getSensorResolution() const;
    void setSensorResolution(double sensorResolution);

    double getOverlapHorizontal() const;
    void setOverlapHorizontal(double overlapHorizontal);

    double getOverlapVertical() const;
    void setOverlapVertical(double overlapVertical);

    CoordinateType getCoordinateType() const;



private:
    CoordinateType m_coordinateType;
    double m_sensorResolution;
    double m_overlapHorizontal;
    double m_overlapVertical;
};
#endif // ABSTRACT_TASK_SURVEY_DESCRIPTOR_H
