#ifndef TASK_SURVERY_DESCRIPTOR_H
#define TASK_SURVERY_DESCRIPTOR_H

#include "task_descriptor.h"
#include "abstract_task_survey_descriptor.h"

#include "base/geometry/base_polygon.h"

#include "base/geometry/polygon_2DC.h"
#include "base/geometry/polygon_2DG.h"


/*!
 * \brief The TaskSurveyDescriptorBase class contains functionality common to all specializations of TaskSurveyDescriptor
 * \tparam PolygonType Underyling type of polygon representing the survey boundary
 */
template <typename PolygonType>
class TaskSurveyDescriptorBase : public AbstractTaskSurveyDescriptor
{
public:
    /*!
     * \brief Constructor
     * \param coordinateType Coordinate type
     */
    TaskSurveyDescriptorBase(TaskDescriptor::CoordinateType coordinateType = TaskDescriptor::CoordinateType::UNKNOWN_COORD) :
        AbstractTaskSurveyDescriptor(coordinateType)
    {
    }

    /*!
     * \brief TaskSurveyDescriptorBase
     * \param creatorID Task creator ID
     * \param taskID Task ID (local)
     * \param coordinateType Coordinate type
     */
    TaskSurveyDescriptorBase(uint64_t creatorID, uint8_t taskID, TaskDescriptor::CoordinateType coordinateType = TaskDescriptor::CoordinateType::UNKNOWN_COORD) :
        AbstractTaskSurveyDescriptor(creatorID, taskID, coordinateType)
    {
    }

    virtual int getNumVertices() const
    {
        return m_surveyBoundary.polygonSize();
    }

    /*!
     * \return Survey boundary
     */
    virtual const PolygonType &getSurveyBoundary() const
    {
        return m_surveyBoundary;
    }

    /*!
     * \param surveyBoundary Survey boundary
     */
    virtual void setSurveyBoundary(const PolygonType &surveyBoundary)
    {
        m_surveyBoundary = surveyBoundary;
    }

protected:
    PolygonType m_surveyBoundary;
};

/*!
 * \brief The TaskSurveyDescriptor class describes a survey task
 * \tparam PolygonType Underyling type of polygon representing the survey boundary
 */
template <typename PolygonType>
class TaskSurveyDescriptor : public TaskSurveyDescriptorBase<PolygonType>
{
public:
    TaskSurveyDescriptor(TaskDescriptor::CoordinateType coordinateType = TaskDescriptor::CoordinateType::UNKNOWN_COORD) :
        TaskSurveyDescriptorBase<PolygonType>(coordinateType)
    {
    }

    TaskSurveyDescriptor(uint64_t creatorID, uint8_t taskID,
                         TaskDescriptor::CoordinateType coordinateType = TaskDescriptor::CoordinateType::UNKNOWN_COORD) :
        TaskSurveyDescriptorBase<PolygonType>(creatorID, taskID, coordinateType)
    {
    }

};

/*!
 * \brief 2D Cartesian specialization of TaskSurveyDescriptor
 */
template <>
class TaskSurveyDescriptor<mace::geometry::Polygon_2DC> : public TaskSurveyDescriptorBase<mace::geometry::Polygon_2DC>
{
public:
    TaskSurveyDescriptor<mace::geometry::Polygon_2DC>() :
        TaskSurveyDescriptorBase<mace::geometry::Polygon_2DC>(CoordinateType::CARTESIAN_2D)
    {
    }

    TaskSurveyDescriptor<mace::geometry::Polygon_2DC>(uint64_t creatorID, uint8_t taskID) :
        TaskSurveyDescriptorBase<mace::geometry::Polygon_2DC>(creatorID, taskID, CoordinateType::CARTESIAN_2D)
    {
    }

    std::vector<std::pair<double, double>> getVertices() const
    {
        std::vector<std::pair<double, double>> vertices;
        for (const auto &vertex : m_surveyBoundary.getVector())
            vertices.push_back(std::pair<double, double>(vertex.getXPosition(), vertex.getYPosition()));

        return vertices;
    }
};

/*!
 * \brief 2D Geodetic specialization of TaskSurveyDescriptor
 */
template <>
class TaskSurveyDescriptor<mace::geometry::Polygon_2DG> : public TaskSurveyDescriptorBase<mace::geometry::Polygon_2DG>
{
public:
    TaskSurveyDescriptor<mace::geometry::Polygon_2DG>() :
        TaskSurveyDescriptorBase<mace::geometry::Polygon_2DG>(CoordinateType::GEODETIC_2D)
    {
    }

    TaskSurveyDescriptor<mace::geometry::Polygon_2DG>(uint64_t creatorID, uint8_t taskID) :
        TaskSurveyDescriptorBase<mace::geometry::Polygon_2DG>(creatorID, taskID, CoordinateType::GEODETIC_2D)
    {
    }

    std::vector<std::pair<double, double>> getVertices() const
    {
        std::vector<std::pair<double, double>> vertices;
        for (const auto &vertex : m_surveyBoundary.getVector())
            vertices.push_back(std::pair<double, double>(vertex.getLatitude(), vertex.getLongitude()));

        return vertices;
    }
};

#endif // TASK_SURVERY_DESCRIPTOR_H

