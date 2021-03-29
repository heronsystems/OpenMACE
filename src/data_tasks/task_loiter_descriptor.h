#ifndef TASK_LOITER_DESCRIPTOR_H
#define TASK_LOITER_DESCRIPTOR_H

#include "common/class_forward.h"

#include "task_descriptor.h"
#include "abstract_task_loiter_descriptor.h"

#include "base/pose/base_position.h"

#include "base/pose/cartesian_position_2D.h"
#include "base/pose/cartesian_position_3D.h"

#include "base/pose/geodetic_position_2D.h"
#include "base/pose/geodetic_position_3D.h"

MACE_CLASS_FORWARD(AbstractTaskLoiterDescriptor);


/*!
 * \brief The TaskLoiterDescriptorBase class contains functionality common to all specializations
 * of a TaskLoiterDescriptor
 * \tparam PositionClass Class representing the underlying coordinate frame
 */
template <typename PositionClass>
class TaskLoiterDescriptorBase : public AbstractTaskLoiterDescriptor
{
public:
    /*!
     * \brief Constructor
     * \param coordinateType Coordinate type
     */
    TaskLoiterDescriptorBase(CoordinateType coordinateType = CoordinateType::UNKNOWN_COORD) :
        AbstractTaskLoiterDescriptor(coordinateType)
    {
    }
    /*!
     * \brief Constructor
     * \param creatorID Task creator ID
     * \param taskID Task ID (local)
     * \param coordinateType Coordinate type
     */
    TaskLoiterDescriptorBase(uint64_t creatorID, uint8_t taskID, CoordinateType coordinateType = CoordinateType::UNKNOWN_COORD) :
        AbstractTaskLoiterDescriptor(creatorID, taskID, coordinateType)
    {
    }

    virtual mace::pose::CoordinateFrame getCoordinateFrame() const
    {
        return m_loiterPosition.getCoordinateFrame();
    }

    virtual void setCoordinateFrame(const mace::pose::CoordinateFrame &frame) = 0;

    /*!
     * \return Loiter position
     */
    virtual const mace::pose::Position<PositionClass> &getLoiterPosition() const
    {
        return m_loiterPosition;
    }

    /*!
     * \param loiterPosition Loiter position
     */
    virtual void setLoiterPosition(const mace::pose::Position<PositionClass> &loiterPosition)
    {
        m_loiterPosition = loiterPosition;
    }

    void getLoiterState(state_space::State *&state) const
    {
        state = m_loiterPosition.getClone();
    }

protected:
    mace::pose::Position<PositionClass> m_loiterPosition;
};

/*!
 * \brief The TaskLoiterDescriptor class describes a task loiter descriptor
 * \tparam PositionClass Class representing the underlying coordinate frame
 */
template <typename PositionClass>
class TaskLoiterDescriptor : public TaskLoiterDescriptorBase<PositionClass>
{
public:
    TaskLoiterDescriptor(TaskDescriptor::CoordinateType coordinateType = TaskDescriptor::CoordinateType::UNKNOWN_COORD) :
        TaskLoiterDescriptorBase<PositionClass>(coordinateType)
    {
    }

    TaskLoiterDescriptor(uint64_t creatorID, uint8_t taskID, TaskDescriptor::CoordinateType coordinateType = TaskDescriptor::CoordinateType::UNKNOWN_COORD) :
        TaskLoiterDescriptorBase<PositionClass>(creatorID, taskID, coordinateType)
    {
    }

};

/*!
 * \brief 2D Cartesian specialization of TaskLoiterDescriptor
 */
template <>
class TaskLoiterDescriptor<mace::pose::CartesianPosition_2D> : public TaskLoiterDescriptorBase<mace::pose::CartesianPosition_2D>
{
public:
    TaskLoiterDescriptor<mace::pose::CartesianPosition_2D>() :
        TaskLoiterDescriptorBase(CoordinateType::CARTESIAN_2D)
    {
    }

    TaskLoiterDescriptor<mace::pose::CartesianPosition_2D>(uint64_t creatorID, uint8_t taskID) :
        TaskLoiterDescriptorBase(creatorID, taskID, CoordinateType::CARTESIAN_2D)
    {
    }

    virtual double getLoiterX() const
    {
        return m_loiterPosition.getXPosition();
    }

    virtual double getLoiterY() const
    {
        return m_loiterPosition.getYPosition();
    }

    virtual void setCoordinateFrame(const mace::pose::CoordinateFrame &frame)
    {
        m_loiterPosition.setCoordinateFrame((mace::pose::LocalFrameType)
                                            ((int) frame  - (int) mace::pose::CoordinateFrame::CF_LOCAL_NED));
    }

};

/*!
 * \brief 3D Cartesian specialization of TaskLoiterDescriptor
 */
template <>
class TaskLoiterDescriptor<mace::pose::CartesianPosition_3D> : public TaskLoiterDescriptorBase<mace::pose::CartesianPosition_3D>
{
public:
    TaskLoiterDescriptor<mace::pose::CartesianPosition_3D>() :
        TaskLoiterDescriptorBase<mace::pose::CartesianPosition_3D>(CoordinateType::CARTESIAN_3D)
    {
    }

    TaskLoiterDescriptor<mace::pose::CartesianPosition_3D>(uint64_t creatorID, uint8_t taskID) :
        TaskLoiterDescriptorBase<mace::pose::CartesianPosition_3D>(creatorID, taskID, CoordinateType::CARTESIAN_3D)
    {
    }

    virtual double getLoiterX() const
    {
        return m_loiterPosition.getXPosition();
    }

    virtual double getLoiterY() const
    {
        return m_loiterPosition.getYPosition();
    }

    virtual double getLoiterZ() const
    {
        return m_loiterPosition.getZPosition();
    }


    virtual void setCoordinateFrame(const mace::pose::CoordinateFrame &frame)
    {
        m_loiterPosition.setCoordinateFrame((mace::pose::LocalFrameType)
                                            ((int) frame  - (int) mace::pose::CoordinateFrame::CF_LOCAL_NED));
    }
};



/*!
 * \brief 2D Geodetic specialization of TaskLoiterDescriptor
 */
template <>
class TaskLoiterDescriptor<mace::pose::GeodeticPosition_2D> : public TaskLoiterDescriptorBase<mace::pose::GeodeticPosition_2D>
{
public:
    TaskLoiterDescriptor<mace::pose::GeodeticPosition_2D>() :
        TaskLoiterDescriptorBase<mace::pose::GeodeticPosition_2D>(CoordinateType::GEODETIC_2D)
    {
    }

    TaskLoiterDescriptor<mace::pose::GeodeticPosition_2D>(uint64_t creatorID, uint8_t taskID) :
        TaskLoiterDescriptorBase<mace::pose::GeodeticPosition_2D>(creatorID, taskID, CoordinateType::GEODETIC_2D)
    {
    }

    virtual double getLoiterX() const
    {
        return m_loiterPosition.getLatitude();
    }

    virtual double getLoiterY() const
    {
        return m_loiterPosition.getLongitude();
    }

    virtual void setCoordinateFrame(const mace::pose::CoordinateFrame &frame)
    {
        m_loiterPosition.setCoordinateFrame((mace::pose::GlobalFrameType) frame);
    }
};

/*!
 * \brief 3D Geodetic specialization of TaskLoiterDescriptor
 */
template <>
class TaskLoiterDescriptor<mace::pose::GeodeticPosition_3D> : public TaskLoiterDescriptorBase<mace::pose::GeodeticPosition_3D>
{
public:
    TaskLoiterDescriptor<mace::pose::GeodeticPosition_3D>() :
        TaskLoiterDescriptorBase<mace::pose::GeodeticPosition_3D>(CoordinateType::GEODETIC_3D)
    {
    }

    TaskLoiterDescriptor<mace::pose::GeodeticPosition_3D>(uint64_t creatorID, uint8_t taskID) :
        TaskLoiterDescriptorBase<mace::pose::GeodeticPosition_3D>(creatorID, taskID, CoordinateType::GEODETIC_3D)
    {
    }

    virtual double getLoiterX() const
    {
        return m_loiterPosition.getLatitude();
    }

    virtual double getLoiterY() const
    {
        return m_loiterPosition.getLongitude();
    }

    virtual double getLoiterZ() const
    {
        return m_loiterPosition.getAltitude();
    }

    virtual void setCoordinateFrame(const mace::pose::CoordinateFrame &frame)
    {
        m_loiterPosition.setCoordinateFrame((mace::pose::GlobalFrameType) frame);
    }
};

#endif // TASK_LOITER_DESCRIPTOR_H
