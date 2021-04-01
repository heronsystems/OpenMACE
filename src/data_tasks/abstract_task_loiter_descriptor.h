#ifndef ABSTRACT_TASK_LOITER_DESCRIPTOR_H
#define ABSTRACT_TASK_LOITER_DESCRIPTOR_H

#include "common/class_forward.h"

#include "task_descriptor.h"

#include "base/pose/base_position.h"

#include "base/state_space/state.h"

using namespace mace;

MACE_CLASS_FORWARD(AbstractTaskLoiterDescriptor);

class AbstractTaskLoiterDescriptor : public TaskDescriptor
{
public:
    static void PrintLayout(std::ostream &out,
                            bool printDescription = true,
                            const std::string &separator = ",");

    AbstractTaskLoiterDescriptor(CoordinateType coordinateType = CoordinateType::UNKNOWN_COORD);
    AbstractTaskLoiterDescriptor(uint64_t creatorID,
                                 uint8_t taskID,
                                 CoordinateType coordinateType = CoordinateType::UNKNOWN_COORD);

    virtual void Print(std::ostream &out,
                       bool displayValueNames = false,
                       const std::string &valueSeparator = ",",
                       const std::string &namesSeparator = ":",
                       const std::string &subtypeStartMarker = "{",
                       const std::string &subtypeEndMarker = "}") const;

    uint32_t getDuration() const;

    void setDuration(const uint32_t &duration);

    /*!
     * \brief Retrieves the coordinate frame
     * \sa mace::pose::CoordinateFrame
     * \return Coordinate frame
     */
    virtual mace::pose::CoordinateFrame getCoordinateFrame() const = 0;

    /*!
     * \brief Sets the coordinate frame
     * \sa mace::pose::CoordinateFrame
     * \param frame Coordinate frame
     */
    virtual void setCoordinateFrame(const mace::pose::CoordinateFrame &frame) = 0;

    virtual CoordinateType getCoordinateType() const;

    virtual double getLoiterX() const;
    virtual double getLoiterY() const;
    virtual double getLoiterZ() const;

    virtual void getLoiterState(state_space::State *&state) const = 0;

protected:
    CoordinateType m_coordinateType = CoordinateType::UNKNOWN_COORD;

private:
    uint32_t m_duration;
};

#endif // ABSTRACT_TASK_LOITER_DESCRIPTOR_H
