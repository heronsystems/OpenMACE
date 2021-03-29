#ifndef TASK_DESCRIPTOR_H
#define TASK_DESCRIPTOR_H

#include <stdint.h>
#include <string>

#include "common/class_forward.h"

#include "data/environment_time.h"

struct TaskKey;

MACE_CLASS_FORWARD(TaskDescriptor);

/*!
 * \brief The TaskDescriptor class describes a task.
 * \details This is the base class for all task descriptions, information and methods
 * common to all tasks.
 */
class TaskDescriptor
{

public:
    /*!
     * \brief The TaskType enum represents the type of task a TaskDescriptor represents
     */
    typedef enum class TaskType : uint8_t
    {
        LOITER,
        SURVEY,
        UNKNOWN_TYPE
    } TaskType;

    /*!
     * \brief The CoordinateType enum represents the underlying coordinate frame type for tasks
     * which utilize positional data
     */
    typedef enum class CoordinateType : uint8_t
    {
        CARTESIAN_2D,
        CARTESIAN_3D,
        GEODETIC_2D,
        GEODETIC_3D,
        UNKNOWN_COORD
    } CoordinateType;

    typedef enum class TaskTiming : uint8_t
    {
        TIMING_BEFORE,
        TIMING_AT,
        TIMING_AFTER,
        TIMING_ABSOLUTE,
        TIMING_PRACTICAL
    } TaskTiming; // NOTE: currently unused

    static constexpr double s_baseReward = 100.0;


    static void PrintLayout(std::ostream &out,
                            bool printDescription = true,
                            const std::string &separator = ",");

    static std::string TaskTypeToString(TaskType type);

    static std::string CoordinateTypeToString(CoordinateType type);


    TaskDescriptor();
    TaskDescriptor(uint64_t creatorID, uint8_t taskID, TaskType type);

    virtual void Print(std::ostream &out,
                       bool displayValueNames = false,
                       const std::string &valuesSeparator = ":",
                       const std::string &namesSeparator = ",",
                       const std::string &subtypeStartMarker = "{",
                       const std::string &subtypeEndMarker = "}") const;

    TaskKey getTaskKey() const;

    /*!
     * \brief Returns a pointer to this descriptor cast to another descriptor
     * \return Pointer to descriptor type, or nullptr if the cast fails
     */
    template <typename T>
    T *GetTaskAs()
    {
        return dynamic_cast<T *>(this);
    }

    /*!
     * \brief Returns a pointer to this descriptor cast to another descriptor
     * \return Pointer to descriptor type, or nullptr if the cast fails
     */
    template <typename T>
    const T *GetTaskAs() const
    {
        return dynamic_cast<const T *>(this);
    }

    /*!
     * \brief Returns a clone of this descriptor as another descriptor type
     * \return Pointer to descriptor type, or nullptr if the cast fails
     */
    template <typename T>
    T *GetTaskClone()
    {
        const T *test = GetTaskAs<T *>();
        if (test)
            return new T(*test);
        return nullptr;
    }

    const Data::EnvironmentTime &getTimeCreated() const;
    void setTimeCreated(const Data::EnvironmentTime &timeCreated);

    double getTimePenalty() const;
    void setTimePenalty(double timePenalty);

    const Data::EnvironmentTime &getRequiredStart() const;
    void setRequiredStart(const Data::EnvironmentTime &requiredStart);

    const Data::EnvironmentTime &getRequiredEnd() const;
    void setRequiredEnd(const Data::EnvironmentTime &requiredEnd);

    TaskTiming getTaskTiming() const;
    void setTaskTiming(const TaskTiming &taskTiming);

    void UpdateID(uint64_t creatorID, uint8_t taskID);

protected:
    uint64_t  m_creatorID = 0;
    // Leaving as a uint64_t for now, since this provides a construction of the ID from
    // MaceCore::ModuleCharacteristic. To change to a uint8_t, there needs to be a mapping
    // between the ID to MaceCore::ModuleCharacteristic so that agents know how to address
    // messages based on the ID.

    uint8_t  m_taskID = 0;
    TaskType  m_type;
    Data::EnvironmentTime m_timeCreated;
    double  m_timePenalty;
    Data::EnvironmentTime m_requiredStart;
    Data::EnvironmentTime m_requiredEnd;

    TaskTiming m_taskTiming;
};

// Generic stream output
inline std::ostream &operator<<(std::ostream &out, const TaskDescriptor& descriptor)
{
    descriptor.Print(out);
    return out;
}

namespace std
{
template<>
struct hash<TaskDescriptor::TaskType>
{
    size_t operator() (const TaskDescriptor::TaskType &type) const
    {
        return hash<int>{}(static_cast<int>(type));
    }
};
}

#endif // TASK_DESCRIPTOR_H
