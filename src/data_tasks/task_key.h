#ifndef TASK_KEY_H
#define TASK_KEY_H

#include <stdint.h>
#include <algorithm>
#include <ostream>

#include "task_descriptor.h"

#include "data/environment_time.h"

/*!
 * \brief The TaskKey struct defines a unique key to identify tasks.
 * \details The creatorID and taskID uniquely identify a task within the MACE network,
 * and the key is sufficient for an agent to determine if it can address the task.
 */
typedef struct TaskKey
{
    uint64_t creatorID;              /*!< Unique creator ID */
    uint8_t taskID;                  /*!< Locally unique ID assigned by the task creator */
    Data::EnvironmentTime timestamp; /*!< Timestamp */
    TaskDescriptor::TaskType   type; /*!< Task type */

    /*!
     * \brief Layout of values from the Print() function
     * \param out Output stream
     * \param separator Value separator
     */
    static void PrintLayout(std::ostream &out,
                     const std::string &separator = ",")
    {
        out << "TaskKey" << separator
            << "creatorID" << separator
            << "taskID" << separator
            << "timestamp" << separator
            << "type";
    }

    /*!
     * \brief Prints to stream
     * \param out Output stream
     * \param displayValueNames Whether variable names are printed
     * \param valueSeparator Value separator
     * \param namesSeparator Separator between variable name and value
     * \param description Object description
     */
    void Print(std::ostream &out,
               bool displayValueNames = false,
               const std::string &valueSeparator = ",",
               const std::string &namesSeparator = ":") const
    {
        out << "TaskKey" << valueSeparator;

        if (displayValueNames)
            out << "creatorID" << namesSeparator;
        out << creatorID << valueSeparator;

        if (displayValueNames)
            out << "taskID" << namesSeparator;
        out << (int) taskID << valueSeparator;

        if (displayValueNames)
            out << "timestamp" << namesSeparator;
        out << timestamp.ToString().toStdString() << valueSeparator;

        if (displayValueNames)
            out << "type" << namesSeparator;
        out << (TaskDescriptor::TaskTypeToString(type));
    }
} TaskKey;

inline bool operator==(const TaskKey &key1, const TaskKey &key2)
{
    return key1.creatorID == key2.creatorID && key1.taskID == key2.taskID;
}

inline bool operator!=(const TaskKey &key1, const TaskKey &key2)
{
    return !(key1 == key2);
}

// Generic stream output
inline std::ostream &operator<<(std::ostream &out, const TaskKey& key)
{
    key.Print(out);
    return out;
}

// std::hash specialization for TaskKey
namespace std
{
template <>
struct hash<TaskKey>
{
    size_t operator() (const TaskKey &key) const
    {
        return hash<uint64_t>{}(key.creatorID) ^ hash<uint8_t>{}(key.taskID);
    }
};
}

#endif // TASK_KEY_H
