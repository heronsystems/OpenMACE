#ifndef AVAILABLE_TASK_QUEUE_H
#define AVAILABLE_TASK_QUEUE_H

#include <vector>
#include <unordered_map>
#include <queue>
#include "data_tasks/task_key.h"

/*!
 * \brief The AvailableTaskQueue class contains tasks which are available to bid on. These
 * are tasks addressable by the agent, not assigned to the agent, and otherwise without
 * consensus on the winning agent having been reached.
 */
class AvailableTaskQueue
{
public:
    typedef std::unordered_map<TaskKey, int>::iterator iterator;
    typedef std::unordered_map<TaskKey, int>::const_iterator const_iterator;

    static void PrintLayout(std::ostream &out,
                            const std::string &separator = ",",
                            const std::string &subtypeStartMarker = "{",
                            const std::string &subtypeEndMarker = "}",
                            const std::string &varSizeMarker = "...");

    AvailableTaskQueue();

    void Print(std::ostream &out,
               bool displayValueNames = true,
               bool taskKeysOnNewLine = false,
               const std::string &valueSeparator = ",",
               const std::string &namesSeparator = ":",
               const std::string &subtypeStartMarker = "{",
               const std::string &subtypeEndMarker = "}") const;

    void AddTask(const TaskKey &key, int priority = 0);

    void RemoveTask(const TaskKey &key);

    void UpdateTaskPriority(const TaskKey &key, int priority);

    // For future use
    std::vector<std::pair<TaskKey, int>> SortedByPriority() const;

    size_t Size() const;

    iterator Begin();
    const_iterator Begin() const;

    iterator End();
    const_iterator End() const;

    iterator Erase(const_iterator pos);

    bool Empty() const;

private:
    std::unordered_map<TaskKey, int> m_tasks;
};

// Generic stream output
inline std::ostream &operator<<(std::ostream &out, const AvailableTaskQueue& queue)
{
    queue.Print(out);
    return out;
}

#endif // AVAILABLE_TASK_QUEUE_H
