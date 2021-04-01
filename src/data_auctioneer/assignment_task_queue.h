#ifndef ASSIGNMENT_TASK_QUEUE_H
#define ASSIGNMENT_TASK_QUEUE_H

#include "data_tasks/task_key.h"

#include <vector>
#include <memory>

#include "state/vehicle_state.h"

/*!
 * \brief The AssignmentTaskQueue class represents the tasks assigned to the host agent.
 *
 * Tasks may be assigned to an agent prior to consensus being reached. If consensus is not
 * reached and another agent outbids the agent this assignment queue belongs to, the agent
 * releases the task and all tasks following it.
 */
class AssignmentTaskQueue
{
public:
    typedef std::vector<std::pair<TaskKey, VehicleStatePtr>>::iterator iterator;
    typedef std::vector<std::pair<TaskKey, VehicleStatePtr>>::const_iterator const_iterator;

    static void PrintLayout(std::ostream &out,
                            const std::string &separator = ",",
                            const std::string &subtypeStartMarker = "{",
                            const std::string &subtypeEndMarker = "}",
                            const std::string &varSizeMarker = "...");

    AssignmentTaskQueue();

    void Print(std::ostream &out,
               bool displayValueNames = false,
               bool tasksOnNewLine = false,
               const std::string &valueSeparator = ",",
               const std::string &namesSeparator = ":",
               const std::string &subtypeStartMarker = "{",
               const std::string &subtypeEndMarker = "}") const;

    void AddTask(const TaskKey &key, const VehicleStatePtr &finalState = nullptr);

    bool getCurrentTaskKey(TaskKey &key) const;

    bool getLastTaskKey(TaskKey &key) const;

    bool getFinalState(VehicleStatePtr &state) const;

    void RemoveFront();

    void Clear();

    bool Empty() const;

    iterator Begin();
    const_iterator Begin() const;

    iterator Begin_NonConsensus();
    const_iterator Begin_NonConsensus() const;

    iterator End();
    const_iterator End() const;

    void RemoveTasksAfter(const const_iterator &removeAfterIterator);
    void RemoveTasksAfter(const TaskKey &removeKey);

    size_t Size() const;

    int getFirstNonConsensusIndex() const;
    void setFirstNonConsensusIndex(int firstNonConsensusIndex);

    bool ConsensusOnAllTasks() const;

    bool RemoveTask(const TaskKey &key);

private:
    std::vector<std::pair<TaskKey, VehicleStatePtr>> m_tasks;
    int m_firstNonConsensusIndex = 0; /*!< First task in the list that consensus has not been reached for */
};

// Generic stream output
inline std::ostream &operator<<(std::ostream &out, const AssignmentTaskQueue& queue)
{
    queue.Print(out);
    return out;
}

#endif // ASSIGNMENT_TASK_QUEUE_H
