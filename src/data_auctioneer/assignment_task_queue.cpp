#include "assignment_task_queue.h"

void AssignmentTaskQueue::PrintLayout(std::ostream &out, const std::string &separator, const std::string &subtypeStartMarker, const std::string &subtypeEndMarker, const std::string &varSizeMarker)
{
    out << "AssignmentTaskQueue" << separator
        << "size" << separator
        << "queue" << subtypeStartMarker;
    TaskKey::PrintLayout(out, separator);
    out << subtypeEndMarker << separator << varSizeMarker;
}

AssignmentTaskQueue::AssignmentTaskQueue()
{

}

/*!
 * \brief Prints to stream
 * \param out Output stream
 * \param displayValueNames Whether variable names are printed
 * \param taskKeysOnNewLine Whether elements of the queue should be printed on a new line
 * \param valueSeparator Value separator
 * \param namesSeparator Separator between variable name and value
 * \param subtypeStartMarker Denotes the start of an embedded type
 * \param subtypeEndMarker Denotes the end of an embedded type
 */
void AssignmentTaskQueue::Print(std::ostream &out,
                                bool displayValueNames,
                                bool taskKeysOnNewLine,
                                const std::string &valueSeparator,
                                const std::string &namesSeparator,
                                const std::string &subtypeStartMarker,
                                const std::string &subtypeEndMarker) const
{
    int size = this->Size();

    out << "AssignmentTaskQueue" << valueSeparator;

    if (displayValueNames)
        out << "size" << namesSeparator;
    out << size << valueSeparator;

    int keyIndex = 0;
    for (const auto &keyStatePair : m_tasks)
    {
        const TaskKey &key = keyStatePair.first;
        if (taskKeysOnNewLine)
            out << std::endl;

        if (displayValueNames)
            out << "AssignedTask_" << keyIndex << namesSeparator;
        out << subtypeStartMarker;
        key.Print(out, displayValueNames, valueSeparator, namesSeparator);
        out << subtypeEndMarker;

        if (keyIndex < size - 1)
            out << valueSeparator;

        ++keyIndex;
    }
}

/*!
 * \brief Adds a task
 * \param key Task key
 */
void AssignmentTaskQueue::AddTask(const TaskKey &key, const VehicleStatePtr &finalState)
{
    m_tasks.push_back({key, finalState});
}

/*!
 * \brief Retrieves the currently assigned task
 * \param key Current task
 * \return true if a task is currently assigned, false otherwise
 */
bool AssignmentTaskQueue::getCurrentTaskKey(TaskKey &key) const
{
    if (m_tasks.empty())
        return false;

    key = m_tasks.front().first;
    return true;
}

/*!
 * \brief Retrieves the last assigned task
 * \param key last assigned task key
 * \return true if a task is currently assigned, false otherwise
 */
bool AssignmentTaskQueue::getLastTaskKey(TaskKey &key) const
{
    if (m_tasks.empty())
        return false;

    key = m_tasks.back().first;
    return true;
}

/*!
 * \brief Retrieves the state the agent will be in when completing the last assigned task
 * \param state last assigned task state
 * \return true if a task is currently assigned, false otherwise
 */
bool AssignmentTaskQueue::getFinalState(VehicleStatePtr &state) const
{
    if (m_tasks.empty())
        return false;

    state = m_tasks.back().second;
    return true;
}

/*!
 * \brief Removes the front of the assigned task queue
 */
void AssignmentTaskQueue::RemoveFront()
{
    auto it = m_tasks.begin();
    if (it != m_tasks.end())
        m_tasks.erase(it);

    if (m_firstNonConsensusIndex > 0)
        --m_firstNonConsensusIndex;
}

/*!
 * \brief Clears all assignments
 */
void AssignmentTaskQueue::Clear()
{
    m_tasks.clear();
    m_firstNonConsensusIndex = 0;
}

/*!
 * \return Whether any tasks are assigned
 */
bool AssignmentTaskQueue::Empty() const
{
    return m_tasks.empty();
}

/*!
 * \return Begin iterator position
 */
AssignmentTaskQueue::iterator AssignmentTaskQueue::Begin()
{
    return m_tasks.begin();
}

/*!
 * \overload
 */
AssignmentTaskQueue::const_iterator AssignmentTaskQueue::Begin() const
{
    return m_tasks.cbegin();
}

/*!
 * \return Iterator positioned at first assigned task without consensus reached
 */
AssignmentTaskQueue::iterator AssignmentTaskQueue::Begin_NonConsensus()
{
    if (m_tasks.empty())
        return m_tasks.end();
    return m_tasks.begin() + m_firstNonConsensusIndex;
}

/*!
 * \overload
 */
AssignmentTaskQueue::const_iterator AssignmentTaskQueue::Begin_NonConsensus() const
{
    if (m_tasks.empty())
        return m_tasks.cend();
    return m_tasks.cbegin() + m_firstNonConsensusIndex;
}

/*!
 * \return End iterator position
 */
AssignmentTaskQueue::iterator AssignmentTaskQueue::End()
{
    return m_tasks.end();
}

/*!
 * \overload
 */
AssignmentTaskQueue::const_iterator AssignmentTaskQueue::End() const
{
    return m_tasks.cend();
}

/*!
 * \brief Remove tasks after the given iterator position
 * \param removeAfterIterator Iterator to begin removal from
 * \return A vector of the tasks which were removed
 */
void AssignmentTaskQueue::RemoveTasksAfter(const AssignmentTaskQueue::const_iterator &removeAfterIterator)
{
    m_tasks.erase(removeAfterIterator, m_tasks.cend());
}

/*!
 * \brief Remove tasks assigned after the given task key
 * \overload
 * \param removeKey TaskKey to remove after
 * \return
 */
void AssignmentTaskQueue::RemoveTasksAfter(const TaskKey &removeKey)
{
    auto taskIterator = m_tasks.begin();
    while (taskIterator != m_tasks.end())
    {
        if (taskIterator->first == removeKey)
        {
            RemoveTasksAfter(taskIterator);
            break;
        }

        ++taskIterator;
    }
}

/*!
 * \return Assignment queue size
 */
size_t AssignmentTaskQueue::Size() const
{
    return m_tasks.size();
}

/*!
 * \return Index of first assigned task without consensus
 */
int AssignmentTaskQueue::getFirstNonConsensusIndex() const
{
    return m_firstNonConsensusIndex;
}

/*!
 * \param firstNonConsensusIndex Index of first assigned task without consensus
 */
void AssignmentTaskQueue::setFirstNonConsensusIndex(int firstNonConsensusIndex)
{
    m_firstNonConsensusIndex = firstNonConsensusIndex;
}

/*!
 * \return Whether consensus has been reached on all assigned tasks
 */
bool AssignmentTaskQueue::ConsensusOnAllTasks() const
{
    return (Size() <= m_firstNonConsensusIndex) || Empty();
}

/*!
 * \brief Removes a task. Subsequent tasks are not removed
 * \return Whether the task was in the queue
 */
bool AssignmentTaskQueue::RemoveTask(const TaskKey &key)
{
    int index = 0;
    auto taskIterator = m_tasks.begin();
    while (taskIterator != m_tasks.end())
    {
        if (taskIterator->first == key)
        {
            m_tasks.erase(taskIterator);
            break;
        }

        ++index;
        ++taskIterator;
    }
    if (taskIterator == m_tasks.end())
        return false;

    if (index < m_firstNonConsensusIndex)
        --m_firstNonConsensusIndex;

    return true;
}
