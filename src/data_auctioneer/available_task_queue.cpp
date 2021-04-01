#include "available_task_queue.h"


/*!
 * \brief Layout of values from the Print() function
 * \param out Output stream
 * \param separator Value separator
 * \param subtypeStartMarker Denotes the start of an embedded type
 * \param subtypeEndMarker Denotes the end of an embedded type
 * \param varSizeMarker Denotes that the previous value is a variable sized list
 */
void AvailableTaskQueue::PrintLayout(std::ostream &out, const std::string &separator, const std::string &subtypeStartMarker, const std::string &subtypeEndMarker, const std::string &varSizeMarker)
{
    out << "AvailableTaskQueue" << separator
        << "size" << separator
        << "queue" << subtypeStartMarker;
    TaskKey::PrintLayout(out, separator);
    out << subtypeEndMarker << separator << varSizeMarker;
}

/*!
 * \brief Constructor
 */
AvailableTaskQueue::AvailableTaskQueue() :
    m_tasks()
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
void AvailableTaskQueue::Print(std::ostream &out,
                             bool displayValueNames,
                             bool taskKeysOnNewLine,
                             const std::string &valueSeparator,
                             const std::string &namesSeparator,
                             const std::string &subtypeStartMarker,
                             const std::string &subtypeEndMarker) const
{
    int size = this->Size();

    out << "AvailableTaskQueue" << valueSeparator;

    if (displayValueNames)
        out << "size" << namesSeparator;
    out << size << valueSeparator;

    int keyIndex = 0;
    auto taskIterator = m_tasks.begin();
    while (taskIterator != m_tasks.end())
    {
        const TaskKey &key = taskIterator->first;
        if (taskKeysOnNewLine)
            out << std::endl;

        if (displayValueNames)
            out << "AvailableTask_" << keyIndex << namesSeparator;
        out << subtypeStartMarker;
        key.Print(out, displayValueNames, valueSeparator, namesSeparator);
        out << subtypeEndMarker;

        if (keyIndex < size - 1)
            out << valueSeparator;

        ++keyIndex;
        ++taskIterator;
    }
}

/*!
 * \brief Adds a task
 * \param key Task key
 * \param priority Priority
 */
void AvailableTaskQueue::AddTask(const TaskKey &key, int priority)
{
    auto it = m_tasks.find(key);
    if (it == m_tasks.end())
        m_tasks.insert({key, priority});
    else
    {
        int &taskPriority = it->second;
        taskPriority = priority;
    }
}

/*!
 * \brief Removes a task
 * \param key Task key
 */
void AvailableTaskQueue::RemoveTask(const TaskKey &key)
{
    m_tasks.erase(key);
}

/*!
 * \brief Updates a task priority
 * \param key Task key
 * \param priority Priority
 */
void AvailableTaskQueue::UpdateTaskPriority(const TaskKey &key, int priority)
{
    auto it = m_tasks.find(key);
    if (it != m_tasks.end())
    {
        int &taskPriority = it->second;
        taskPriority = priority;
    }
}

/*!
 * \brief Constructs a vector of (task key, priority) pairs, sorts by priority, and returns the vector
 * \return Vector of available tasks sorted by priority
 */
std::vector<std::pair<TaskKey, int>> AvailableTaskQueue::SortedByPriority() const
{
    std::vector<std::pair<TaskKey, int>> sortedAvailable;

    auto it = m_tasks.begin();
    while (it != m_tasks.end())
    {
        sortedAvailable.push_back(*it);
        ++it;
    }
    std::sort(sortedAvailable.begin(), sortedAvailable.end(),
              [&](std::pair<TaskKey, int> val1,  std::pair<TaskKey, int> val2)->bool
    {
        return val1.second < val2.second;
    });

    return sortedAvailable;
}

/*!
 * \return Number of available tasks
 */
size_t AvailableTaskQueue::Size() const
{
    return m_tasks.size();
}

/*!
 * \return Begin iterator position
 */
AvailableTaskQueue::iterator AvailableTaskQueue::Begin()
{
    return m_tasks.begin();
}

/*!
 * \overload
 */
AvailableTaskQueue::const_iterator AvailableTaskQueue::Begin() const
{
    return m_tasks.cbegin();
}

/*!
 * \return Begin iterator position
 */
AvailableTaskQueue::iterator AvailableTaskQueue::End()
{
    return m_tasks.end();
}

/*!
 * \overload
 */
AvailableTaskQueue::const_iterator AvailableTaskQueue::End() const
{
    return m_tasks.cend();
}

/*!
 * \brief Erase item in bundle at iterator position
 * \param pos Erase position
 * \return incremented iterator
 */
AvailableTaskQueue::iterator AvailableTaskQueue::Erase(AvailableTaskQueue::const_iterator pos)
{
    return m_tasks.erase(pos);
}

/*!
 * \return Whether the available task queue is empty
 */
bool AvailableTaskQueue::Empty() const
{
    return m_tasks.empty();
}
