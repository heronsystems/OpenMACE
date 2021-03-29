#include "task_survey_topic.h"


const char g_taskSurveyName[] = "task_survey";
const MaceCore::TopicComponentStructure g_taskSurveyStructure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<AbstractTaskSurveyDescriptor>("Survey Task");
    structure.AddTerminal<bool>("Requesting");

    return structure;
}();

/*!
 * \brief Generates a datagram from underlying data
 * \return Generated datagram
 */
MaceCore::TopicDatagram TaskSurveyTopic::GenerateDatagram() const
{
    MaceCore::TopicDatagram datagram;

    datagram.AddTerminal<std::shared_ptr<AbstractTaskSurveyDescriptor>>("Survey Task", m_descriptor);
    datagram.AddTerminal<bool>("Requesting", m_responding);

    return datagram;
}

/*!
 * \brief Creates underyling data from a datagram
 * \param datagram Datagram to create from
 */
void TaskSurveyTopic::CreateFromDatagram(const MaceCore::TopicDatagram &datagram)
{
    m_descriptor = datagram.GetTerminal<std::shared_ptr<AbstractTaskSurveyDescriptor>>("Survey Task");
    m_responding = datagram.GetTerminal<bool>("Requesting");
}

/*!
 * \brief Returns the descriptor
 * \return Task descriptor
 */
const std::shared_ptr<AbstractTaskSurveyDescriptor> &TaskSurveyTopic::getDescriptor() const
{
    return m_descriptor;
}

/*!
 * \brief Sets the descriptor
 * \param descriptor Task descriptor
 */
void TaskSurveyTopic::setDescriptor(const std::shared_ptr<AbstractTaskSurveyDescriptor> &descriptor)
{
    m_descriptor = descriptor;
}

/*!
 * \brief Whether the topic is a response
 * \return Whether the topic is a response
 */
bool TaskSurveyTopic::getResponding() const
{
    return m_responding;
}

/*!
 * \brief Sets whether the topic is a response
 * \param responding Whether the topic is a response
 */
void TaskSurveyTopic::setResponding(bool responding)
{
    m_responding = responding;
}
