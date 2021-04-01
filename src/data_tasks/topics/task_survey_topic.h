#ifndef TASK_SURVEY_TOPIC_H
#define TASK_SURVEY_TOPIC_H

#include "data/topic_data_object_collection.h"
#include "mace_core/topic.h"

#include "../task_survey_descriptor.h"
#include "mace_core/module_characteristics.h"

extern const char g_taskSurveyName[];
extern const MaceCore::TopicComponentStructure g_taskSurveyStructure;

/*!
 * \brief The TaskSurveyTopic class can be used to send a survey descriptor.
 */
class TaskSurveyTopic : public Data::NamedTopicComponentDataObject<g_taskSurveyName, &g_taskSurveyStructure>
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;
    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);

    const std::shared_ptr<AbstractTaskSurveyDescriptor> &getDescriptor() const;

    void setDescriptor(const std::shared_ptr<AbstractTaskSurveyDescriptor> &descriptor);

    bool getResponding() const;
    void setResponding(bool responding);

private:
    std::shared_ptr<AbstractTaskSurveyDescriptor> m_descriptor;
    bool m_responding;
};

#endif // TASK_SURVEY_TOPIC_H
