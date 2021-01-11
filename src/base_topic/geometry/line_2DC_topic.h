#ifndef LINE_2DC_TOPIC_H
#define LINE_2DC_TOPIC_H

#include "data/i_topic_component_data_object.h"

#include "base/geometry/base_line.h"

namespace mace{
namespace geometryTopic{

extern const char Line_2DC_Topic_name[];
extern const MaceCore::TopicComponentStructure Line_2DC_Topic_structure;

class Line_2DC_Topic :public Data::NamedTopicComponentDataObject<Line_2DC_Topic_name, &Line_2DC_Topic_structure>
{
public:
    MaceCore::TopicDatagram GenerateDatagram() const override;
    void CreateFromDatagram(const MaceCore::TopicDatagram &datagram) override;

public:
    Line_2DC_Topic() = default;

    Line_2DC_Topic(const geometry::Line_2DC &obj)
    {
        this->setLine(obj);
    }

    void setLine(const geometry::Line_2DC &obj)
    {
        this->line = obj;
    }

    geometry::Line_2DC getLine() const
    {
        return line;
    }

private:
    geometry::Line_2DC line;
};

} //end of namepsace geometry
} //end of namespace mace

#endif // LINE_2DC_TOPIC_H
