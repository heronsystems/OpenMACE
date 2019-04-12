#ifndef MISSION_HOME_TOPIC_H
#define MISSION_HOME_TOPIC_H

#include "data/i_topic_component_data_object.h"
#include "data_generic_command_item/spatial_items/spatial_home.h"

namespace MissionTopic{

extern const char MissionHomeTopic_name[];
extern const MaceCore::TopicComponentStructure MissionHomeTopic_structure;

class MissionHomeTopic :public Data::NamedTopicComponentDataObject<MissionHomeTopic_name, &MissionHomeTopic_structure>
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;
    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);

public:
    MissionHomeTopic()
    {

    }

    MissionHomeTopic(const CommandItem::SpatialHome &obj)
    {
        this->setHome(obj);
    }


    void setHome(const CommandItem::SpatialHome &homeItem){
        item = homeItem;
    }

    CommandItem::SpatialHome getHome(){
        return item;
    }


private:
    CommandItem::SpatialHome item;
};

} //end of namespace MissionTopic

#endif // MISSION_HOME_TOPIC_H
