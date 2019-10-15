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

    MissionHomeTopic(const command_item::SpatialHome &obj)
    {
        this->setHome(obj);
    }


    void setHome(const command_item::SpatialHome &homeItem){
        item = homeItem;
    }

    command_item::SpatialHome getHome(){
        return item;
    }


private:
    command_item::SpatialHome item;
};

} //end of namespace MissionTopic

#endif // MISSION_HOME_TOPIC_H
