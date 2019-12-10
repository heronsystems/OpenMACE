#ifndef MISSION_LIST_TOPIC_H
#define MISSION_LIST_TOPIC_H

#include "data/i_topic_component_data_object.h"
#include "data_generic_command_item/mission_items/mission_list.h"

using namespace MissionItem;

namespace MissionTopic{

extern const char MissionListTopic_name[];
extern const MaceCore::TopicComponentStructure MissionListTopic_structure;

class MissionListTopic :public Data::NamedTopicComponentDataObject<MissionListTopic_name, &MissionListTopic_structure>
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;
    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);
public:
    MissionListTopic();
    MissionListTopic(const MissionItem::MissionList missionList);

public:
    void setMissionList(const MissionItem::MissionList missionListA);
    MissionItem::MissionList getMissionList();

    unsigned int getVehicleID() const{
        return vehicleID;
    }

    unsigned int getCreatorID() const{
        return missionList.getCreatorID();
    }

    uint64_t getMissionID() const{
        return missionList.getMissionID();
    }

    MISSIONTYPE getCommandType() const{
        return missionList.getMissionType();
    }

    MISSIONSTATE getMissionTXState() const{
        return missionList.getMissionTXState();
    }

private:
    unsigned int vehicleID;
    MissionItem::MissionList missionList;
};

} //end of namespace MissionTopic

#endif // MISSION_LIST_TOPIC_H
