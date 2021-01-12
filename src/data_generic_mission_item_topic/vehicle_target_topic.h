#ifndef VEHICLE_TARGET_TOPIC_H
#define VEHICLE_TARGET_TOPIC_H

#include <iostream>
#include <iomanip>
#include <sstream>

#include <mavlink.h>

#include "data/i_topic_component_data_object.h"
#include "base/pose/pose_components.h"
#include "data/controller_state.h"
#include "data/jsonconverter.h"

namespace MissionTopic{

extern const char VehicleTargetTopic_name[];
extern const MaceCore::TopicComponentStructure VehicleTargetTopic_structure;

class VehicleTargetTopic: public Data::NamedTopicComponentDataObject<VehicleTargetTopic_name, &VehicleTargetTopic_structure>, public JSONConverter
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;
    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);
public:    
    VehicleTargetTopic();
    VehicleTargetTopic(const unsigned int &vehicleID, const mace::pose::Position* targetPosition, const double &distanceToTarget);
    VehicleTargetTopic(const VehicleTargetTopic &copy);
    VehicleTargetTopic(const mavlink_guided_target_stats_t &obj);

    ~VehicleTargetTopic()
    {
        if(m_targetPosition) {
            delete m_targetPosition;
            m_targetPosition = nullptr;
        }
    }

public:
    unsigned int getVehicleID() const{
        return systemID;
    }

    void setVehicleID(const unsigned int &ID){
        this->systemID = ID;
    }

    mavlink_guided_target_stats_t getMACECommsObject() const;
    mavlink_message_t getMACEMsg(const uint8_t systemID, const uint8_t compID, const uint8_t chan) const;

    virtual QJsonObject toJSON(const int &vehicleID, const std::string &dataType) const;

    virtual void fromJSON(const QJsonDocument &inputJSON) ;

    virtual std::string toCSV(const std::string &delimiter) const;

public:
    void operator = (const VehicleTargetTopic &rhs)
    {
        this->systemID = rhs.systemID;
        this->m_targetPosition = rhs.m_targetPosition;
        this->m_distanceToTarget = rhs.m_distanceToTarget;
    }

    bool operator == (const VehicleTargetTopic &rhs) {

        if(this->systemID != rhs.systemID)
        {
            return false;
        }

        if(this->m_targetPosition != rhs.m_targetPosition)
        {
            return false;
        }

        if(this->m_distanceToTarget != rhs.m_distanceToTarget)
        {
            return false;
        }
        return true;
    }

    bool operator != (const VehicleTargetTopic &rhs) {
        return !(*this == rhs);
    }

    friend std::ostream& operator<<(std::ostream& os, const VehicleTargetTopic& t);

private:
    unsigned int systemID;

public:
    mace::pose::Position* m_targetPosition;
    double m_distanceToTarget;
};
} //end of namespace MissionTopic
#endif // VEHICLE_TARGET_TOPIC_H
