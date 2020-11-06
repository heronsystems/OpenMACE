#include "vehicle_target_topic.h"

namespace MissionTopic{

const char VehicleTargetTopic_name[] = "VehicleTarget";
const MaceCore::TopicComponentStructure VehicleTargetTopic_structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<int>("systemID");

    structure.AddTerminal<std::string>("Position Name");
    structure.AddTerminal<double>("Distance to Target");
    structure.AddTerminal<CoordinateSystemTypes>("Coordinate System");
    structure.AddTerminal<mace::CoordinateFrameTypes>("Explicit Frame");
    structure.AddTerminal<mace::AltitudeReferenceTypes>("Explicit Altitude Frame");
    structure.AddTerminal<uint8_t>("Dimension");
    structure.AddTerminal<Eigen::VectorXd>("Data");
    return structure;
}();

MaceCore::TopicDatagram VehicleTargetTopic::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<int>("systemID",systemID);

    datagram.AddTerminal<std::string>("Position Name", m_targetPosition->getName());
    datagram.AddTerminal<double>("Distance to Target", m_distanceToTarget);
    datagram.AddTerminal<CoordinateSystemTypes>("Coordinate System", m_targetPosition->getCoordinateSystemType());
    datagram.AddTerminal<mace::CoordinateFrameTypes>("Explicit Frame", m_targetPosition->getExplicitCoordinateFrame());
    datagram.AddTerminal<uint8_t>("Dimension", m_targetPosition->getDimension());
    datagram.AddTerminal<Eigen::VectorXd>("Data", m_targetPosition->getDataVector());

    if(m_targetPosition->is3D())
    {
        switch (m_targetPosition->getCoordinateSystemType()) {
        case CoordinateSystemTypes::GEODETIC:
        {
            datagram.AddTerminal<mace::AltitudeReferenceTypes>("Explicit Altitude Frame", m_targetPosition->positionAs<mace::pose::GeodeticPosition_3D>()->getAltitudeReferenceFrame());
            break;
        }
        case CoordinateSystemTypes::CARTESIAN:
        {
            datagram.AddTerminal<mace::AltitudeReferenceTypes>("Explicit Altitude Frame", m_targetPosition->positionAs<mace::pose::CartesianPosition_3D>()->getAltitudeReferenceFrame());
            break;
        }
        case CoordinateSystemTypes::UNKNOWN:
        case CoordinateSystemTypes::NOT_IMPLIED:
            datagram.AddTerminal<mace::AltitudeReferenceTypes>("Explicit Altitude Frame", mace::AltitudeReferenceTypes::REF_ALT_UNKNOWN);
            break;
        }
    }
    else
    {
        datagram.AddTerminal<mace::AltitudeReferenceTypes>("Explicit Altitude Frame", mace::AltitudeReferenceTypes::REF_ALT_UNKNOWN);
    }

    return datagram;
}

void VehicleTargetTopic::CreateFromDatagram(const MaceCore::TopicDatagram &datagram)
{
    if(m_targetPosition) {
        delete this->m_targetPosition;
        m_targetPosition = nullptr;
    }

    systemID = datagram.GetTerminal<int>("systemID");
    uint8_t dimension = datagram.GetTerminal<uint8_t>("Dimension");

    std::string positionName = datagram.GetTerminal<std::string>("Position Name");
    CoordinateSystemTypes coordinateSystem = datagram.GetTerminal<CoordinateSystemTypes>("Coordinate System");
    mace::CoordinateFrameTypes coordinateFrame = datagram.GetTerminal<mace::CoordinateFrameTypes>("Explicit Frame");
    mace::AltitudeReferenceTypes altitudeFrame = datagram.GetTerminal<mace::AltitudeReferenceTypes>("Explicit Altitude Frame");
    Eigen::VectorXd data = datagram.GetTerminal<Eigen::VectorXd>("Data");

    switch (coordinateSystem) {
    case CoordinateSystemTypes::GEODETIC:
    {
        mace::GeodeticFrameTypes geoFrame = getGeodeticCoordinateFrame(coordinateFrame);
        if(dimension == 2 && data.size() == 2)
            m_targetPosition = new mace::pose::GeodeticPosition_2D(geoFrame, data(1,0), data(0,0), positionName);
        else if (dimension == 3 && data.size() == 3)
            m_targetPosition = new mace::pose::GeodeticPosition_3D(geoFrame, data(1,0), data(0,0), altitudeFrame, data(2,0), positionName);
        break;
    }
    case CoordinateSystemTypes::CARTESIAN:
    {
        mace::CartesianFrameTypes geoFrame = getCartesianCoordinateFrame(coordinateFrame);
        if(dimension == 2 && data.size() == 2)
            m_targetPosition = new mace::pose::CartesianPosition_2D(geoFrame, data(0,0), data(1,0), positionName);
        else if (dimension == 3 && data.size() == 3)
            m_targetPosition = new mace::pose::CartesianPosition_3D(geoFrame, data(0,0), data(1,0), altitudeFrame, data(2,0), positionName);
        break;
    }
    case CoordinateSystemTypes::UNKNOWN:
    case CoordinateSystemTypes::NOT_IMPLIED:
        break;
    }

    m_distanceToTarget = datagram.GetTerminal<double>("Distance to Target");
}

VehicleTargetTopic::VehicleTargetTopic() :
    systemID(0), m_targetPosition(nullptr), m_distanceToTarget(0.0)
{

}

VehicleTargetTopic::VehicleTargetTopic(const unsigned int &vehicleID, const mace::pose::Position* targetPosition, const double &distanceToTarget):
    systemID(vehicleID)
{
    if(targetPosition)
        m_targetPosition = targetPosition->getPositionalClone();

    m_distanceToTarget = distanceToTarget;
}


VehicleTargetTopic::VehicleTargetTopic(const VehicleTargetTopic &copy)
{
    this->systemID = copy.systemID;
    if(copy.m_targetPosition)
        this->m_targetPosition = copy.m_targetPosition->getPositionalClone();

    this->m_distanceToTarget = copy.m_distanceToTarget;
}

VehicleTargetTopic::VehicleTargetTopic(const mace_guided_target_stats_t &obj)
{
    UNUSED(obj);
    //Ken we need to reconstruct inside here
}

mace_guided_target_stats_t VehicleTargetTopic::getMACECommsObject() const
{
    //Ken we need to reconstruct inside here
    mace_guided_target_stats_t rtn;
    rtn.distance = this->m_distanceToTarget;
    rtn.coordinate_frame = static_cast<uint8_t>(this->m_targetPosition->getExplicitCoordinateFrame());

    switch (this->m_targetPosition->getCoordinateSystemType()) {
    case CoordinateSystemTypes::CARTESIAN:
    {
        if(m_targetPosition->is2D())
        {
            mace::pose::CartesianPosition_2D* castPosition = m_targetPosition->positionAs<mace::pose::CartesianPosition_2D>();
            rtn.x = static_cast<float>(castPosition->getXPosition());
            rtn.y = static_cast<float>(castPosition->getYPosition());
        }
        else if(m_targetPosition->is3D())
        {
            mace::pose::CartesianPosition_3D* castPosition = m_targetPosition->positionAs<mace::pose::CartesianPosition_3D>();
            rtn.x = static_cast<float>(castPosition->getXPosition());
            rtn.y = static_cast<float>(castPosition->getYPosition());
            rtn.z = static_cast<float>(castPosition->getZPosition());
        }
        break;
    }
    case CoordinateSystemTypes::GEODETIC:
    {
        if(m_targetPosition->is2D())
        {
            mace::pose::GeodeticPosition_2D* castPosition = m_targetPosition->positionAs<mace::pose::GeodeticPosition_2D>();
            rtn.x = static_cast<float>(castPosition->getLongitude());
            rtn.y = static_cast<float>(castPosition->getLatitude());
        }
        else if(m_targetPosition->is3D())
        {
            mace::pose::GeodeticPosition_3D* castPosition = m_targetPosition->positionAs<mace::pose::GeodeticPosition_3D>();
            rtn.x = static_cast<float>(castPosition->getLongitude());
            rtn.y = static_cast<float>(castPosition->getLatitude());
            rtn.z = static_cast<float>(castPosition->getAltitude());
        }
        break;
    }

    case CoordinateSystemTypes::UNKNOWN:
    case CoordinateSystemTypes::NOT_IMPLIED:
        break;
    }
    return rtn;
}
mace_message_t VehicleTargetTopic::getMACEMsg(const uint8_t systemID, const uint8_t compID, const uint8_t chan) const
{
    mace_guided_target_stats_t target = getMACECommsObject();
    mace_message_t msg;
    mace_msg_guided_target_stats_encode_chan(systemID,compID,chan,&msg,&target);
    return msg;
}

std::ostream& operator<<(std::ostream& os, const VehicleTargetTopic& t)
{
    UNUSED(t);

    std::stringstream stream;
    stream.precision(6);
    //stream << std::fixed << "Target Topic for system " << t.systemID << ": " << t.targetPosition.getX() << ", "<< t.targetPosition.getY() << ", "<< t.targetPosition.getZ() << ", "<< t.targetDistance<<", "<<Data::ControllerStateToString(t.targetState)<< ".";
    os << stream.str();

    return os;
}


} //end of namespace MissionTopic
