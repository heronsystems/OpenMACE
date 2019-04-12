#include "sensor_camera.h"

namespace DataVehicleSensors
{

const char Camera_name[] = "sensor_camera";
const MaceCore::TopicComponentStructure Camera_structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<double>("FOV horizontal angle");
    structure.AddTerminal<double>("FOV vertical angle");
    structure.AddTerminal<double>("focal length");
    structure.AddTerminal<double>("image width");
    structure.AddTerminal<double>("image height");
    structure.AddTerminal<double>("sensor width");
    structure.AddTerminal<double>("sensor height");
    structure.AddTerminal<double>("image rate");

    return structure;
}();

MaceCore::TopicDatagram SensorCamera::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;

    datagram.AddTerminal<double>("FOV horizontal angle",HFOVA);
    datagram.AddTerminal<double>("FOV vertical angle",VFOVA);
    datagram.AddTerminal<double>("focal length",focalLength);
    datagram.AddTerminal<double>("image width",imageWidth);
    datagram.AddTerminal<double>("image height",imageHeight);
    datagram.AddTerminal<double>("sensor width",sensorWidth);
    datagram.AddTerminal<double>("sensor height",sensorHeight);
    datagram.AddTerminal<double>("image rate",imageRate);

    return datagram;
}

void SensorCamera::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    HFOVA = datagram.GetTerminal<double>("FOV horizontal angle");
    VFOVA = datagram.GetTerminal<double>("FOV vertical angle");
    focalLength = datagram.GetTerminal<double>("focal length");
    imageWidth = datagram.GetTerminal<double>("image width");
    imageHeight = datagram.GetTerminal<double>("image height");
    sensorWidth = datagram.GetTerminal<double>("sensor width");
    sensorHeight = datagram.GetTerminal<double>("sensor height");
    imageRate = datagram.GetTerminal<double>("image rate");
}

void SensorCamera::updateCameraProperties()
{
    HFOVA = atan(sensorWidth/(2*focalLength)) * 2;
    VFOVA = atan(sensorHeight/(2*focalLength)) * 2;
}


} //end of namespace DataVehicleSensors
