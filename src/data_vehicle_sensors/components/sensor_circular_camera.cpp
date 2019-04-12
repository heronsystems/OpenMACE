#include "sensor_circular_camera.h"
#include <cmath>

namespace DataVehicleSensors
{

const char Circular_Camera_name[] = "sensor_circular_camera";
const MaceCore::TopicComponentStructure Circular_Camera_structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<double>("viewHalfAngle");
    structure.AddTerminal<double>("alphaAttenuation");
    structure.AddTerminal<double>("betaAttenuation");
    structure.AddTerminal<double>("certainRangePercent");
    structure.AddTerminal<double>("p_d");
    structure.AddTerminal<double>("p_fa");

    return structure;
}();

MaceCore::TopicDatagram SensorCircularCamera::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;

    datagram.AddTerminal<double>("viewHalfAngle", viewHalfAngle);
    datagram.AddTerminal<double>("alphaAttenuation", alphaAttenuation);
    datagram.AddTerminal<double>("betaAttenuation", betaAttenuation);
    datagram.AddTerminal<double>("certainRangePercent", certainRangePercent);
    datagram.AddTerminal<double>("p_d", p_d);
    datagram.AddTerminal<double>("p_fa", p_fa);

    return datagram;
}

void SensorCircularCamera::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    viewHalfAngle = datagram.GetTerminal<double>("viewHalfAngle");
    alphaAttenuation = datagram.GetTerminal<double>("alphaAttenuation");
    betaAttenuation = datagram.GetTerminal<double>("betaAttenuation");
    certainRangePercent = datagram.GetTerminal<double>("certainRangePercent");
    p_d = datagram.GetTerminal<double>("p_d");
    p_fa = datagram.GetTerminal<double>("p_fa");
}

//!
//! \brief attenuatedDiskConfidence Calculate the sigma value ("confidence") to augment our probability of detection/false alarm updates to the log-odds probabilty.
//! \param distanceToSensorOrigin Distance to sensor origin
//! \param radius Radius of sensor footprint
//! \return Sigma ("confidence") value
//!
double SensorCircularCamera::attenuatedDiskConfidence(const double &distanceToSensorOrigin, const double &radius) {
    // TODO: Calculate attenuated disk confidence based on alpha/beta values:
    double confidence = 0.0;
    double certainRange = radius * certainRangePercent;
    if(distanceToSensorOrigin <= certainRange) {
        confidence = 1.0;
    }
    else if((distanceToSensorOrigin > certainRange) && (distanceToSensorOrigin <= radius)) {
        confidence = std::exp(-this->alphaAttenuation * std::pow((distanceToSensorOrigin - certainRange), this->betaAttenuation));
    }

    return confidence;
}


} //end of namespace DataVehicleSensors
