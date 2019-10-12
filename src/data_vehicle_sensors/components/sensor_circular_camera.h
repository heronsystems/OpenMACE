#ifndef SENSOR_CIRCULAR_CAMERA_H
#define SENSOR_CIRCULAR_CAMERA_H

#include <string>
#include <cmath>
#include <limits>

#include "data/i_topic_component_data_object.h"

namespace DataVehicleSensors
{

extern const char Circular_Camera_name[];
extern const MaceCore::TopicComponentStructure Circular_Camera_structure;

class SensorCircularCamera : public Data::NamedTopicComponentDataObject<Circular_Camera_name, &Circular_Camera_structure>
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;
    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);
public:

    //!
    //! \brief setCameraName Set circular camera name
    //! \param cameraName Camera name
    //!
    void setCameraName(const std::string &cameraName){
        this->cameraName = cameraName;
    }

    //!
    //! \brief getCameraName Get circular camera name
    //! \return Camera name
    //!
    std::string getCameraName() const {
        return(cameraName);
    }

    //!
    //! \brief setViewHalfAngle Set cone angle (half angle) for circular camera footprint
    //! \param halfAngle Cone half angle
    //!
    void setViewHalfAngle(const double &halfAngle) {
        this->viewHalfAngle = halfAngle;
    }

    //!
    //! \brief getViewHalfAngle Get cone angle (half angle) for circular camera footprint
    //! \return Cone half angle
    //!
    double getViewHalfAngle() const {
        return(viewHalfAngle);
    }

    //!
    //! \brief setAlphaAttenuation Set alpha value for attenuation calculation. Value should be between 0 and 1.
    //! \param alpha Alpha value for attenuation calculation
    //!
    void setAlphaAttenuation(const double &alpha) {
        this->alphaAttenuation = alpha;
    }

    //!
    //! \brief getAlphaAttenuation Get alpha value for attenuation.
    //! \return Alpha value for attenuation
    //!
    double getAlphaAttenuation() const {
        return(alphaAttenuation);
    }

    //!
    //! \brief setBetaAttenuation Set beta value for attenuation calculation. Value should be between 0 and 1.
    //! \param beta Beta value for attenuation calculation
    //!
    void setBetaAttenuation(const double &beta) {
        this->betaAttenuation = beta;
    }

    //!
    //! \brief getBetaAttenuation Get beta value for attenuation.
    //! \return Beta value for attenuation
    //!
    double getBetaAttenuation() const {
        return(betaAttenuation);
    }

    //!
    //! \brief setCertainRangePercent Set the percentage of the radial footprint of which we are confident of the measurement
    //! \param range Percent of the footprint. e.g. if the radius is 10 m, and the certain range is 0.5, attenuation will start at 5 m.
    //!
    void setCertainRangePercent(const double &range) {
        this->certainRangePercent = range;
    }

    //!
    //! \brief getCertainRangePercent Get percentage of the footprint we are confident in before attenuation starts
    //! \return Percent of the footprint we are confident in
    //!
    double getCertainRangePercent() const {
        return(certainRangePercent);
    }

    //!
    //! \brief setProbDetection Given a sensor reading, the probability of detection is a value that adds to the log-odds probability, meaning we are more confident in the reading
    //! \param probDetection Probability of detection value
    //!
    void setProbDetection(const double &probDetection) {
        this->p_d = probDetection;
    }

    //!
    //! \brief getProbDetection Get probability of detection value
    //! \return Probability of detection
    //!
    double getProbDetection() const {
        return(p_d);
    }

    //!
    //! \brief setProbFalseAlarm Given a sensor reading, the probability of false alarm is a value that subtracts from the log-odds probability, meaning we are less confident in the reading
    //! \param probFalseAlarm Probabilty of false alarm
    //!
    void setProbFalseAlarm(const double &probFalseAlarm) {
        this->p_fa = probFalseAlarm;
    }

    //!
    //! \brief getProbFalseAlarm Get probability of false alarm
    //! \return Probability of false alarm
    //!
    double getProbFalseAlarm() const {
        return(p_fa);
    }


    //!
    //! \brief attenuatedDiskConfidence Calculate the sigma value ("confidence") to augment our probability of detection/false alarm updates to the log-odds probabilty.
    //! \param distanceToSensorOrigin Distance to sensor origin
    //! \param radius Radius of sensor footprint
    //! \return Sigma ("confidence") value
    //!
    double attenuatedDiskConfidence(const double &distanceToSensorOrigin, const double &radius);


public:
    bool operator == (const SensorCircularCamera &rhs) {
        if(this->cameraName != rhs.cameraName){
            return false;
        }
        if(fabs(this->viewHalfAngle - rhs.viewHalfAngle) > std::numeric_limits<double>::epsilon()){
            return false;
        }
        if(fabs(this->alphaAttenuation - rhs.alphaAttenuation) > std::numeric_limits<double>::epsilon()){
            return false;
        }
        if(fabs(this->betaAttenuation - rhs.betaAttenuation) > std::numeric_limits<double>::epsilon()){
            return false;
        }
        if(fabs(this->certainRangePercent - rhs.certainRangePercent) > std::numeric_limits<double>::epsilon()){
            return false;
        }
        if(fabs(this->p_d - rhs.p_d) > std::numeric_limits<double>::epsilon()){
            return false;
        }
        if(fabs(this->p_fa - rhs.p_fa) > std::numeric_limits<double>::epsilon()){
            return false;
        }

        return true;
    }

    bool operator != (const SensorCircularCamera &rhs) {
        return !(*this == rhs);
    }

protected:
    //!
    //! \brief cameraName Name of the camera sensor
    //!
    std::string cameraName;

    //!
    //! \brief viewHalfAngle Cone half angle for circular aperture. Value in degrees
    //!
    double viewHalfAngle;

    //!
    //! \brief alphaAttenuation Alpha value for attenuation calculation. Value should be between 0 and 1.
    //!
    double alphaAttenuation;

    //!
    //! \brief betaAttenuation Beta value for attenuation calculation. Value should be between 0 and 1.
    //!
    double betaAttenuation;

    //!
    //! \brief certainRangePercent Percent of the radial footprint that produces 100% confidence in the reading.
    //! Between 0-1; If 0, our attenuation will start immediately. If > 0, attenuation starts certainRange % from the center
    //!
    double certainRangePercent;

    //!
    //! \brief p_d Probability of detection value that adds to the log-odds probability, meaning we are more confident in the reading. Value should between 0 and 1.
    //!
    double p_d;

    //!
    //! \brief p_fa Probability of false alarm value that subtracts from the log-odds probability, meaning we are less confident in the reading. Value should between 0 and 1.
    //!
    double p_fa;
};

}

#endif // SENSOR_CIRCULAR_CAMERA_H
