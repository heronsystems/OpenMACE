#ifndef SENSOR_CAMERA_H
#define SENSOR_CAMERA_H

#include <string>
#include <cmath>
#include <limits>

#include "data/i_topic_component_data_object.h"

namespace DataVehicleSensors
{

extern const char Camera_name[];
extern const MaceCore::TopicComponentStructure Camera_structure;

class SensorCamera : public Data::NamedTopicComponentDataObject<Camera_name, &Camera_structure>
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;
    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);
    void updateCameraProperties();
public:

    void setCameraName(const std::string &cameraName){
        this->cameraName = cameraName;
    }
    std::string getCameraName() const{
        return(cameraName);
    }

    void setStabilization(const bool &stabilized){
        this->stabilized = stabilized;
    }
    bool getStabilization() const{
        return(stabilized);
    }

    void setFOV_Horizontal(const double &horizontalFOV){
        this->HFOVA = horizontalFOV;
    }
    double getFOV_Horizontal() const{
        return(HFOVA);
    }

    void setFOV_Vertical(const double &verticalFOV){
        this->VFOVA = verticalFOV;
    }
    double getFOV_Vertical() const{
        return(VFOVA);
    }


    void setFocalLength(const double &focalLength){
        this->focalLength = focalLength;
        this->updateCameraProperties();
    }
    double getFocalLength() const{
        return(focalLength);
    }


    void setImageWidth(const unsigned int &imageWidth){
        this->imageWidth = imageWidth;
    }
    unsigned int getImageWidth() const{
        return(imageWidth);
    }


    void setImageHeight(const unsigned int &imageHeight){
        this->imageHeight = imageHeight;
    }
    unsigned int getImageHeight() const{
        return(imageHeight);
    }


    void setSensorWidth(const double &sensorWidth){
        this->sensorWidth = sensorWidth;
        this->updateCameraProperties();
    }
    double getSensorWidth() const{
        return(sensorWidth);
    }


    void setSensorHeight(const double &sensorHeight){
        this->sensorHeight = sensorHeight;
        this->updateCameraProperties();
    }
    double getSensorHeight() const{
        return(sensorHeight);
    }


    void setImageRate(const double &rate){
        this->imageRate = rate;
    }
    double getImageRate() const{
       return(imageRate);
    }

public:
    bool operator == (const SensorCamera &rhs) {
        if(this->cameraName != rhs.cameraName){
            return false;
        }
        if(this->stabilized != rhs.stabilized){
            return false;
        }
        if(fabs(this->HFOVA - rhs.HFOVA) > std::numeric_limits<double>::epsilon()){
            return false;
        }
        if(fabs(this->VFOVA - rhs.VFOVA) > std::numeric_limits<double>::epsilon()){
            return false;
        }
        if(fabs(this->focalLength - rhs.focalLength) > std::numeric_limits<double>::epsilon()){
            return false;
        }
        if(this->imageWidth != rhs.imageWidth){
            return false;
        }
        if(this->imageHeight != rhs.imageHeight){
            return false;
        }
        if(fabs(this->sensorWidth - rhs.sensorWidth) > std::numeric_limits<double>::epsilon()){
            return false;
        }
        if(fabs(this->sensorHeight - rhs.sensorHeight) > std::numeric_limits<double>::epsilon()){
            return false;
        }
        if(fabs(this->imageRate - rhs.imageRate) > std::numeric_limits<double>::epsilon()){
            return false;
        }
        return true;
    }

    bool operator != (const SensorCamera &rhs) {
        return !(*this == rhs);
    }

protected:
    std::string cameraName;
    bool stabilized;
    double HFOVA; //value in radians
    double VFOVA; //value in radians
    double focalLength; //value in mm
    unsigned int imageWidth; //value in pixels
    unsigned int imageHeight; //value in pixels
    double sensorWidth; //value in mm
    double sensorHeight; //value in mm
    double imageRate; //value in hz

    bool providedFOV;
};

}

#endif // SENSOR_CAMERA_H
