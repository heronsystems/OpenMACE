#include "data_generic_item_relative_pose.h"

namespace DataGenericItem{

DataGenericItem_RelativePose::DataGenericItem_RelativePose() :
     isReference(false), distance(0), bearing(0), altitude(0), heading(0)
{
}

DataGenericItem_RelativePose::DataGenericItem_RelativePose(const DataGenericItem_RelativePose &copyObj){
    this->distance = copyObj.getDistance();
    this->bearing = copyObj.getBearing();
    this->altitude = copyObj.getAltitude();
    this->heading = copyObj.getHeading();
    this->isReference = copyObj.getReference();
}

DataGenericItem_RelativePose::DataGenericItem_RelativePose(bool reference, const QJsonObject &position) :
    isReference(reference),
    distance(position["r"].toDouble()),
    bearing(position["bearing"].toDouble()),
    altitude(position["alt"].toDouble()),
    heading(position["hng"].toDouble())
{
}









}
