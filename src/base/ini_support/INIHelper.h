
#ifndef INIHELPER_H
#define INIHELPER_H

#include <stdint.h>
#include <string>
#include <unordered_set>
#include <unordered_map>
#include <vector>

#include "common/string_methods.h"
#include "INIReader.h"

#include "base/geometry/polygon_2DG.h"
#include "base/pose/geodetic_position_2D.h"

class INIHelper
{
public:
   static bool generateGeodetic2DLocation(INIReader *iniReader, const std::string &sectionTag, const std::string &sectionField_Location, mace::pose::GeodeticPosition_2D &obj)
   {
       std::string value_Location = "";
       value_Location = iniReader->Get(sectionTag, sectionField_Location, value_Location);

       mace::pose::GeodeticPosition_2D origin;
       std::vector<std::string> geodeticLocation = StringMethods::SplitString(value_Location, ',');
       if(geodeticLocation.size() < 2)
            return false;

       obj.setLatitude(std::stod(geodeticLocation.at(0)));
       obj.setLongitude(std::stod(geodeticLocation.at(1)));
       return true;
   }

   static bool generatePolygon(INIReader *iniReader, const std::string &sectionTag, mace::geometry::Polygon_2DG &obj)
   {
       std::string sectionField_LatOperationalBoundaries = "latitude", value_LatOperationalBoundaries = "";
       value_LatOperationalBoundaries = iniReader->Get(sectionTag, sectionField_LatOperationalBoundaries, value_LatOperationalBoundaries);
       std::string sectionField_LngOperationalBoundaries = "longitude", value_LngOperationalBoundaries = "";
       value_LngOperationalBoundaries = iniReader->Get(sectionTag, sectionField_LngOperationalBoundaries, value_LngOperationalBoundaries);

       //check that the lengths of each vector match
       std::vector<std::string> polygonLat = StringMethods::SplitString(value_LatOperationalBoundaries, ',');
       std::vector<std::string> polygonLng = StringMethods::SplitString(value_LngOperationalBoundaries, ',');
       if (polygonLat.size() != polygonLng.size())
       {
           std::cout << "Boundary sizes are not of the same length." << std::endl;
       }

       obj.clearPolygon();

       for (size_t index = 0; index < polygonLat.size(); index++)
       {
           mace::pose::GeodeticPosition_2D vertex(std::stod(polygonLat.at(index)), std::stod(polygonLng.at(index)));
           obj.appendVertex(vertex);
       }

       return true;
   }

   
};

#endif //INIHELPER_H
