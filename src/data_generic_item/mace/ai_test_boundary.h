#ifndef AI_TESTBOUNDARY_H
#define AI_TESTBOUNDARY_H

#include <iostream>
#include <common/common.h>
#include <base/geometry/polygon_2DG.h>

#include "base/ini_support/INIHelper.h"
#include "base/ini_support/INIReader.h"

#include "base/pose/pose_basic_state.h"
#include "base/vehicle/vehicle_state.h"

namespace DataGenericItem {

class AI_TestBoundary
{
public:
    AI_TestBoundary();

    AI_TestBoundary(const AI_TestBoundary &copy);

    void setBoundary(const mace::geometry::Polygon_2DG &boundary);

    mace::geometry::Polygon_2DG getBoundary();

    // ** INI Loading **
    void populateBoundaryConditionsFromINI(const std::string &boundaryConditionsFilePath)
    {
        INIReader *readerTC = new INIReader(boundaryConditionsFilePath);

        if (readerTC->ParseError() < 0)
        {
            std::cout << "Can't load ini file: " + boundaryConditionsFilePath << std::endl;
            return;
        }

        // Limits fields:
        std::string section_limits = "limits",
                    sectionField_ceiling = "ceiling",
                    sectionField_floor = "floor";

        //check that indeed the ceiling is above the floor
        std::string ceiling, floor = "";
        _ceiling = std::stod(readerTC->Get(section_limits, sectionField_ceiling, ceiling));
        _floor = std::stod(readerTC->Get(section_limits, sectionField_floor, floor));


        // Boundary fields:
        std::string section_boundary = "boundary",
                    sectionField_lat = "latitude",
                    sectionField_lng = "longitude";

        // Set boundary:
        bool validRequest = INIHelper::generatePolygon(readerTC, section_boundary, _boundary);

        if(!validRequest) {
            std::cout << "!!!! Boundary not valid from INI file: " + boundaryConditionsFilePath << std::endl;
        }
    }

public:
    bool validateState(const mace::pose::GeodeticPosition_3D &position) const
    {
        //check that the start is within the boundary, this should be done before this stage to assess the validity of the start state
        double altitude = position.getAltitude();
        if((altitude > _ceiling) || (altitude < _floor))
            return false;
        return _boundary.contains(position);
    }

public:
    void operator = (const AI_TestBoundary &rhs)
    {
        _ceiling = rhs._ceiling;
        _floor = rhs._floor;
        _boundary = rhs._boundary;
    }

    bool operator == (const AI_TestBoundary &rhs) const{
        if(this->_boundary != rhs._boundary){
            return false;
        }

        return true;
    }

    bool operator != (const AI_TestBoundary &rhs) const {
        return !(*this == rhs);
    }

public:

    friend std::ostream &operator<<(std::ostream &out, const AI_TestBoundary &obj)
    {
        UNUSED(obj);
        return out;
    }

    // Boundary:
    mace::geometry::Polygon_2DG _boundary;

private:
    double _ceiling = 400;
    double _floor = 20;
};


} //end of namespace DataGenericItem

#endif // AI_TESTBOUNDARY_H
