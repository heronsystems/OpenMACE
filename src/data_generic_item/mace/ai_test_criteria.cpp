#include "ai_test_criteria.h"

namespace DataGenericItem {

AI_TestCriteria::AI_TestCriteria() :
    _elapsedEndtime(0.0)
{
}

AI_TestCriteria::AI_TestCriteria(const AI_TestCriteria &copy)
{
    _startCriteria = copy._startCriteria;
    _stopCriteria = copy._stopCriteria;
    _elapsedEndtime = copy._elapsedEndtime;
}


// ** SETTERS **
void AI_TestCriteria::criteria::set_dX(const double &dx)
{
    _dx = dx;
}

void AI_TestCriteria::criteria::set_dY(const double &dy)
{
    _dy = dy;
}

void AI_TestCriteria::criteria::set_dZ(const double &dz)
{
    _dz = dz;
}

void AI_TestCriteria::criteria::set_dRoll(const double &droll)
{
    _droll = droll;
}

void AI_TestCriteria::criteria::set_dPitch(const double &dpitch)
{
    _dpitch = dpitch;
}

void AI_TestCriteria::criteria::set_dYaw(const double &dyaw)
{
    _dyaw = dyaw;
}

void AI_TestCriteria::criteria::set_dSpeed(const double &dspeed)
{
    _dspeed = dspeed;
}


void AI_TestCriteria::setMaxTestDuration(const double &time_sec)
{
    _elapsedEndtime = time_sec;
}


// ** GETTERS **
double AI_TestCriteria::criteria::get_dX() const
{
    return _dx;
}

double AI_TestCriteria::criteria::get_dY() const
{
    return _dy;
}

double AI_TestCriteria::criteria::get_dZ() const
{
    return _dz;
}

double AI_TestCriteria::criteria::get_dRoll() const
{
    return _droll;
}

double AI_TestCriteria::criteria::get_dPitch() const
{
    return _dpitch;
}

double AI_TestCriteria::criteria::get_dYaw() const
{
    return _dyaw;
}

double AI_TestCriteria::criteria::get_dSpeed() const
{
    return _dspeed;
}

double AI_TestCriteria::getMaxTestDuration() const
{
    return _elapsedEndtime;
}

void AI_TestCriteria::populateTestConditionsFromINI(const std::string &testConditionsFilePath)
{
    INIReader *readerTC = new INIReader(testConditionsFilePath);

    if (readerTC->ParseError() < 0)
    {
        std::cout << "Can't load ini file: " + testConditionsFilePath << std::endl;
        return;
    }

    // Start criteria fields:
    std::string section_start = "start",
                sectionField_dx = "dx",
                sectionField_dy = "dy",
                sectionField_dz = "dz",
                sectionField_droll = "droll",
                sectionField_dpitch = "dpitch",
                sectionField_dyaw = "dyaw",
                sectionField_dspeed = "dspeed";

    // Abort criteria fields:
    std::string section_abort = "abort";
    // Section field names for abort are the same as start

    // End criteria fields:
    std::string section_end = "end",
                sectionField_time_sec = "time_sec";

    // Boundary fields:
    std::string section_boundary = "boundary",
                sectionField_lat = "lat",
                sectionField_lng = "lng";


    // Set start criteria:
    std::string dx_start, dy_start, dz_start, droll_start, dpitch_start, dyaw_start, dspeed_start = "";
    _startCriteria.set_dX(std::stod(readerTC->Get(section_start, sectionField_dx, dx_start)));
    _startCriteria.set_dY(std::stod(readerTC->Get(section_start, sectionField_dy, dy_start)));
    _startCriteria.set_dZ(std::stod(readerTC->Get(section_start, sectionField_dz, dz_start)));
    _startCriteria.set_dRoll(std::stod(readerTC->Get(section_start, sectionField_droll, droll_start)));
    _startCriteria.set_dPitch(std::stod(readerTC->Get(section_start, sectionField_dpitch, dpitch_start)));
    _startCriteria.set_dYaw(std::stod(readerTC->Get(section_start, sectionField_dyaw, dyaw_start)));
    _startCriteria.set_dSpeed(std::stod(readerTC->Get(section_start, sectionField_dspeed, dspeed_start)));

    // Set abort criteria:
    std::string dx_abort, dy_abort, dz_abort, droll_abort, dpitch_abort, dyaw_abort, dspeed_abort = "";
    _stopCriteria.set_dX(std::stod(readerTC->Get(section_abort, sectionField_dx, dx_abort)));
    _stopCriteria.set_dY(std::stod(readerTC->Get(section_abort, sectionField_dy, dy_abort)));
    _stopCriteria.set_dZ(std::stod(readerTC->Get(section_abort, sectionField_dz, dz_abort)));
    _stopCriteria.set_dRoll(std::stod(readerTC->Get(section_abort, sectionField_droll, droll_abort)));
    _stopCriteria.set_dPitch(std::stod(readerTC->Get(section_abort, sectionField_dpitch, dpitch_abort)));
    _stopCriteria.set_dYaw(std::stod(readerTC->Get(section_abort, sectionField_dyaw, dyaw_abort)));
    _stopCriteria.set_dSpeed(std::stod(readerTC->Get(section_abort, sectionField_dspeed, dspeed_abort)));

    // Set end criteria:
    std::string time_sec = "";
    _elapsedEndtime = std::stod(readerTC->Get(section_end, sectionField_time_sec, time_sec));

    // Clean up INI reader:
    delete readerTC;
    readerTC = nullptr;
}


} // end of namespace DataGenericItem
