#include <Eigen/Geometry>

#include "maps/data_2d_grid.h"

#include <QCoreApplication>
#include <iostream>
#include <QFile>
#include <QTextStream>
#include <QStringList>


const char kPathSeperator =
        #ifdef _WIN32
        '\\';
#else
        '/';
#endif
using namespace std;

int main(int argc, char *argv[])
{
    double value = 0;
    mace::maps::Data2DGrid<double> newGridMap(&value);
    std::cout<<"Let us pause here"<<std::endl;

    return 0;
}
