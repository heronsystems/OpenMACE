#include <Eigen/Core>

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

int main(int argc, char *argv[])
{

    std::cout << "HI" << std::endl;
    Eigen::Vector2d newVector (0,1);
    Eigen::VectorXd tempVecotr;

    newVector = tempVecotr;

    std::cout<<"Pausing here"<<std::endl;

    return 0;
}
