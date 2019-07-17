#ifndef RANDOM_NUMBER_GENERATOR_H
#define RANDOM_NUMBER_GENERATOR_H

#include <stdlib.h>
#include <time.h>

#include <random>

#include "common/common.h"


namespace mace {
namespace math {


class RandomNumberGenerator
{
public:
    RandomNumberGenerator();

    double randomReal(const double &lower, const double &upper);

    double uniformReal(const double &lower, const double &upper);

    void sampleRadial(const double &radius, double &theta, double &length);

    double uniform01();


private:

    std::mt19937 generator;
    std::uniform_real_distribution<> uniDist{0,1};
    std::normal_distribution<> normalDist{0,1};
};

} //end of namespace math
} //end of namespace mace

#endif // RANDOM_NUMBER_GENERATOR_H
