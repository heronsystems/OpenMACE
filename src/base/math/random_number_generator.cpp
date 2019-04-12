#include "random_number_generator.h"

namespace mace {
namespace math {

RandomNumberGenerator::RandomNumberGenerator()
{
    generator = std::mt19937(time(0));
}

double RandomNumberGenerator::uniform01()
{
    return uniDist(generator);
}

double RandomNumberGenerator::randomReal(const double &lower, const double &upper)
{
    srand(time(NULL));
    double delta = upper - lower;
    return (delta * rand()) + lower;
}

double RandomNumberGenerator::uniformReal(const double &lower, const double &upper)
{
    double delta = upper - lower;
    return (delta * uniDist(generator)) + lower;
}

void RandomNumberGenerator::sampleRadial(const double &radius, double &theta, double &length)
{

}

} //end of namespace math
} //end of namespace mace
