#include "random_number_generator.h"

namespace mace {
namespace math {

RandomNumberGenerator::RandomNumberGenerator()
{
    generator = std::mt19937(static_cast<unsigned int>(time(nullptr)));
}

double RandomNumberGenerator::uniform01()
{
    return uniDist(generator);
}

double RandomNumberGenerator::randomReal(const double &lower, const double &upper)
{
    srand(static_cast<unsigned int>(time(nullptr)));
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
    UNUSED(radius); UNUSED(theta); UNUSED(length);
}

} //end of namespace math
} //end of namespace mace
