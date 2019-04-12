#ifndef REAL_VECTOR_BOUNDS_H
#define REAL_VECTOR_BOUNDS_H

#include <stdlib.h>
#include <vector>

namespace mace {
namespace state {

class RealVectorBounds
{
public:
    RealVectorBounds(unsigned int dim);

    void setLow(const double &value);
    void setHigh(const double &value);
    void setLow(const unsigned int &index, const double &value);
    void setHigh(const unsigned int &index, const double &value);

    void getBounds(const unsigned int &index, double &valueL, double &valueH);

    std::vector<double> getLowVector()
    {
        return this->low;
    }

    std::vector<double> getHighVector()
    {
        return this->high;
    }
protected:
    std::vector<double> low;
    std::vector<double> high;
};

} //end of namespace state
} //end of namespace mace

#endif // REAL_VECTOR_BOUNDS_H
