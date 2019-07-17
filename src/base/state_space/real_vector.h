#ifndef REAL_VECTOR_H
#define REAL_VECTOR_H

#include <stdlib.h>
#include <vector>
#include <map>

#include "common/common.h"

namespace mace {
namespace state {

class RealVector
{
public:
    RealVector(unsigned int dim = 0);

public:
    void addDimension(double minBound = 0.0, double maxBound = 0.0);

    std::string getDimensionName(const size_t &index) const;

    void setDimensionName(const size_t &index, const std::string &name);


protected:
    unsigned int dimension;
    std::vector<std::string> dimensionNames;
    std::map<std::string, unsigned int> dimensionMap;
    size_t byteSize;

};

} //end of namespace state
} //end of namespace mace

#endif // REAL_VECTOR_H
