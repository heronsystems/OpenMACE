#include "cartesian_2D_space.h"

namespace mace {
namespace state_space {

void Cartesian2DSpace_Sampler::sampleUniform(State *state)
{
    //first grab the bounds at which we are allowed to sample within
    const Cartesian2DSpaceBounds &bounds = static_cast<const Cartesian2DSpace*>(m_space)->getBounds();

    //lets cast the state pointer into the type we are expecting it to be
    pose::CartesianPosition_2D* cast = static_cast<pose::CartesianPosition_2D*>(state);
    cast->setXPosition(m_rng.uniformReal(bounds.getMinX(),bounds.getMaxX()));
    cast->setYPosition(m_rng.uniformReal(bounds.getMinY(),bounds.getMaxY()));
}

void Cartesian2DSpace_Sampler::sampleUniformNear(State *sample, const State *near, const double distance)
{
    UNUSED(sample); UNUSED(near); UNUSED(distance);
}

void Cartesian2DSpace_Sampler::sampleGaussian(State *sample, const State *mean, const double stdDev)
{
    UNUSED(sample); UNUSED(mean); UNUSED(stdDev);
}

double Cartesian2DSpace::distanceBetween(const State *lhs, const State *rhs) const
{
    return lhs->stateAs<pose::CartesianPosition_2D>()->distanceBetween2D(*rhs->stateAs<pose::CartesianPosition_2D>());
}

bool Cartesian2DSpace::interpolateStates(const State *begin, const State *end, const double &percentage, State** interState)
{

    pose::CartesianPosition_2D castBegin(*begin->stateAs<pose::CartesianPosition_2D>());
    pose::CartesianPosition_2D castEnd(*end->stateAs<pose::CartesianPosition_2D>());

    double distance = castBegin.distanceTo(&castEnd);

    /**
     * Let v = (x1,y1) - (x0,y0) and u = v/magnitude(v)
     * New point is now (x0,y0) + d*u
     */
    pose::CartesianPosition_2D v = castEnd - castBegin;
    v.normalize();
    v.scale(distance * percentage);

    *interState = new pose::CartesianPosition_2D(v + castBegin);
    return true;
}


} //end of namespace state_space
} //end of namespace mace
