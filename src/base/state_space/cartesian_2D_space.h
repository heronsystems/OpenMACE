#ifndef CARTESIAN_2D_SPACE_H
#define CARTESIAN_2D_SPACE_H

#include <limits>

#include "base/base_global.h"

#include "state_sampler.h"
#include "state_space.h"

#include "base/math/random_number_generator.h"

#include "base/pose/cartesian_position_2D.h"

namespace mace {
namespace state_space {

/**
\* @file  cartesian_2D_space.h
\*
\* @section PROJECT
\*
\*   This is a part of the Heron System's project MACE.
\*
\* @section DESCRIPTION
\*
\*  Header for gui widget displaying test setup progress
\*/

class BASESHARED_EXPORT Cartesian2DSpaceBounds
{
public:
    Cartesian2DSpaceBounds(const double &minX = std::numeric_limits<double>::lowest(), const double &maxX = std::numeric_limits<double>::max(), const double &minY = -std::numeric_limits<double>::lowest(), const double &maxY = std::numeric_limits<double>::max())
    {
        this->min_X = minX;
        this->max_X = maxX;
        this->min_Y = minY;
        this->max_Y = maxY;
    }

    Cartesian2DSpaceBounds(const Cartesian2DSpaceBounds &copy)
    {
        this->min_X = copy.min_X;
        this->max_X = copy.max_X;
        this->min_Y = copy.min_Y;
        this->max_Y = copy.max_Y;
    }

    void getBounds(double &minX, double &maxX, double &minY, double &maxY) const
    {
        minX = this->min_X;
        maxX = this->max_X;
        minY = this->min_Y;
        maxY = this->max_Y;
    }

    void setBounds(const double &minX, const double &maxX, const double &minY, const double &maxY)
    {
        this->min_X = minX;
        this->max_X = maxX;
        this->min_Y = minY;
        this->max_Y = maxY;
    }

    double getMinX() const
    {
        return this->min_X;
    }
    double getMaxX() const
    {
        return this->max_X;
    }
    double getMinY() const
    {
        return this->min_Y;
    }
    double getMaxY() const
    {
        return this->max_Y;
    }

private:
    double max_X;
    double max_Y;
    double min_X;
    double min_Y;
};

MACE_CLASS_FORWARD(Cartesian2DSpace_Sampler);

class BASESHARED_EXPORT Cartesian2DSpace_Sampler: public StateSampler
{
public:
    Cartesian2DSpace_Sampler(const StateSpacePtr space):
        StateSampler(space)
    {

    }

    void sampleUniform(State *state) override;

    void sampleUniformNear(State *sample, const State *near, const double distance = 1) override;

    void sampleGaussian(State *sample, const State *mean, const double stdDev = 0) override;

private:
    math::RandomNumberGenerator m_rng;
};

MACE_CLASS_FORWARD(Cartesian2DSpace);

class BASESHARED_EXPORT Cartesian2DSpace : public StateSpace
{
public:
    Cartesian2DSpace()
    {
        m_type = StateSpaceTypes::CARTESIAN_2D;
        m_name = "Cartesian 2D";
    }

    State* getNewState() const override
    {
        pose::CartesianPosition_2D* newState = new pose::CartesianPosition_2D();
        return newState;
    }

    void removeState(State* state) const override
    {
        delete state;
        state = nullptr;
    }

    void removeStates(std::vector<State*> states) const override
    {
        for(unsigned int i = 0; i < states.size(); i++)
            removeState(states[i]);
    }

    State* copyState(const State* state) const override
    {
        const pose::CartesianPosition_2D* castState = state->stateAs<const pose::CartesianPosition_2D>();
        state_space::State* newState = castState->getStateClone();
        return newState;
    }

    void setBounds(const Cartesian2DSpaceBounds &bounds)
    {
        this->bounds = bounds;
    }

    const Cartesian2DSpaceBounds& getBounds() const
    {
        return bounds;
    }

    double distanceBetween(const State* lhs, const State* rhs) const override;

    bool interpolateStates(const State *begin, const State *end, const double &percentage, State** interState) override;
public:
    Cartesian2DSpaceBounds bounds;
};

} //end of namespace state_space
} //end of namespace mace

#endif // CARTESIAN_2D_SPACE_H
